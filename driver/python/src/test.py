import limu_py
import numpy as np
import open3d as o3d
import cv2
import io
import time
import json
from flask import Flask, send_file, send_from_directory, render_template, Response
from pynput import keyboard
from flask_cors import CORS
from collections import deque

class BoundedQueue:
    def __init__(self, max_size):
        self.queue = deque(maxlen=max_size)
    
    def put(self, item):
        self.queue.append(item)
    
    def get(self):
        if not self.is_empty():
            return self.queue.popleft()
        else:
            return None

    def is_empty(self):
        return not self.queue
    
    def is_full(self):
        return len(self.queue) == self.queue.maxlen

    def size(self):
        return len(self.queue)
    
class Lidar:

    settings = {"lens_type": 0,
                "frequency_modulation": 2,
                "channel": 0,
                "image_type": 1,
                "hdr_mode": 2,
                "integration_time_tof_1": 50,
                "integration_time_tof_2": 400,
                "integration_time_tof_3": 4000,
                "integration_time_tof_Gr": 10000,
                "min_amplitude": 60,
                "lens_center_offset_x": 0,
                "lens_center_offset_y": 0,
                "roi_left_x": 0,
                "roi_right_x": 319,
                "roi_top_y": 0,
                "roi_bottom_y": 319,
                "roi_height": 0,
                }

    def __init__(self):
        print("connecting to LIMU")
        self.tof = limu_py.ToF.tof320("10.10.31.180", "50660")
        self.applySettings()

    def update_settings_from_json(self, settings_json):
        new_settings_dict: dict = json.loads(settings_json)
        print(new_settings_dict)
        toApply: bool = False
        for key, value in new_settings_dict.items():
            print(f"{key} : {value}") 
            if key in self.settings and value != self.settings[key]:
                self.settings[key] = value
                toApply = True
        if toApply:
            self.applySettings()

    def send_settings_as_json(self) -> str:
        return json.dumps(self.settings)

    def streamDistance(self):
        self.settings["image_type"] = 1
        self.tof.streamDistance()
        print("Starting Stream Distance")

    def streamStop(self):
        self.settings["image_type"] = 0
        self.tof.stopStream()
        print("Stopping Stream")

    def setFrameCallback(self, callback):
        self.tof.subscribeFrame(callback)

    def applySettings(self):
        self.tof.setModulation(self.settings["frequency_modulation"], self.settings["channel"])
        self.tof.setMinAmplitude(self.settings["min_amplitude"])
        self.tof.setIntegrationTime(self.settings["integration_time_tof_1"], self.settings["integration_time_tof_2"], self.settings["integration_time_tof_3"], self.settings["integration_time_tof_Gr"])
        self.tof.setHDRMode(self.settings["hdr_mode"])
        self.tof.setRoi(self.settings["roi_left_x"], self.settings["roi_top_y"], self.settings["roi_right_x"], self.settings["roi_bottom_y"])
        self.tof.setLensType(self.settings["lens_type"])
        self.tof.setLensCenter(self.settings["lens_center_offset_x"], self.settings["lens_center_offset_y"])
        self.tof.setFilter(0, 0, 0, 0, 0, 0, 0, 0, 0)
        match self.settings["image_type"]:
            case 0:
                self.tof.stopStream()
            case 1:
                self.tof.streamDistance()
            case 2:
                self.tof.streamDistanceAmplitude()

class myPointCloud:
    cam_x_scale = 1
    cam_y_scale = 1
    cam_x_trans = 0
    cam_y_trans = 0
    _cam_remap_map = ()

    color_from_cam = False
    last_frame_time = time.perf_counter()

    pcd_queue = BoundedQueue(3)

    frame_addresses = {}
    alert_callback = None

    def __init__(self):
        self._generate_remap_map()

    def handleFrame(self, limu_frame):
        print(f'Frame Received FPS: {1 / (time.perf_counter() - self.last_frame_time):.2f}')
        self.last_frame_time = time.perf_counter()

        t_start = time.perf_counter()
        if id(limu_frame) not in self.frame_addresses:
            self.frame_addresses[id(limu_frame)] = limu_frame
            print(f"added frame at address {limu_frame}")

        pcd = o3d.geometry.PointCloud()
        # convert the frame list to an np array, and reshape to 8 things wide
        xyz_rgb_np = np.array(limu_frame.get_xyz_rgb()).reshape(limu_frame.n_points, 8)
        xyz = xyz_rgb_np[:,:3]
        pcd.points = o3d.utility.Vector3dVector(xyz)

        if self.color_from_cam:
            t_start = time.perf_counter()
            global rgb_cam
            ret, cam_frame = rgb_cam.read()
            # cv2.imshow("frame", cam_frame)
            cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
            cam_frame = cv2.resize(cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
            cam_frame = cv2.flip(cam_frame, 1)
            cam_frame = cv2.remap(cam_frame, self._cam_remap_map[0], self._cam_remap_map[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            cam_frame = cam_frame.reshape(76800, 3) / 255
            pcd.colors = o3d.utility.Vector3dVector(cam_frame)
            t_end = time.perf_counter()
            # print(t_end - t_start)
        else:
            t_start = time.perf_counter()
            rgb_float = np.array(xyz_rgb_np[:limu_frame.n_points,4])
            rgb_int = rgb_float.view(np.uint8)
            rgb = rgb_int.reshape(rgb_float.size, 8)
            rgb = rgb[:,-3:]/255
            pcd.colors = o3d.utility.Vector3dVector(rgb)
            t_end = time.perf_counter()
            # print(t_end - t_start)

        pcd = pcd.remove_non_finite_points()
        self.pcd_queue.put(pcd)

        
        if self.alert_callback is not None:
            self.alert_callback()

        t_end = time.perf_counter()
        print(f"Frame Process Time: {(t_end - t_start) * 1000:.1f} ms")

    def get_latest_pcd(self) -> o3d.geometry.PointCloud:
        pcd = self.pcd_queue.get()
        # If the queue is empty, put the latest cloud back so other clients can access it.
        if self.pcd_queue.is_empty():
            self.pcd_queue.put(pcd)        
        return pcd

    def set_cam_adjustment_params(self, x_translate: float, y_translate: float, x_scale: float, y_scale: float):
        self.cam_x_trans = x_translate
        self.cam_y_trans = y_translate
        self.cam_x_scale = x_scale
        self.cam_y_scale = y_scale
        self._generate_remap_map()

    def _generate_remap_map(self):
        width = 320
        height = 240
        map_x = np.zeros((height, width), dtype=np.float32)
        map_y = np.zeros((height, width), dtype=np.float32)
        for i in range(map_x.shape[0]):
            for j in range(map_x.shape[1]):
                if j > map_x.shape[1]*0 and j < map_x.shape[1]*1 and i > map_x.shape[0]*0 and i < map_x.shape[0]*1:
                    map_x[i,j] = (1/self.cam_x_scale) * (j-map_x.shape[1]*self.cam_y_trans) + 0.5
                    map_y[i,j] = (1/self.cam_y_scale) * (i-map_y.shape[0]*self.cam_x_trans) + 0.5
                else:
                    map_x[i,j] = 0
                    map_y[i,j] = 0
        self._cam_remap_map = (map_x, map_y)

my_point_cloud = myPointCloud()
lidar = Lidar()
rgb_cam = cv2.VideoCapture(0)

# Glue the point cloud handler to the lidar frame callback
lidar.setFrameCallback(my_point_cloud.handleFrame)
lidar.streamDistance()

# Make sure we can open the camera
if not rgb_cam.isOpened():
    print("Error: Could not open camera.")
    exit()

webAPI = Flask(__name__)
CORS(webAPI, resources={r"/*": {"origins": "*"}})

@webAPI.route("/points")
def latestPointsAsPCD():
    foo = io.BytesIO(my_point_cloud.pcdBytes)
    foo.seek(0)
    return send_file(foo, download_name="foo")

@webAPI.route("/asbytes")
def lastestPointsAsBytes():
    t_start = time.perf_counter()
    pcd = my_point_cloud.get_latest_pcd()
    points = np.asarray(pcd.points).astype('float32')
    colors = np.asarray(pcd.colors) * 255
    colors = colors.astype('uint8')
    points_b = points.tobytes()
    colors_b = colors.tobytes()
    combined = points_b + colors_b
    t_end = time.perf_counter()
    # print(f"Pack Time: {(t_end - t_start) * 1000:.1f} ms")
    return Response(combined, mimetype="application/octet-stream")

@webAPI.route("/")
def serveHomepage():
    return render_template("index.html")

@webAPI.route("/<path:filename>")
def serveFile(filename):
    return send_from_directory("./web", filename)

@webAPI.route("/api/on", methods = ['POST'])
def turnOnLidar():
    lidar.streamDistance()
    return "1"

@webAPI.route("/api/off", methods = ['POST'])
def turnOffLidar():
    lidar.streamStop()
    return "1"

@webAPI.route("/api/color_webcam", methods = ['POST'])
def webcamColor():
    my_point_cloud.color_from_cam = True
    return "1"

@webAPI.route("/currentSettings")
def sendCurrentSettings():
    return Response(lidar.send_settings_as_json(), mimetype="application/json")

@webAPI.route("/api/color_default", methods = ['POST'])
def defaultColor():
    my_point_cloud.color_from_cam = False
    return "1"

if __name__ == '__main__':
    webAPI.run(debug=True)
    