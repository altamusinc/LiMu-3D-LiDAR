import limu_py
import numpy as np
import open3d as o3d
import cv2
import io
import time
import json
from flask import Flask, request, send_file, send_from_directory, render_template, Response
from flask_cors import CORS
from collections import deque
import matplotlib.pyplot as plt
import threading

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
    def __init__(self):
        print("connecting to LIMU")
        self.tof = limu_py.ToF.tof320("10.10.31.180", "50660")
        self._lens_type = 0  
        self._frequency_modulation = 0
        self._channel = 0
        self._image_type = 2
        self._hdr_mode = 2
        self._integration_time_tof_1 = 50
        self._integration_time_tof_2 = 400
        self._integration_time_tof_3 = 8000
        self._integration_time_tof_Gr = 10000
        self._min_amplitude = 0
        self._lens_center_offset_x = 0
        self._lens_center_offset_y = 0
        self._roi_left_x = 0
        self._roi_right_x = 319
        self._roi_top_y = 0
        self._roi_bottom_y = 239
        self._filter_median_filter: bool = True
        self._filter_average_filter: bool = True
        self._filter_temporal_factor: int = 0
        self._filter_temporal_threshold: int = 0
        self._filter_edge_threshold: int = 0
        self._filter_temporal_edge_threshold_low: int = 0
        self._filter_temporal_edge_threshold_high: int = 0
        self._filter_interference_detection_limit: int = 0
        self._filter_interference_detection_use_last_value: bool = False
        self.apply_all_settings()

    @property
    def lens_type(self):
        return self._lens_type
    
    @lens_type.setter
    def lens_type(self, val):
        self._lens_type = val
        print(f"Setting Lens Type to {val}")
        self.tof.setLensType(val)

    @property
    def frequency_modulation(self):
        return self._frequency_modulation
    
    @frequency_modulation.setter
    def frequency_modulation(self, val):
        self._frequency_modulation = val
        self.apply_modulation_and_channel_settings()
    
    @property
    def channel(self):
        return self._channel

    @channel.setter
    def channel(self, val):
        self._channel = val
        self.apply_modulation_and_channel_settings()
    
    @property
    def hdr_mode(self):
        return self._hdr_mode
    
    @hdr_mode.setter
    def hdr_mode(self, val):
        self._hdr_mode = val
        print(f"Setting HDR Mode to {val}")
        self.tof.setHDRMode(val)

    @property
    def integration_time_tof_1(self):
        return self._integration_time_tof_1
    
    @integration_time_tof_1.setter
    def integration_time_tof_1(self, val):
        self._integration_time_tof_1 = val
        self.apply_integration_time_settings()

    @property
    def integration_time_tof_2(self):
        return self._integration_time_tof_2
    
    @integration_time_tof_2.setter
    def integration_time_tof_2(self, val):
        self._integration_time_tof_2 = val
        self.apply_integration_time_settings()

    @property
    def integration_time_tof_3(self):
        return self._integration_time_tof_3
    
    @integration_time_tof_3.setter
    def integration_time_tof_3(self, val):
        self._integration_time_tof_3 = val
        self.apply_integration_time_settings()
    
    @property
    def integration_time_tof_Gr(self):
        return self._integration_time_tof_Gr
    
    @integration_time_tof_Gr.setter
    def integration_time_tof_Gr(self, val):
        self._integration_time_tof_Gr = val
        self.apply_integration_time_settings()

    @property
    def min_amplitude(self):
        return self._min_amplitude
    
    @min_amplitude.setter
    def min_amplitude(self, val):
        self._min_amplitude = val
        print(f"Setting Min Amplitude to {val}")
        self.tof.setMinAmplitude(val)

    @property
    def lens_center_offset_x(self):
        return self._lens_center_offset_x

    @lens_center_offset_x.setter
    def lens_center_offset_x(self, val):
        self._lens_center_offset_x = val
        self.apply_lens_offset_settings()

    @property
    def lens_center_offset_y(self):
        return self._lens_center_offset_y

    @lens_center_offset_y.setter
    def lens_center_offset_y(self, val):
        self._lens_center_offset_y = val
        self.apply_lens_offset_settings()

    @property
    def roi_left_x(self):
        return self._roi_left_x

    @roi_left_x.setter
    def roi_left_x(self, val):
        self._roi_left_x = val
        self.apply_roi_settings()

    @property
    def roi_right_x(self):
        return self._roi_right_x

    @roi_right_x.setter
    def roi_right_x(self, val):
        self._roi_right_x = val
        self.apply_roi_settings()

    @property
    def roi_top_y(self):
        return self._roi_top_y

    @roi_top_y.setter
    def roi_top_y(self, val):
        self._roi_top_y = val
        self.apply_roi_settings()

    @property
    def roi_bottom_y(self):
        return self._roi_bottom_y

    @roi_bottom_y.setter
    def roi_bottom_y(self, val):
        self._roi_bottom_y = val
        self.apply_roi_settings()

    @property
    def image_type(self):
        return self._image_type

    @image_type.setter
    def image_type(self, val):
        self._image_type = val
        match val:
            case 0:
                print("Stopping Stream")
                self.tof.stopStream()
            case 1:
                print("Streaming Distance")
                self.tof.streamDistance()
            case 2:
                print("Streaming Distance/Amplitude")
                self.tof.streamDistanceAmplitude()

    @property
    def filter_median_filter(self):
        return self._filter_median_filter
    
    @filter_median_filter.setter
    def filter_median_filter(self, val: bool):
        self._filter_median_filter = val
        self.apply_filter_settings()
        
    @property
    def filter_average_filter(self):
        return self._filter_average_filter

    @filter_average_filter.setter
    def filter_average_filter(self, val: bool):
        self._filter_average_filter = val
        self.apply_filter_settings()
        
    @property
    def filter_temporal_factor(self):
        return self._filter_temporal_factor

    @filter_temporal_factor.setter
    def filter_temporal_factor(self, val: int):
        self._filter_temporal_factor = val
        self.apply_filter_settings()
        
    @property
    def filter_temporal_threshold(self):
        return self._filter_temporal_threshold

    @filter_temporal_threshold.setter
    def filter_temporal_threshold(self, val: int):
        self._filter_temporal_threshold = val
        self.apply_filter_settings()
        
    @property
    def filter_edge_threshold(self):
        return self._filter_edge_threshold

    @filter_edge_threshold.setter
    def filter_edge_threshold(self, val: int):
        self._filter_edge_threshold = val
        self.apply_filter_settings()
        
    @property
    def filter_temporal_edge_threshold_low(self):
        return self._filter_temporal_edge_threshold_low

    @filter_temporal_edge_threshold_low.setter
    def filter_temporal_edge_threshold_low(self, val: int):
        self._filter_temporal_edge_threshold_low = val
        self.apply_filter_settings()
        
    @property
    def filter_temporal_edge_threshold_high(self):
        return self._filter_temporal_edge_threshold_high

    @filter_temporal_edge_threshold_high.setter
    def filter_temporal_edge_threshold_high(self, val: int):
        self._filter_temporal_edge_threshold_high = val
        self.apply_filter_settings()

    @property
    def filter_interference_detection_limit(self):
        return self._filter_interference_detection_limit

    @filter_interference_detection_limit.setter
    def filter_interference_detection_limit(self, val: int):
        self._filter_interference_detection_limit = val
        self.apply_filter_settings()
        
    @property
    def filter_interference_detection_use_last_value(self):
        return self._filter_interference_detection_use_last_value

    @filter_interference_detection_use_last_value.setter
    def filter_interference_detection_use_last_value(self, val: bool):
        self._filter_interference_detection_use_last_value = val
        self.apply_filter_settings()
        

    def apply_roi_settings(self):
        self.tof.setRoi(self.roi_left_x, self.roi_top_y, self.roi_right_x, self.roi_bottom_y)

    def apply_modulation_and_channel_settings(self):
        print(f"Setting modulation to {self.frequency_modulation} and channel to {self.channel}")
        self.tof.setModulation(self.frequency_modulation, self.channel)

    def apply_integration_time_settings(self):
        print(f"Applying Integration Times: 1: {self.integration_time_tof_1} 2: {self.integration_time_tof_2} 3: {self.integration_time_tof_3} GR: {self.integration_time_tof_Gr}")
        self.tof.setIntegrationTime(self.integration_time_tof_1, self.integration_time_tof_2, self.integration_time_tof_3, self.integration_time_tof_Gr)

    def apply_lens_offset_settings(self):
        print(f"Applying Lens offset X: {self.lens_center_offset_x} Y: {self.lens_center_offset_y}")
        self.tof.setLensCenter(self.lens_center_offset_x, self.lens_center_offset_y)

    def apply_filter_settings(self):
        print("Applying filter settings")
        self.tof.setFilter(self.filter_median_filter, 
                           self.filter_average_filter, 
                           self.filter_temporal_factor, 
                           self.filter_temporal_threshold, 
                           self.filter_edge_threshold, 
                           self.filter_temporal_edge_threshold_low, 
                           self.filter_temporal_edge_threshold_high, 
                           self.filter_interference_detection_limit, 
                           self.filter_interference_detection_use_last_value)

    def apply_all_settings(self):
        self.apply_modulation_and_channel_settings()
        self.min_amplitude = self.min_amplitude
        self.apply_integration_time_settings()
        self.hdr_mode = self.hdr_mode
        self.apply_roi_settings()
        self.lens_type = self.lens_type
        self.apply_lens_offset_settings()
        self.apply_filter_settings
        self.image_type = self.image_type

    def update_settings_from_json(self, settings_json):
        res: bool = True
        new_settings_dict: dict = json.loads(settings_json)
        for key, value in new_settings_dict.items():
            if hasattr(self, key):
                if getattr(self, key) != value:
                    setattr(self, key, value)
            else:
                print(f"I don't have an attribute named {key}")
                res = False
        return res

    def settings_as_dict(self) -> dict:
        return {"lens_type": self.lens_type,
                "frequency_modulation": self.frequency_modulation,
                "channel": self.channel,
                "image_type": self.image_type,
                "hdr_mode": self.hdr_mode,
                "integration_time_tof_1": self.integration_time_tof_1,
                "integration_time_tof_2": self.integration_time_tof_2,
                "integration_time_tof_3": self.integration_time_tof_3,
                "integration_time_tof_Gr": self.integration_time_tof_Gr,
                "min_amplitude": self.min_amplitude,
                "lens_center_offset_x": self.lens_center_offset_x,
                "lens_center_offset_y": self.lens_center_offset_y,
                "roi_left_x": self.roi_left_x,
                "roi_right_x": self.roi_right_x,
                "roi_top_y": self.roi_top_y,
                "roi_bottom_y": self.roi_bottom_y}

    def streamDistance(self):
        self.image_type = 1
        print("Starting Stream Distance")

    def streamStop(self):
        self.image_type = 0
        print("Stopping Stream")

    def setFrameCallback(self, callback):
        self.tof.subscribeFrame(callback)

class myPointCloud:
    cam_x_scale = .9
    cam_y_scale = .9
    cam_x_trans = 0
    cam_y_trans = 0
    _cam_remap_map = ()

    point_color = 0
    last_frame_time = time.perf_counter()

    pcd_queue = BoundedQueue(3)

    frame_addresses = {}
    alert_callback = None

    def __init__(self):
        self._generate_remap_map()
    
    def settings_as_dict(self) -> dict:
        return {"point_color": self.point_color,
                "cam_x_scale": float(self.cam_x_scale),
                "cam_y_trans": float(self.cam_y_scale),
                "cam_x_trans": float(self.cam_x_trans),
                "cam_y_trans": float(self.cam_y_trans),
                }

    def handleFrame(self, limu_frame):
        print(f'Frame Received FPS: {1 / (time.perf_counter() - self.last_frame_time):.2f}')
        self.last_frame_time = time.perf_counter()

        t_start = time.perf_counter()
        if id(limu_frame) not in self.frame_addresses:
            self.frame_addresses[id(limu_frame)] = limu_frame
            print(f"added frame at address {limu_frame}")

        # convert the frame list to an np array, and reshape to 8 things wide
        xyz_rgb_np = np.array(limu_frame.get_xyz_rgb(), dtype=np.float32).reshape(limu_frame.n_points, 8)
        xyz = xyz_rgb_np[:,:3]
        colors = []

        match self.point_color:
            case 0:
                # Webcam
                t_start = time.perf_counter()
                global rgb_cam
                ret, cam_frame = rgb_cam.read()
                # cv2.imshow("frame", cam_frame)
                cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
                cam_frame = cv2.resize(
                    cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
                cam_frame = cv2.flip(cam_frame, 1)
                cam_frame = cv2.remap(
                    cam_frame, self._cam_remap_map[0], self._cam_remap_map[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                cam_frame = cam_frame.reshape(76800, 3) / 255
                colors = cam_frame
                t_end = time.perf_counter()
                # print(t_end - t_start)
            case 1:
                # Distance
                t_start = time.perf_counter()
                rgb = np.array(xyz_rgb_np[:limu_frame.n_points, 4]) # get the combined RGB float from the 5th byte of the xyz_rgb array
                rgb = np.frombuffer(rgb.tobytes(), dtype=np.uint8) # Convert it to a big ol list of uint8 bytes
                rgb = rgb.reshape(limu_frame.n_points, 4) # Reshape it back into 2d array matching the points array
                rgb = rgb[:,:3] # The first byte is unused, float is 4 bytes, we only have r/g/b
                rgb = rgb[:,::-1] # They come in reversed, so flip it around to be rgb instead of bgr
                rgb = rgb.astype("float32") # o3d wants colors as 0-1 floats, convert our 0-255 values to float
                colors = rgb / 255
                t_end = time.perf_counter()
                # print(t_end - t_start)
            case 2:
                # Amplitude
                amplitude = np.array(limu_frame.get_amplitude_data(), dtype=np.float32)
                norm_min = np.nanmin(amplitude)
                norm_max = np.nanmax(amplitude)/5
                normalizer = plt.Normalize(norm_min, norm_max)
                normalized = normalizer(amplitude)
                cmap = plt.get_cmap("plasma")
                colors = cmap(normalized)[:,0:3]
            case 3 | 4 | 5:
                # Color by x y or z
                i = None
                match self.point_color:
                    # Z
                    case 3:
                        i = xyz[:,2] # slice off z from xyz points
                    case 4:
                        i = xyz[:,1] # slice off y 
                    case 5:
                        i = xyz[:,0]
                            
                norm = plt.Normalize(np.nanmin(i), np.nanmax(i))
                cmap = plt.get_cmap("coolwarm")
                colors = cmap(norm(i))[:,0:3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)
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
# lidar.streamDistance()

# Make sure we can open the camera
if not rgb_cam.isOpened():
    print("Error: Could not open camera.")
    exit()

webAPI = Flask(__name__, template_folder="./")
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
    return send_from_directory("./", filename)

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

@webAPI.route("/applySettings", methods=['POST'])
def applySettings():
    print(request.data)
    if lidar.update_settings_from_json(request.data):
        d = lidar.settings_as_dict()
        d.update(my_point_cloud.settings_as_dict())
        return Response(json.dumps(d), mimetype="application/json")
    else:
        return "Error"

@webAPI.route("/displaySettings", methods=['POST'])
def displaySettings():
    print(request.data)
    display_settings = json.loads(request.data)
    if "point_color" in display_settings:
        print("setting point color")
        my_point_cloud.point_color = display_settings["point_color"]
    d = lidar.settings_as_dict()
    d.update(my_point_cloud.settings_as_dict())
    return Response(json.dumps(d), mimetype="application/json")

@webAPI.route("/currentSettings")
def sendCurrentSettings():
    d = lidar.settings_as_dict()
    d.update(my_point_cloud.settings_as_dict())
    return Response(json.dumps(d), mimetype="application/json")

@webAPI.route("/api/color_default", methods = ['POST'])
def defaultColor():
    my_point_cloud.color_from_cam = False
    return "1"

def handle_input():
    while True:
        command = input("Enter Input: ")
        print(f"Entered {command}")

input_thread = threading.Thread(target=handle_input)
input_thread.start()

if __name__ == '__main__':
    webAPI.run(debug=True)
    