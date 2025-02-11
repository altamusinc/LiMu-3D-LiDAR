import limu_py
import numpy as np
import open3d as o3d
import cv2
import threading
import time
import io
import tempfile
from flask import Flask, send_file, send_from_directory, render_template
from pynput import keyboard
import json_numpy
from flask_cors import CORS
 
class Lidar:
    lensType = 0
    frequencyModulation = 2
    channel = 0
    hdr_mode = 2
    int0 = 50
    int1 = 400 
    int2 = 4000
    intGr = 10000
    minAmplitude = 60
    lensCenterOffsetX = 0
    lensCenterOffsetY = 0
    roi_leftX = 0
    roi_topY = 0
    roi_rightX = 319
    roi_bottomY = 239

    def __init__(self):
        print("connecting to LIMU")
        self.tof = limu_py.ToF.tof320("10.10.31.180", "50660")
        self.setParameters()

    def streamDistance(self):
        self.tof.streamDistance()
        print("Starting Stream Distance")

    def streamStop(self):
        self.tof.stopStream()
        print("Stopping Stream")

    def setFrameCallback(self, callback):
        self.tof.subscribeFrame(callback)

    def setParameters(self):
        self.tof.setModulation(self.frequencyModulation, self.channel)
        self.tof.setMinAmplitude(self.minAmplitude)
        self.tof.setIntegrationTime(self.int0, self.int1, self.int2, self.intGr)
        self.tof.setHDRMode(self.hdr_mode)
        self.tof.setRoi(self.roi_leftX, self.roi_topY, self.roi_rightX, self.roi_bottomY)
        self.tof.setLensType(self.lensType)
        self.tof.setLensCenter(self.lensCenterOffsetX, self.lensCenterOffsetY)
        self.tof.setFilter(0, 0, 0, 0, 0, 0, 0, 0, 0)

class myPointCloud:
    x_scale = 1
    y_scale = 1
    x_trans = 0
    y_trans = 0

    color_from_cam = False
    o3d_geometry = o3d.geometry.PointCloud()
    np_points = np.zeros(6)

    frame_addresses = {}
    alert_callback = None

    pcdBytes = bytes()

    def handleFrame(self, limu_frame):
        if id(limu_frame) not in self.frame_addresses:
            self.frame_addresses[id(limu_frame)] = limu_frame
            print(f"added frame at address {limu_frame}")

        # convert the frame list to an np array, and reshape to 8 things wide
        xyz_rgb_np = np.array(limu_frame.get_xyz_rgb()).reshape(limu_frame.n_points, 8)
        xyz = xyz_rgb_np[:,:3]
        self.np_points = xyz
        self.o3d_geometry.points = o3d.utility.Vector3dVector(xyz)

        if self.color_from_cam:
            global rgb_cam
            ret, cam_frame = rgb_cam.read()
            # cv2.imshow("frame", cam_frame)
            cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
            cam_frame = cv2.resize(cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
            cam_frame = cv2.flip(cam_frame, 1)
            cam_frame = self.remap_image(cam_frame)
            cam_frame = cam_frame.reshape(76800, 3) / 255
            self.o3d_geometry.colors = o3d.utility.Vector3dVector(cam_frame)
        else:
            self.o3d_geometry.paint_uniform_color([1, 0, 0])
        #     rgb_float = np.array(xyz_rgb_np[:limu_frame.n_points,4])
        #     rgb_int = rgb_float.view(np.uint8)
        #     rgb = rgb_int.reshape(rgb_float.size, 8)
        #     rgb = rgb[:,-3:]/255
        #     self.o3d_geometry.colors = o3d.utility.Vector3dVector(rgb)

        self.o3d_geometry = self.o3d_geometry.remove_non_finite_points()
        self.pcdBytes = o3d.io.write_point_cloud_to_bytes(self.o3d_geometry,format='mem::xyz', write_ascii=False)
        
        if self.alert_callback is not None:
            self.alert_callback()

    def remap_image(self, src):
        map_x = np.zeros((src.shape[0], src.shape[1]), dtype=np.float32)
        map_y = np.zeros((src.shape[0], src.shape[1]), dtype=np.float32)
        for i in range(map_x.shape[0]):
            for j in range(map_x.shape[1]):
                if j > map_x.shape[1]*0 and j < map_x.shape[1]*1 and i > map_x.shape[0]*0 and i < map_x.shape[0]*1:
                    map_x[i,j] = (1/self.x_scale) * (j-map_x.shape[1]*self.y_trans) + 0.5
                    map_y[i,j] = (1/self.y_scale) * (i-map_y.shape[0]*self.x_trans) + 0.5
                else:
                    map_x[i,j] = 0
                    map_y[i,j] = 0
        dst = cv2.remap(src, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return dst


def on_press(key):
    global my_point_cloud
    try:
        if key.char == "c":
            my_point_cloud.color_from_cam = not my_point_cloud.color_from_cam
        
    except AttributeError:
        if key == keyboard.Key.left:
            my_point_cloud.y_trans -= 0.01
        elif key == keyboard.Key.right:
            my_point_cloud.y_trans += 0.01
        elif key == keyboard.Key.up:
            my_point_cloud.x_trans += 0.01
        elif key == keyboard.Key.down:
            my_point_cloud.x_trans -= 0.01
        elif key == keyboard.Key.delete:
            my_point_cloud.x_scale -= 0.01
        elif key == keyboard.Key.page_down:
            my_point_cloud.x_scale += 0.01
        elif key == keyboard.Key.insert:
            my_point_cloud.y_scale -= 0.01
        elif key == keyboard.Key.page_up:
            my_point_cloud.y_scale += 0.01
        print(f"X Scale: {my_point_cloud.x_scale} Y Scale: {my_point_cloud.y_scale} X Trans: {my_point_cloud.x_trans} Y Trans: {my_point_cloud.y_trans}")

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

listener = keyboard.Listener(on_press=on_press,)
# listener.start()

webAPI = Flask(__name__)
CORS(webAPI, resources={r"/*": {"origins": "*"}})

@webAPI.route("/points")
def latestPointsAsPCD():
    foo = io.BytesIO(my_point_cloud.pcdBytes)
    foo.seek(0)
    return send_file(foo, download_name="foo")

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

@webAPI.route("/api/color_default", methods = ['POST'])
def defaultColor():
    my_point_cloud.color_from_cam = False
    return "1"

if __name__ == '__main__':
    webAPI.run(debug=True)
    