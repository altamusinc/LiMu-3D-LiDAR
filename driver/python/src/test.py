import limu_py
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import cv2
import threading
import time
from pynput import keyboard

class myGUI:
    update = False
    def __init__(self, geometry):
        self.geometry = geometry
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Open3D GUI Example", 1024, 768)
        self.scene = gui.SceneWidget()
        self.scene.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)
        self.scene.scene.add_geometry("PointCloud", self.geometry, o3d.visualization.rendering.MaterialRecord())
        update_thread = threading.Thread(target=self.update_geometry_background)
        update_thread.start()

    def run(self):
        self.app.run()

    def handle_new_frame(self):
        self.update = True

    def update_geometry_background(self):
        def update_geometry():
            def update_on_main_thread():
                self.scene.scene.clear_geometry()
                self.scene.scene.add_geometry("PointCloud", self.geometry, o3d.visualization.rendering.MaterialRecord())
            self.app.post_to_main_thread(self.window, update_on_main_thread)
        
        while True:
            if self.update is True:
                update_geometry()
                self.update = False

    def visualize_debug(self, cloud):
        geometries = []
        geometries.append(cloud)
        o3d.visualization.draw_geometries(geometries,
                                    width=1200,height=800,left=2000,top=100,
                                    window_name='Vis',
                                    zoom=0.75,
                                    front=[1,-1,1],
                                    lookat=[0,0,1],
                                    up=[0,0,1],
                                    point_show_normal=False,
                                    mesh_show_wireframe=False,
                                    mesh_show_back_face=False)
        
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
        self.tof = limu_py.ToF.tof320("10.10.31.180", "50660")
        self.setParameters()

    def streamDistance(self):
        self.tof.streamDistance()

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
    geometry = o3d.geometry.PointCloud()
    frame_addresses = {}
    alert_callback = None

    def handleFrame(self, limu_frame):
        if id(limu_frame) not in self.frame_addresses:
            self.frame_addresses[id(limu_frame)] = limu_frame
            print(f"added frame at address {limu_frame}")

        # convert the frame list to an np array, and reshape to 8 things wide
        xyz_rgb_np = np.array(limu_frame.get_xyz_rgb()).reshape(limu_frame.n_points, 8)
        xyz = xyz_rgb_np[:,:3]
        self.geometry.points = o3d.utility.Vector3dVector(xyz)

        if self.color_from_cam:
            global rgb_cam
            ret, cam_frame = rgb_cam.read()
            # cv2.imshow("frame", cam_frame)
            cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
            cam_frame = cv2.resize(cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
            cam_frame = cv2.flip(cam_frame, 1)
            cam_frame = self.remap_image(cam_frame)
            cam_frame = cam_frame.reshape(76800, 3) / 255
            self.geometry.colors = o3d.utility.Vector3dVector(cam_frame)
        else:
            rgb_float = np.array(xyz_rgb_np[:limu_frame.n_points,4])
            rgb_int = rgb_float.view(np.uint8)
            rgb = rgb_int.reshape(rgb_float.size, 8)
            rgb = rgb[:,-3:]/255
            self.geometry.colors = o3d.utility.Vector3dVector(rgb)

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
my_gui = myGUI(my_point_cloud.geometry)
rgb_cam = cv2.VideoCapture(1)

# Glue the point cloud handler to the lidar frame callback
lidar.setFrameCallback(my_point_cloud.handleFrame)

# Glue the GUI to the point cloud handler so it updates when a new point cloud is processed.
my_point_cloud.alert_callback = my_gui.handle_new_frame

# Make sure we can open the camera
if not rgb_cam.isOpened():
    print("Error: Could not open camera.")
    exit()

listener = keyboard.Listener(on_press=on_press,)
listener.start()

lidar.streamDistance()

my_gui.run()