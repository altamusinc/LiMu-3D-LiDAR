import limu_py
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import cv2
import threading
import time
from pynput import keyboard

frames = {}

geometry = o3d.geometry.PointCloud()
update = False
first = True

color_from_cam = False
cam_frame_raw = None

x_scale = 1
y_scale = 1
x_trans = 0
y_trans = 0


app = gui.Application.instance
app.initialize()
window = app.create_window("Open3D GUI Example", 1024, 768)
scene = gui.SceneWidget()
scene.scene = o3d.visualization.rendering.Open3DScene(window.renderer)
window.add_child(scene)
scene.scene.add_geometry("PointCloud", geometry, o3d.visualization.rendering.MaterialRecord())

def on_press(key):
    global color_from_cam
    global x_scale
    global x_trans
    global y_scale
    global y_trans
    try:
        if key.char == "c":
            color_from_cam = not color_from_cam
        
    except AttributeError:
        if key == keyboard.Key.left:
            y_trans -= 0.01
        elif key == keyboard.Key.right:
            y_trans += 0.01
        elif key == keyboard.Key.up:
            x_trans += 0.01
        elif key == keyboard.Key.down:
            x_trans -= 0.01
        elif key == keyboard.Key.delete:
            x_scale -= 0.01
        elif key == keyboard.Key.page_down:
            x_scale += 0.01
        elif key == keyboard.Key.insert:
            y_scale -= 0.01
        elif key == keyboard.Key.page_up:
            y_scale += 0.01
        print(f"X Scale: {x_scale} Y Scale: {y_scale} X Trans: {x_trans} Y Trans: {y_trans}")

def update_geometry_background():
    global update
    global geometry
    global app
    global window

    def update_geometry():
        def update_on_main_thread():
            scene.scene.clear_geometry()
            scene.scene.add_geometry("PointCloud", geometry, o3d.visualization.rendering.MaterialRecord())
        app.post_to_main_thread(window, update_on_main_thread)
    
    while True:
        if update is True:
            update_geometry()
            update = False

def visualize(cloud):
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

def remap_image(src):
    global x_scale
    global x_trans
    global y_scale
    global y_trans
    map_x = np.zeros((src.shape[0], src.shape[1]), dtype=np.float32)
    map_y = np.zeros((src.shape[0], src.shape[1]), dtype=np.float32)
    for i in range(map_x.shape[0]):
        for j in range(map_x.shape[1]):
            if j > map_x.shape[1]*0 and j < map_x.shape[1]*1 and i > map_x.shape[0]*0 and i < map_x.shape[0]*1:
                map_x[i,j] = (1/x_scale) * (j-map_x.shape[1]*y_trans) + 0.5
                map_y[i,j] = (1/y_scale) * (i-map_y.shape[0]*x_trans) + 0.5
            else:
                map_x[i,j] = 0
                map_y[i,j] = 0
    dst = cv2.remap(src, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return dst
        
def handleFrame(limu_frame):
    global geometry
    global update
    global color_from_cam
    global cam_frame_raw

    if id(limu_frame) not in frames:
        frames[id(limu_frame)] = limu_frame
        print("added frame")

    # convert the frame list to an np array, and reshape to 8 things wide
    xyz_rgb_np = np.array(limu_frame.get_xyz_rgb()).reshape(limu_frame.n_points, 8)
    xyz = xyz_rgb_np[:,:3]
    geometry.points = o3d.utility.Vector3dVector(xyz)

    if color_from_cam:
        ret, cam_frame = cap.read()
        # cv2.imshow("frame", cam_frame)
        cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
        cam_frame = cv2.resize(cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
        cam_frame = cv2.flip(cam_frame, 1)
        cam_frame = remap_image(cam_frame)
        cam_frame = cam_frame.reshape(76800, 3) / 255
        geometry.colors = o3d.utility.Vector3dVector(cam_frame)
    else:
        rgb_float = np.array(xyz_rgb_np[:limu_frame.n_points,4])
        rgb_int = rgb_float.view(np.uint8)
        rgb = rgb_int.reshape(rgb_float.size, 8)
        rgb = rgb[:,-3:]/255
        geometry.colors = o3d.utility.Vector3dVector(rgb)

    update = True

def setParameters():
    global tof

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

    tof.setModulation(frequencyModulation, channel)
    tof.setMinAmplitude(minAmplitude)
    tof.setIntegrationTime(int0, int1, int2, intGr)
    tof.setHDRMode(hdr_mode)
    tof.setRoi(roi_leftX, roi_topY, roi_rightX, roi_bottomY)
    tof.setLensType(lensType)
    tof.setLensCenter(lensCenterOffsetX, lensCenterOffsetY)
    tof.setFilter(0, 0, 0, 0, 0, 0, 0, 0, 0)

tof = limu_py.ToF.tof320("10.10.31.180", "50660")
setParameters()
tof.streamDistance()
tof.subscribeFrame(handleFrame)

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

listener = keyboard.Listener(on_press=on_press,)
listener.start()

update_thread = threading.Thread(target=update_geometry_background)
update_thread.start()
app.run()