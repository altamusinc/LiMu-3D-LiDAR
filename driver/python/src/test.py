import limu_py
import numpy as np
import open3d as o3d
import cv2
import struct
# import keyboard
from pynput import keyboard

frames = {}

vis = o3d.visualization.Visualizer()
geometry = o3d.geometry.PointCloud()
update = False
first = True

flip = 0
color_from_cam = False

def reinterpret_float_as_int(f):
    return struct.unpack('i', struct.pack('f', f))[0]

def on_press(key):
    global color_from_cam
    global flip
    try:
        if key.char == "c":
            color_from_cam = not color_from_cam
        elif key.char == "5":
            flip = 0
        elif key.char == "6":
            flip = 1
        elif key.char == "7":
            flip = -1
        elif key.char == "8":
            flip = 2
        
    except AttributeError:
        print('special key {0} pressed'.format(key))

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
    
def handleFrame(limu_frame):
    global vis
    global geometry
    global update
    global first
    global flip
    global color_from_cam

    if id(limu_frame) not in frames:
        frames[id(limu_frame)] = limu_frame
        print("added frame")

    # convert the frame list to an np array, and reshape to 8 things wide
    xyz_rgb_np = np.array(limu_frame.get_xyz_rgb()).reshape(limu_frame.n_points, 8)
    xyz = xyz_rgb_np[:,:3]
    geometry.points = o3d.utility.Vector3dVector(xyz)

    if color_from_cam:
        ret, cam_frame = cap.read()
        cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
        cam_frame = cv2.resize(cam_frame, (320, 240), interpolation=cv2.INTER_AREA)
        cam_frame = cam_frame.reshape(76800, 3) / 255
        if flip != 2:
            cam_frame = cv2.flip(cam_frame, 1)
            cam_frame = cv2.flip(cam_frame, flip)
        geometry.colors = o3d.utility.Vector3dVector(cam_frame)
    else:
        rgb_float = np.array(xyz_rgb_np[:limu_frame.n_points,4])
        rgb_int = rgb_float.view(np.uint8)
        rgb = rgb_int.reshape(rgb_float.size, 8)
        rgb = rgb[:,-3:]/255
        geometry.colors = o3d.utility.Vector3dVector(rgb)

    if first:
        vis.add_geometry(geometry)
        first = False
        print("Adding")

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
vis.create_window(width=1200,height=800,left=2000,top=100)
opt = vis.get_render_option()
opt.point_size = 3
opt.light_on = True
opt.show_coordinate_frame = True
opt.background_color = np.asarray([0.25, 0.25, 0.25])
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

tof.subscribeFrame(handleFrame)

listener = keyboard.Listener(on_press=on_press,)
listener.start()

while True:
    if update is True:
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        update = False
