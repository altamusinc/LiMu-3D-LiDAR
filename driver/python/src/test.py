import limu_py
import numpy as np
import open3d as o3d

frames = {}

vis = o3d.visualization.Visualizer()
geometry = o3d.geometry.PointCloud()
update = False
first = True

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
    
def handleFrame(frame):
    global vis
    global geometry
    global update
    global first
    if id(frame) not in frames:
        frames[id(frame)] = frame
        print("added frame")

    # convert the frame list to an np array, and reshape to 8 things wide
    xyz_rgb_np = np.array(frame.get_xyz_rgb()).reshape(frame.n_points, 8)
    xyz = xyz_rgb_np[:,:3]
    geometry.points = o3d.utility.Vector3dVector(xyz)
    geometry.estimate_normals()
    # pcd_xyz.paint_uniform_color([0, 0, 1])
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
    hdr_mode = 1
    int0 = 1000
    int1 = 1000 
    int2 = 8000
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

tof.subscribeFrame(handleFrame)
# vis.run()
while True:
    if update is True:
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        # print("updated")
        update = False

    # user_input = input("Enter a command (or 'quit' to exit): ")
    # if user_input == "quit":
    #     break
    # # Process the user input
    # print("You entered:", user_input) 