import pickle
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy

class LimuRawData:
    def __init__(self, distances, amplitude, xyz, rgb):
        self.distances = distances
        self.amplitude = amplitude
        self.rgb = rgb
        self.xyz = xyz

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.xyz)
        self.pcd_from_xyz = pcd

        self.rgbd, self.pcd_from_rgbd = create_rgbd_and_pcd(self.distances, self.rgb)

def create_rgbd_and_pcd(distance, color):
    width = 320
    height = 240

    # RGB image
    if color.shape == (width * height, 3):
        rgb_reshaped = color.reshape(height, width, 3)
    # Amplitude as float
    else:
        rgb_reshaped = color.reshape(height, width)
    dist_reshaped = distance.reshape(height, width)
    rgb_img = o3d.geometry.Image(rgb_reshaped.astype(np.float32)).flip_horizontal()
    dist_img = o3d.geometry.Image(dist_reshaped.astype(np.float32)).flip_horizontal()
    
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, dist_img, depth_scale = 1, depth_trunc=1000)

    cam_settings = o3d.camera.PinholeCameraIntrinsic(width=width, height=height, fx = 120, fy = 120, cx = width/2, cy = height/2)
    pcd_from_rgbd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_settings)
    return rgbd, pcd_from_rgbd

def filter_by_amplitude(data: LimuRawData, min_amplitude: float):
    bad_points = (data.amplitude < min_amplitude).nonzero()[0].tolist()
    data.pcd_from_xyz = data.pcd_from_xyz.select_by_index(bad_points, invert=True)

def filter_outliers(pcd: o3d.geometry.PointCloud, neighbors: int = 20, std_dev: float = 2):
    cl, index = pcd.remove_statistical_outlier(nb_neighbors=neighbors, std_ratio=std_dev)
    return pcd.select_by_index(index)

def plot_list(data: list):
    average = np.nanmean(data)
    std_dev = np.nanstd(data)

    # Create the plot
    plt.plot(data, label="Data Points")
    plt.axhline(average, color='r', linestyle='dashed', linewidth=1, label=f"Average: {average:.2f}")
    plt.axhline(average + std_dev, color='g', linestyle='dashed', linewidth=1, label=f"Std Dev: {std_dev:.2f}")
    plt.axhline(average - std_dev, color='g', linestyle='dashed', linewidth=1)

    # Add labels and title
    plt.xlabel("Index")
    plt.ylabel("Value")
    plt.title("Average and Standard Deviation of Data")
    plt.legend()
    plt.show()
# END FUNCTIONS - START SCRIPT

with open("frames.pickle", "rb") as file:
    distances = pickle.load(file)
    amplitudes = pickle.load(file)
    xyz = pickle.load(file)
    rgb = pickle.load(file)

frames = []
for i in range(len(distances)):
    frames.append(LimuRawData(distances[i], amplitudes[i], xyz[i], rgb[i]))

frame = frames[25]
# o3d.visualization.draw_geometries([frame.pcd_from_xyz], "before from xyz")
# o3d.visualization.draw_geometries([frame.pcd_from_rgbd], "before from rgbd")
# o3d.visualization.draw_geometries([frame.pcd_from_rgbd, frame.pcd_from_xyz], "Together!")

random_p1 = []
random_p2 = []
averaged_distance = []
averaged_amplitude = []
averaged_xyz = []
for frame in frames:
    random_p1.append(frame.distances[5000])
    random_p2.append(frame.distances[12000])
    averaged_distance.append(frame.distances)
    averaged_amplitude.append(frame.amplitude)
    averaged_xyz.append(frame.xyz)
    filter_by_amplitude(frame, 25)

averaged_amplitude = np.nanmean(np.array(averaged_amplitude), axis=0)
averaged_distance = np.nanmean(np.array(averaged_distance), axis=0)

# Created Averaged PCD in native units (meters)
averaged_xyz = np.nanmean(np.array(averaged_xyz), axis=0)
avg_pcd = o3d.geometry.PointCloud()
avg_pcd.points = o3d.utility.Vector3dVector(averaged_xyz)

# Grab a single frame from the averages for comparison
single_pcd = frames[25].pcd_from_xyz

# Read point cloud from Disk
hyperion_pcd_feet = o3d.io.read_point_cloud("hyperion_eos_feet.pcd")

# Convert between feet/meters for each permutation
hyperion_pcd_meters = copy.deepcopy(hyperion_pcd_feet)
hyperion_pcd_meters.scale(0.3048, hyperion_pcd_meters.get_center())

averaged_xyz_feet = copy.deepcopy(avg_pcd)
averaged_xyz_feet.scale(3.28084, averaged_xyz_feet.get_center())

single_pcd_feet = copy.deepcopy(single_pcd)
single_pcd_feet.scale(3.28084, single_pcd_feet.get_center())

# Save them to files
o3d.io.write_point_cloud("pcds/hyperion_ft.pcd", hyperion_pcd_feet)
o3d.io.write_point_cloud("pcds/hyperion_m.pcd", hyperion_pcd_meters)
o3d.io.write_point_cloud("pcds/limu_averaged_ft.pcd", averaged_xyz_feet)
o3d.io.write_point_cloud("pcds/limu_averaged_m.pcd", avg_pcd)
o3d.io.write_point_cloud("pcds/limu_single_ft.pcd", single_pcd_feet)
o3d.io.write_point_cloud("pcds/limu_single_m.pcd", single_pcd)

exit()

rgbd, avg_rgbd_pcd = create_rgbd_and_pcd(averaged_distance, averaged_amplitude)
o3d.visualization.draw_geometries([avg_rgbd_pcd], "averaged from RGBD")
o3d.visualization.draw_geometries([avg_pcd], "averaged from xyz")


plot_list(random_p1)
plot_list(random_p2)
frame = frames[25]
o3d.visualization.draw_geometries([frame.pcd_from_xyz], "after amplitude filtration")

combined = o3d.geometry.PointCloud()

for frame in frames:
    combined += frame.pcd_from_xyz
o3d.visualization.draw_geometries([combined], "combined")

combined = filter_outliers(combined, neighbors=200, std_dev=2.0)

o3d.visualization.draw_geometries([combined], "outliers")

combined = combined.voxel_down_sample(0.2)

o3d.visualization.draw_geometries([combined], "voxelized")