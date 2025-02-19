import pickle
import open3d as o3d
import numpy as np

class LimuRawData:
    def __init__(self, distances, amplitude, xyz, rgb):
        self.distances = distances
        self.amplitude = amplitude
        self.rgb = rgb
        self.xyz = xyz

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.xyz)
        self.pcd_from_xyz = pcd

        rgb_reshaped = rgb.reshape(240, 320, 3)
        dist_reshaped = distances.reshape(240, 320)
        rgb_img = o3d.geometry.Image(rgb_reshaped.astype(np.float32)).flip_horizontal()
        dist_img = o3d.geometry.Image(dist_reshaped.astype(np.float32)).flip_horizontal()
        self.rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_img, dist_img, depth_scale = 1, depth_trunc=1000)
        # cam_settings = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        cam_settings = o3d.camera.PinholeCameraIntrinsic(width=320, height=240, fx = 120, fy = 120, cx = 160, cy = 120)
        self.pcd_from_rgbd = o3d.geometry.PointCloud.create_from_rgbd_image(self.rgbd, cam_settings)

with open("frames.pickle", "rb") as file:
    distances = pickle.load(file)
    amplitudes = pickle.load(file)
    xyz = pickle.load(file)
    rgb = pickle.load(file)

frames = []
for i in range(len(distances)):
    frames.append(LimuRawData(distances[i], amplitudes[i], xyz[i], rgb[i]))


def filter_by_amplitude(data: LimuRawData, min_amplitude: float):
    bad_points = (data.amplitude < min_amplitude).nonzero()[0].tolist()
    data.pcd_from_xyz = data.pcd_from_xyz.select_by_index(bad_points, invert=True)
    print("hello")

def filter_outliers(pcd: o3d.geometry.PointCloud, neighbors: int = 20, std_dev: float = 2):
    cl, index = pcd.remove_statistical_outlier(nb_neighbors=neighbors, std_ratio=std_dev)
    return pcd.select_by_index(index)

frame = frames[25]
o3d.visualization.draw_geometries([frame.pcd_from_xyz], "before from xyz")
o3d.visualization.draw_geometries([frame.pcd_from_rgbd], "before from rgbd")
o3d.visualization.draw_geometries([frame.pcd_from_rgbd, frame.pcd_from_xyz], "Together!")

for frame in frames:
    filter_by_amplitude(frame, 25)

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