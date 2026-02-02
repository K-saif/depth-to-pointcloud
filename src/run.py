import cv2
import yaml
import open3d as o3d
from disparity import compute_disparity
from depth import disparity_to_depth
from pointcloud import depth_to_pointcloud

# Load images
left = cv2.imread("/home/medprime/Music/depth-to-pointcloud/data/left.png")
right = cv2.imread("/home/medprime/Music/depth-to-pointcloud/data/right.png")

# Load camera params
with open("/home/medprime/Music/depth-to-pointcloud/config/camera.yaml") as f:
    cam = yaml.safe_load(f)

# Compute disparity
disparity = compute_disparity(left, right)
cv2.imwrite("/home/medprime/Music/depth-to-pointcloud/output/disparity.png", (disparity / disparity.max() * 255))

# Depth
depth = disparity_to_depth(disparity, cam["fx"], cam["baseline"])

# Point cloud
pc = depth_to_pointcloud(
    depth, left,
    cam["fx"], cam["fy"],
    cam["cx"], cam["cy"]
)

o3d.io.write_point_cloud("/home/medprime/Music/depth-to-pointcloud/output/cloud.ply", pc)
o3d.visualization.draw_geometries([pc])
