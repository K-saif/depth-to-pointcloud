import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np


# Read color and depth images
color_raw = o3d.io.read_image("/home/medprime/Downloads/00286/rgb/0000001-000000000000.jpg")
depth_raw = o3d.io.read_image("/home/medprime/Downloads/00286/depth/0000001-000000000000.png")

# Create RGBD image
# Default conversion: color is converted to grayscale and normalized [0, 1]
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, 
    depth_raw, 
    convert_rgb_to_intensity=True
)

# Visualize the RGB-D components using Matplotlib
plt.subplot(1, 2, 1)
plt.title('Grayscale Image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Depth Image')
plt.imshow(rgbd_image.depth)
plt.show()



# Use default intrinsic parameters (e.g., for PrimeSense camera)
intrinsics = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
)

# Create point cloud from the RGBD image
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image, 
    intrinsics
)

# Flip the point cloud to prevent it from being upside down
pcd.transform([[1, 0, 0, 0], 
               [0, -1, 0, 0], 
               [0, 0, -1, 0], 
               [0, 0, 0, 1]])

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])