
'''
Here we use PinholeCameraIntrinsicParameters.PrimeSenseDefault as default camera parameter.
It has image resolution 640x480, focal length (fx, fy) = (525.0, 525.0), and optical center (cx, cy) = (319.5, 239.5).
An identity matrix is used as the default extrinsic parameter.
pcd.transform applies an up-down flip transformation on the point cloud for better visualization purpose.
'''


import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np


# Read color and depth images
color_raw = o3d.io.read_image("/livingroom1-color/00006.jpg")
depth_raw = o3d.io.read_image("/livingroom1-depth-clean/00006.png")

# Create RGBD image
# Default conversion: color is converted to grayscale and normalized [0, 1]
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, 
    depth_raw,
    depth_scale=1000.0,
    depth_trunc=3.0,
    convert_rgb_to_intensity=False
)

# Visualize the RGB-D components using Matplotlib
plt.subplot(1, 2, 1)
plt.title('Color Image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Depth Image')
plt.imshow(rgbd_image.depth)
plt.show()


intrinsics= o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

'''
we can also generate point cloud by using given values:
width = 640
height = 480
fx = 525.0  # Focal length in x
fy = 525.0  # Focal length in y
cx = 319.5
cy = 239.5

# Use default intrinsic parameters (e.g., for PrimeSense camera)
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
'''

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


# Assuming you have a point cloud object named 'pcd'
o3d.io.write_point_cloud("/data/pcd/00006.pcd", pcd)


