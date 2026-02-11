import open3d as o3d
import numpy as np

# 1. Load your PCD file
pcd = o3d.io.read_point_cloud("/data/pcd/cloud_bin_0.pcd")

# 2. Downsample to speed up the RANSAC process
# 2cm voxels are usually enough to maintain plane accuracy
pcd_down = pcd.voxel_down_sample(voxel_size=0.02)

print("Segmenting the floor...")
# 3. Run RANSAC Plane Segmentation
# distance_threshold: 5cm (points within 5cm of the plane are 'ground')
plane_model, inliers = pcd_down.segment_plane(
    distance_threshold=0.01,
    ransac_n=3,
    num_iterations=1000
)

[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# 4. Extract the Ground (Inliers)
ground_cloud = pcd_down.select_by_index(inliers)
ground_cloud.paint_uniform_color([1, 0, 0])  # Paint it Red for clarity

# 5. Extract the Objects/Walls (Outliers) 
# The 'invert=True' is the key move to REMOVE the ground
objects_cloud = pcd_down.select_by_index(inliers, invert=True)
objects_cloud.paint_uniform_color([0.5, 0.5, 0.5]) # Paint objects Grey

print(f"Removed {len(inliers)} points belonging to the ground.")

# 6. Visualize the separation
# We show both to confirm the "cut" was clean
o3d.visualization.draw_geometries([objects_cloud, ground_cloud],
                                  window_name="Ground Removal (Red = Ground, Grey = Objects)")