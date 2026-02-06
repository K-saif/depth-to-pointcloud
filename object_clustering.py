import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# 1. Load your PCD file
pcd = o3d.io.read_point_cloud("/home/medprime/Music/depth-to-pointcloud/data/pcd/cloud_bin_0.pcd")

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



# now perform DBSCAN clustering on the objects_cloud
print("Clustering the objects...")
# 1. Assume 'objects_cloud' is the output from your previous Ground Removal step
# If you haven't downsampled, do it now to save processing time
objects_cloud = objects_cloud.voxel_down_sample(voxel_size=0.02)

# 2. Run DBSCAN Clustering
# eps: The distance to neighbors (10cm). If a point is within this distance, it's the same object.
# min_points: Minimum points to form a cluster (prevents noise from being an object).
labels = np.array(objects_cloud.cluster_dbscan(eps=0.1, min_points=50, print_progress=True))

# 3. Handle the labels
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")

# 4. Colorize the clusters
# We use a colormap to give each object a unique color
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0  # Points with label -1 are noise (make them black)
objects_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

# 5. Visualize the result
o3d.visualization.draw_geometries([objects_cloud], 
                                  window_name="DBSCAN Clusters")







