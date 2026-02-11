import open3d as o3d

def save_frame_as_pcd(color_path, depth_path, output_filename, intrinsics):
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)
    
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False
    )
    
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)
    
    # Standardize orientation (the flip you used before)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    # Optional: Downsample to make ICP faster (highly recommended)
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    
    # Save to disk
    o3d.io.write_point_cloud(output_filename, pcd)
    print(f"Saved: {output_filename}")

# Define your intrinsics as you did before
intrinsics = o3d.camera.PinholeCameraIntrinsic(640, 480, 525.0, 525.0, 319.5, 239.5)

# Save the two frames the tutorial expects
save_frame_as_pcd("/livingroom1-color/00000.jpg", "/livingroom1-depth-clean/00000.png", "/data/pcd/cloud_bin_0.pcd", intrinsics)
save_frame_as_pcd("/livingroom1-color/00001.jpg", "/livingroom1-depth-clean/00001.png", "/data/pcd/cloud_bin_1.pcd", intrinsics)