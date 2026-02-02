import numpy as np
import open3d as o3d

def depth_to_pointcloud(depth, rgb, fx, fy, cx, cy):
    h, w = depth.shape
    points = []
    colors = []

    for v in range(h):
        for u in range(w):
            z = depth[v, u]
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            points.append([x, y, z])
            colors.append(rgb[v, u] / 255.0)

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    pc.colors = o3d.utility.Vector3dVector(colors)

    return pc
