# 🌐 Depth to Point Cloud - Advanced 3D Scene Processing Toolkit

Convert RGB-D images into 3D point clouds with tools for registration, ground removal, and object clustering

## Overview
This project provides a comprehensive toolkit for converting depth images into 3D point clouds and performing advanced point cloud processing operations. It leverages Open3D to enable depth-to-point-cloud conversion, point cloud registration, ground plane removal, and object clustering. This is particularly useful in applications such as robotics, 3D scene reconstruction, computer vision, autonomous navigation, and augmented reality.

## Features
- **Depth-to-Point Cloud Conversion**: Convert RGB-D images (color + depth) into 3D point clouds
- **Point Cloud Registration**: Align multiple point clouds using Iterative Closest Point (ICP)
  - Point-to-point ICP registration
  - Point-to-plane ICP registration
- **Ground Plane Removal**: Automatically segment and remove ground planes using RANSAC plane detection
- **Object Clustering**: Identify and cluster distinct objects in scenes using DBSCAN clustering
- **Point Cloud Visualization**: Interactive 3D visualization of point clouds
- **Downsampling**: Voxel-based downsampling for performance optimization
- **PCD Format Support**: Save and load point clouds in PCD (Point Cloud Data) format

## Project Structure
```
depth-to-pointcloud/
├── README.md                                      # Project documentation
├── create_pcd.py                                  # Create point clouds from image pairs
├── generate_PC.py                                 # Generate point clouds from RGB-D images
├── ground_removal.py                              # Remove ground planes using RANSAC
├── object_clustering.py                           # Cluster objects using DBSCAN
├── Point_Cloud_Registration_using_Open3D.ipynb    # ICP registration tutorial notebook
├── data/
│   ├── pcd/                                       # Point cloud data files (.pcd)
│   └── pcl/                                       # Additional point cloud files
└── requirements.txt                               # Python dependencies
```

## Installation

### Prerequisites
- Python 3.7 or higher
- pip package manager

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/depth-to-pointcloud.git
cd depth-to-pointcloud
```

### Step 2: Install Dependencies
```bash
pip install -r requirements.txt
```

**Key Dependencies:**
- **Open3D**: 3D data processing library for point cloud operations
- **NumPy**: Numerical computing
- **Matplotlib**: Data visualization
- **OpenCV**: Image processing (if needed)

## Usage

### 1. Generate Point Clouds from RGB-D Images

Convert color and depth images into point clouds:

```python
python generate_PC.py
```

This script reads color and depth images, creates an RGBD image, and generates a point cloud with proper camera intrinsics.

**Example code snippet:**
```python
import open3d as o3d

# Read color and depth images
color_raw = o3d.io.read_image("path/to/color.jpg")
depth_raw = o3d.io.read_image("path/to/depth.png")

# Create RGBD image
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, depth_scale=1000.0, depth_trunc=3.0
)

# Define camera intrinsics (PrimeSense default: 640x480)
intrinsics = o3d.camera.PinholeCameraIntrinsic(640, 480, 525.0, 525.0, 319.5, 239.5)

# Create point cloud
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)

# Save to disk
o3d.io.write_point_cloud("output.pcd", pcd)
```

### 2. Create Point Clouds from Multiple Frames

Batch process multiple image pairs:

```python
python create_pcd.py
```

This generates point cloud files (`.pcd`) for each color-depth image pair.

### 3. Remove Ground Plane

Segment and remove ground planes from point clouds:

```python
python ground_removal.py
```

**Features:**
- Uses RANSAC plane detection to identify the ground plane
- Separates objects from ground/walls
- Downsamples point clouds for faster processing
- Visualizes results with color-coded separation

### 4. Cluster Objects

Identify distinct objects in the scene using DBSCAN clustering:

```python
python object_clustering.py
```

**Features:**
- Performs ground removal first
- Applies DBSCAN clustering with configurable parameters
- Colors each cluster uniquely
- Outputs cluster statistics

### 5. Point Cloud Registration (ICP)

Register and align multiple point clouds:

Open and run the Jupyter notebook:

```bash
jupyter notebook Point_Cloud_Registration_using_Open3D.ipynb
```

**Supported Methods:**
- **Point-to-Point ICP**: Direct point correspondence
- **Point-to-Plane ICP**: Uses surface normals for better alignment

## Configuration Parameters

### Camera Intrinsics
The default camera intrinsics (PrimeSense) are:
- Image Resolution: 640 × 480
- Focal Length: fx = 525.0, fy = 525.0
- Principal Point: cx = 319.5, cy = 239.5

Modify these values in the scripts to match your camera specifications.

### Common Parameters
- **depth_scale**: Scale factor for depth values (default: 1000.0)
- **depth_trunc**: Maximum depth threshold in meters (default: 3.0)
- **voxel_size**: Voxel size for downsampling (default: 0.02 m)
- **eps**: DBSCAN cluster radius (default: 0.1 m)
- **min_points**: Minimum points per cluster (default: 50)

## Requirements

See [requirements.txt](requirements.txt) for the complete list of dependencies. Main packages:

```
open3d>=0.16.0
numpy>=1.19.0
matplotlib>=3.3.0
```

## Example Workflow

A typical workflow for scene processing:

1. **Capture** RGB-D images from your camera
2. **Generate** point clouds using `generate_PC.py`
3. **Remove Ground** using `ground_removal.py` to isolate objects
4. **Cluster Objects** using `object_clustering.py` for object segmentation
5. **Register** multiple point clouds if needed using the ICP notebook
6. **Visualize** and analyze results in Open3D viewer

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- [Open3D](http://www.open3d.org/) - An Open Source Library for 3D Data Processing
- [NumPy](https://numpy.org/) - The fundamental package for array computing
- [Matplotlib](https://matplotlib.org/) - Visualization library for Python
- [PrimeSense](https://www.primesense.com/) - For default camera intrinsic parameters

## References
- [Open3D Documentation](https://www.open3d.org/docs/latest/)
- [Open3D ICP Registration Tutorial](https://www.open3d.org/html/tutorial/Basic/icp_registration.html)
- [Point Cloud Registration](https://en.wikipedia.org/wiki/Point_cloud)

## Contact
For any inquiries, issues, or suggestions, please open an issue on GitHub or contact the project maintainers.