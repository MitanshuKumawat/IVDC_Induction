import numpy as np
import open3d as o3d

# Sample LIDAR data (in the sensor's coordinate system)
lidar_data = np.array([
    [1.0, 2.0, 3.0],
    [2.5, 1.5, 4.0],
    [3.2, 3.1, 2.8],
    [1.8, 2.7, 3.5],
    [2.2, 1.9, 3.9]
])

# Coordinate transformation (from sensor to global coordinate system)
translation = np.array([0.5, 1.0, 0.0])
rotation = np.array([[0.9, -0.2, 0.1],
                    [0.2, 0.8, -0.3],
                    [-0.1, 0.3, 0.9]])

transformed_data = np.dot(lidar_data, rotation.T) + translation

# Create a point cloud
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(transformed_data)

# here we can add additional information (e.g., intensity, timestamp, color)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])