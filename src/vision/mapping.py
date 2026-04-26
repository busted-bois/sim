import numpy as np
from dataclasses import dataclass
from typing import Dict, Tuple

@dataclass
class VoxelGrid:
    """
    A simple sparse voxel grid to store occupied space.
    """
    resolution: float = 0.5  # meters
    points: Dict[Tuple[int, int, int], int] = None  # (ix, iy, iz) -> count/intensity

    def __post_init__(self):
        if self.points is None:
            self.points = {}

    def add_point(self, x: float, y: float, z: float):
        ix = int(np.floor(x / self.resolution))
        iy = int(np.floor(y / self.resolution))
        iz = int(np.floor(z / self.resolution))
        key = (ix, iy, iz)
        self.points[key] = self.points.get(key, 0) + 1

    def is_occupied(self, x: float, y: float, z: float, threshold: int = 1) -> bool:
        ix = int(np.floor(x / self.resolution))
        iy = int(np.floor(y / self.resolution))
        iz = int(np.floor(z / self.resolution))
        return self.points.get((ix, iy, iz), 0) >= threshold

class Mapper:
    """
    Handles projection of 2D depth maps into 3D world space.
    """
    def __init__(self, fov_degrees: float, width: int, height: int):
        self.fov_rad = np.radians(fov_degrees)
        self.width = width
        self.height = height
        
        # Focal length in pixels
        self.f = self.width / (2.0 * np.tan(self.fov_rad / 2.0))
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        
        self.voxel_grid = VoxelGrid()

    def project_to_body(self, depth_map: np.ndarray) -> np.ndarray:
        """
        Projects a depth map to 3D points in the camera/body frame.
        
        Returns:
            An (N, 3) array of (x, y, z) coordinates in the camera frame.
            Note: Camera frame convention often uses Z as forward, X right, Y down.
        """
        h, w = depth_map.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # Flatten for vectorized computation
        u = u.flatten()
        v = v.flatten()
        z = depth_map.flatten()
        
        # Filter out very distant or invalid points
        mask = (z > 0.1) & (z < 50.0)
        u, v, z = u[mask], v[mask], z[mask]
        
        x = (u - self.cx) * z / self.f
        y = (v - self.cy) * z / self.f
        
        # In many CV conventions: Z is forward, X is right, Y is down
        # Points are (X, Y, Z) in camera frame
        return np.stack([x, y, z], axis=1)

    def transform_to_world(self, points_body: np.ndarray, drone_pose) -> np.ndarray:
        """
        Transforms points from body frame to world frame.
        
        Args:
            points_body: (N, 3) array
            drone_pose: An object with position (x, y, z) and orientation (quaternion or matrix)
        """
        # This requires rotation matrix from drone orientation
        # For now, a placeholder for the math:
        # points_world = R_body_to_world @ points_body + T_drone_world
        
        # Placeholder implementation
        return points_body # TODO: Implement actual rotation

    def update_map(self, depth_map: np.ndarray, drone_pose=None):
        """
        Projects depth map and updates the voxel grid.
        """
        points_body = self.project_to_body(depth_map)
        
        # If we have pose, transform to world
        if drone_pose:
            points_world = self.transform_to_world(points_body, drone_pose)
        else:
            points_world = points_body
            
        # Downsample for mapping performance
        # Only take every 10th point if too many
        step = max(1, len(points_world) // 1000)
        for i in range(0, len(points_world), step):
            p = points_world[i]
            self.voxel_grid.add_point(p[0], p[1], p[2])
