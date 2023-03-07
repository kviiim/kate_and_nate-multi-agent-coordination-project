"""Occupany Grid and Associated functions"""


from collections import namedtuple
from math import ceil
from typing import Sequence
import numpy as np

Point2d = namedtuple("Point2d", "x y")
#Point3d = namedtuple("Point3d", "x y z")

Pose2d = namedtuple("Pose2d", "x y h")

class OccupanyGrid2d:
    """Occupancy Grid written to use cm as units."""

    def __init__(
        self,
        grid_width: int,
        grid_depth: int,
        origin: Point2d,
        cell_size=5,
    ) -> None:
        if (
            grid_width % 2 == 1
            or grid_depth % 2 == 1
            or grid_depth <= 1
            or grid_depth <= 1
        ):
            raise ValueError(
                f"Occupancy grid should be even shaped {grid_width} {grid_depth}"
            )
        if (
            origin.x >= cell_size * grid_width
            or origin.y >= cell_size * grid_depth
        ):
            raise ValueError("Specify an origin within the Occupancy Grid")
        self.origin = origin  # in CM not in cells

        self.cell_size = cell_size
        self.map = np.zeros((grid_width, grid_depth), dtype=bool)

    @staticmethod
    def cells_to_glbl_pts(cells: np.array, cell_size: float, origin: Point2d):
        """Takes a 2 by X array of cell indicies."""
        return (cells * cell_size - origin).transpose()

    @staticmethod
    def glbl_pts_to_cells(
        origin: Point2d, cell_size: float, points: np.ndarray
    ) -> np.ndarray:
        transformed = points + np.array(origin)
        scaled = np.floor(transformed / cell_size).astype("int")
        return scaled

    @property
    def min_max(self):
        return (
            -np.array(self.origin),
            np.array(self.map.shape) * self.cell_size - self.origin,
        )

    def obstacles_in_global(self) -> np.array:
        """Return Obstacle map in Global Coordinates (cms)"""
        obstacles = np.array(self.map.nonzero()).transpose()
        return OccupanyGrid2d.cells_to_glbl_pts(obstacles, self.cell_size, self.origin)

    def add_points_global(self, points: Sequence[namedtuple]):
        """Add points in the global frame (relative to origin) to the occupancy_grid."""
        # TODO: Raise error on too big of query
        cells = OccupanyGrid2d.glbl_pts_to_cells(self.origin, self.cell_size, points)
        transposed = cells.transpose()
        self.map[(transposed[0], transposed[1])] = True

    def get_points_global(self, points: np.ndarray):
        """Take Points in the global Mpa frame and return if they are occupied"""
        cells = OccupanyGrid2d.glbl_pts_to_cells(self.origin, self.cell_size, points)
        cells = np.clip(cells, [0, 0], np.array(self.map.shape) - 1)
        transposed = cells.transpose()
        return self.map[(transposed[0], transposed[1])]

    # def get_points_rel(pose: Pose2d, points: Sequence[namedtuple]):
    #     # TODO: Matrix multiplication for rotation and transformation and then query points
    #     raise NotImplementedError

    # def set_points_rel(pose: Pose2d, points: Sequence[namedtuple]):
    #     # TODO: Matrix multiplication for rotation and transformation and then set points
    #     raise NotImplementedError

    def set_rectangles(self, bottom_left_point: Point2d, top_right_point: Point2d, value:bool=True):
        """Helper to add rows in our environment. Positioned on our gloabl coordinate"""
        # TODO: Refactor to use linspace to then use numpy indexing
        # self.map[(range(bottom_left_cell[0], top_right_cell[0] + 1), range(bottom_left_cell[1], top_right_cell[1] + 1), range(bottom_left_cell[2], top_right_cell[2] + 1))]

        bottom_left_cell = OccupanyGrid2d.glbl_pts_to_cells(
            self.origin, self.cell_size, bottom_left_point
        )
        top_right_cell = OccupanyGrid2d.glbl_pts_to_cells(
            self.origin, self.cell_size, top_right_point
        )
        xs = range(bottom_left_cell[0], top_right_cell[0] + 1)
        ys = range(bottom_left_cell[1], top_right_cell[1] + 1)

        for x in xs:
            for y in ys:
                self.map[(x, y)] = value


    def check_line(
        self, start_pt: Point2d, end_pt: Point2d, bounding_box: list[Point2d]
    ) -> bool:
        """Take in points in the global frame (ie. 0,0 is the origin)

        Args:
            start_pt (Point3d): _description_
            end_pt (Point3d): _description_

        Returns: True if there is no obstacle map collision
        """
        start_vec = np.array(start_pt)
        end_vec = np.array(end_pt)
        length = np.linalg.norm(end_vec - start_vec)
        num_pts = int(length / (self.cell_size / 4))
        if num_pts < 2:
            num_pts = 2

        points = np.linspace(start_vec, end_vec, num=num_pts, endpoint=True)
        points_list = np.concatenate([points + np.array(c) for c in bounding_box])
        return not np.any(self.get_points_global(points_list))

    # def project(self, point: Point3d):
    #     yaw, pitch, roll = 0, 0, -30 * np.pi / 180
    #     r_yaw = np.array(
    #         [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    #     )  # Rotation around z
    #     r_pitch = np.array(
    #         [
    #             [np.cos(pitch), 0, np.sin(pitch)],
    #             [0, 1, 0],
    #             [-np.sin(pitch), 0, np.cos(pitch)],
    #         ]
    #     )  # Rotation around y
    #     r_roll = np.array(
    #         [
    #             [1, 0, 0],
    #             [0, np.cos(roll), -np.sin(roll)],
    #             [0, np.sin(roll), np.cos(roll)],
    #         ]
    #     )  # Rotation around x

    #     R = np.matmul(r_yaw, np.matmul(r_pitch, r_roll))

    #     pt = np.array(point)

    #     print(f"Rotaion: {R.shape}, POint: {pt.shape}")
    #     print(np.matmul(R, point))

