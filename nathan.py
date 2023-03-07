
import math
from occupany_grid import OccupanyGrid2d, Point2d
from viz import viz_world


if __name__ == "__main__":
    glbl_origin = Point2d(
            90.8, 90.8,
        )  # Bottom Left corner of grid is 35.75 inches in from corner (x) and y
    init_state = Point2d(0, 0) # Starting spot of the drone
    cell_size = 5
    grid_width = int(math.ceil(300.4 / cell_size))  # 118.25 inches meters wide (x)
    grid_depth = int(math.ceil(211 / cell_size))  # 83 inches meters deep (y)

    if grid_width % 2 == 1:
        grid_width += 1
    if grid_depth % 2 == 1:
        grid_depth += 1

    robosys_grid = OccupanyGrid2d(
        grid_width, grid_depth, glbl_origin, cell_size=cell_size
    )

    robosys_grid.set_rectangles(Point2d(0, 30), Point2d(122, 60.5))
    robosys_grid.set_rectangles(Point2d(0, -60.5), Point2d(122, -30))
    # robosys_grid.set_rectangles(Point3d(0, -60.5, 62.3), Point3d(35, -35, 90))

    world = viz_world(robosys_grid)
    world.add_omap_to_fig()
    
    world.show_figure()