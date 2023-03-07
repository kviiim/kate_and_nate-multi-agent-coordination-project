"""Simulator/Visualizer for outputs of RRT algorithm"""
from math import ceil
import math
import numpy as np
import plotly.graph_objects as go
import plotly

from occupany_grid import OccupanyGrid2d, Point2d


class viz_world:
    def __init__(self, omap: OccupanyGrid2d):
        self.omap = omap
        self.fig = go.Figure()
        self.fig["layout"]["uirevision"] = 42

    def add_omap_to_fig(self):
        obstacles = self.omap.obstacles_in_global()
        omap_plot = go.Scatter(
            x=obstacles[0],
            y=obstacles[1],
            opacity=1,
            mode="markers",
            marker=dict(
                size=10,
                symbol="square",  # ['circle', 'circle-open', 'cross', 'diamond','diamond-open', 'square', 'square-open', 'x']
                # color=plots[0],                # set color to an array/list of desired values
                # colorscale='Viridis',   # choose a colorscale
                opacity=0.5,
            ),
            name="Obstacle Map",
        )
        self.fig.add_trace(omap_plot)

    def add_point(
        self,
        point: Point2d,
        marker: dict | None = None,
        name: str | None = None,
        show_legend=False,
    ):
        if marker is None:
            marker = dict(
                size=5,
                symbol="circle",
                color="black",
                opacity=0.25,
            )
        plotly_point = go.Scatter(
            x=[point.x],
            y=[point.y],
            opacity=1,
            mode="markers",
            marker=marker,
            name=name,
            showlegend=show_legend,
        )
        self.fig.add_trace(plotly_point)

    def plot_trajectory(
        self,
        points: list,
        marker: dict | None = None,
        line: dict | None = None,
        text: list[int] = None,
        name: str = None,
    ) -> None:
        """Plots a list of nodes."""
        if marker is None:
            marker = dict(
                size=2,
                symbol="circle",
                color="black",
                opacity=0.25,
            )
        if text is None:
            nums = range(len(points))
            text = [str(n) for n in nums]
        if line is None:
            line = dict(
                color="black",
                width=1,
            )
        x = np.asarray(points).transpose()[0]
        y = np.asarray(points).transpose()[1]
        trajectory = go.Scatter(
            x=x,
            y=y,
            opacity=1,
            marker=marker,
            mode="markers+text+lines",
            text=text,
            textposition="top center",
            line=line,
            showlegend=(name is not None),
            name=name,
        )
        self.fig.add_trace(trajectory)

    def update_figure(self):
        min, max = self.omap.min_max
        self.fig.update_xaxes(range=[min[0], max[0]], constrain='domain')
        self.fig.update_yaxes(range=[min[1], max[1]], constrain='domain', scaleanchor = "x", scaleratio = 1,)


    def show_figure(self):
        self.update_figure()
        self.fig.show()

    def save_figure(self):
        self.fig.write_html("plotly.html")

def build_robosys_world():
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

    return world