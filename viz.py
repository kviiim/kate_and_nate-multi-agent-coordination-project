"""Simulator/Visualizer for outputs of RRT algorithm"""
from math import ceil
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
        trajectory = go.Scatter3d(
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
