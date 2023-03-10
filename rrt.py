from occupany_grid import OccupanyGrid2d, Point2d
import numpy as np
import networkx as nx


class CrazyflieRRT:
    step_size: float
    tree: nx.DiGraph
    grid: OccupanyGrid2d
    goal_bias: float = 0.3

    rng: np.random.Generator

    def __init__(
        self,
        grid: OccupanyGrid2d,
        step_size=5,
        goal_bias=0.25,
    ):
        # Store occupancy grid to reference dimensions of area, as well as checking for collisions!
        self.tree = nx.DiGraph()
        self.grid = grid
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.bbox = [(0, 0), (12, 12), (-12, 12), (-12, -12), (-12, -12)]

        self.rng = np.random.default_rng(10)

    def generate(self, start: Point2d, goal: Point2d, max_iter: int = 800, goal_diameter=5):
        self.tree.add_node(start, id=0)
        self.initial_node = start
        new_node = start
        i = 0
        posn_min = self.grid.min_max[0]
        posn_max = self.grid.min_max[1]
        while (
            np.linalg.norm(np.array(goal) - np.array(new_node)) > goal_diameter
            and i < max_iter
        ):
            print(i)
            # 1. Sample a random point from the configuration space
            posn_random = self.rng.uniform(
                posn_min, posn_max, (1, 2)
            ).transpose()  # Generate random position as column vector

            # With a goal_bias percent chance, choose the actual goal as the random point
            # Heuristically bias the graph towards the goal a little bit
            if goal is not None and self.rng.random() > 1 - self.goal_bias:
                posn_random = np.array(goal).reshape(2, 1)

            # 2. Find the point in existing graph closest to the random point
            current_posns = np.array(
                [np.array(node) for node in self.tree.nodes]
            ).transpose()  # Fetch all current node positions
            distances = np.linalg.norm(current_posns - posn_random, axis=0)
            i_closest_node = np.argmin(distances)
            closest_posn = np.atleast_2d(
                current_posns[:, i_closest_node]
            ).T  # Extract the column that corresponds to the closest point - and keep it as column
            closest_node = Point2d(*closest_posn.flatten())

            # 3. Take a 1 step_size step towards the random point from the chosen closest point
            displacement_vector = posn_random - closest_posn
            step_vector = (
                displacement_vector
                * self.step_size
                / np.linalg.norm(displacement_vector)
            )
            new_posn = closest_posn + step_vector
            new_posn = np.clip( # Make sure this is within our world
                new_posn, posn_min.reshape((2, 1)), posn_max.reshape((2, 1))
            )
            new_node = Point2d(*new_posn.flatten())

            # 4. Check if that step will encounter a collision. If yes, skip to the next loop
            clear = self.grid.check_line(
                closest_node,
                new_node,
                bounding_box=self.bbox,
            )
            if not clear:
                new_node = closest_node
                continue

            # 5. Add the node to the tree.
            self.tree.add_node(new_node, id=i + 1)
            self.tree.add_edges_from([(closest_node, new_node)])

            i += 1

        # Goal was reached
        if np.linalg.norm(np.array(goal) - np.array(new_node)) <= goal_diameter:
            return self.get_final_traj(new_node, goal)

        # Goal not found, Max iters reached
        else:
            raise ValueError("No path Found")

    @staticmethod
    def interpolate_path(path: list[Point2d], des_dist = 10) -> list[Point2d]:
        new_path = []
        for seg in range(len(path)-1):
            p1 = np.asarray((path[seg][0], path[seg][1]) )
            p2 = np.asarray((path[seg+1][0], path[seg+1][1]))
            print(f"p1: {p1}, p2: {p2}")
            total_distance = np.linalg.norm(p2-p1)
            # if total_distance <= des_dist:
            #     return [p1, p2]
            num_pts = int(total_distance/des_dist)
            new_pts = np.linspace(p1, p2, num_pts, endpoint=True)
            print(f"dnew_pts{new_pts}")
            new_path.extend(new_pts)
        print(new_path)
        point_path = [Point2d(p[0], p[1]) for p in new_path]
        print(f"Point path{point_path}")
        res = []
        [res.append(x) for x in point_path if x not in res]
        print(res)
        return res
    
    def relax_path(self, path: list[Point2d], step_size: int) -> list[Point2d]:
        min_path_len = 3
        new_path = []
        curr_forward_idx = 0
        last_node_idx = len(path) - 1
        while curr_forward_idx <= last_node_idx:
            new_path.append(path[curr_forward_idx])
            curr_backward_idx = last_node_idx
            relaxed = False
            while curr_backward_idx != curr_forward_idx and not relaxed:
                if self.grid.check_line(
                    path[curr_forward_idx],
                    path[curr_backward_idx],
                    bounding_box=self.bbox,
                ):
                    relaxed = True
                else:
                    curr_backward_idx -= 1
            if relaxed:
                curr_forward_idx = curr_backward_idx
            else:
                curr_forward_idx += 1

        print(f" Realxed: {new_path}")
        return self.fit_spline(self.interpolate_path(new_path, step_size))
    
    @staticmethod
    def fit_spline(path: list):
        import numpy as np
        import matplotlib.pyplot as plt
        from scipy.interpolate import UnivariateSpline

        x = [point[0] for point in path]
        y = [point[1] for point in path]
        from scipy.interpolate import splprep, splev
        tck, _ = splprep([x, y])
        u = np.linspace(0,1,num=50)
        new_points = splev(u, tck)
        return np.asarray(new_points).transpose()
        


    def get_final_traj(self, final_node: Point2d, goal: Point2d):
        # Working backwards, at each point, the preceding node should be the adjacent node with the lowest ID.
        node = final_node
        node_list = [goal, final_node]
        print(f"Final trajj calc {node_list}, initial_node {self.initial_node}")
        while node != self.initial_node:
            print(node)
            prev_node = list(self.tree.predecessors(node))[0]
            node_list.append(prev_node)
            node = prev_node
        node_list.reverse()
        self.tree = nx.DiGraph()
        return node_list

    def rrt_wrapper(self, start: Point2d, goal: Point2d, max_iter: int = 800, goal_diameter=10, num_spline_pts=20):
        print(f"Start RRT {start} {goal}")
        final_path = self.generate(start, goal, max_iter, goal_diameter)
        splined_relaxed_path = self.relax_path(final_path, self.step_size)
        return splined_relaxed_path