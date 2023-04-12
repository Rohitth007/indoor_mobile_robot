import argparse
from heapq import heappush, heappop, heapify

# from math import sqrt

from PIL import Image, ImageDraw
from matplotlib import pyplot as plt

from map import Map


class Node:
    """..."""

    def __init__(self, location, direction, parent, heuristic, accum_cost):
        self.loc = location
        self.dir = direction
        self.parent = parent
        self.heuristic = heuristic
        self.accum_cost = accum_cost
        self.ctg = heuristic + accum_cost

    def __lt__(self, other):
        return self.ctg < other.ctg

    def __gt__(self, other):
        return self.ctg > other.ctg


class PriorityQueueFrontier:
    """..."""

    def __init__(self):
        self.frontier = []
        heapify(self.frontier)

    def push(self, node):
        """..."""
        heappush(self.frontier, node)

    def contains_state(self, loc):
        """..."""
        return any(node.loc == loc for node in self.frontier)

    def empty(self):
        """..."""
        return len(self.frontier) == 0

    def pop(self):
        """..."""
        if not self.empty():
            return heappop(self.frontier)
        else:
            raise Exception("Empty Frontier\n")


class GlobalPlanner:
    """..."""

    def __init__(self, _map, src_loc, src_dir, goal):
        self.map = _map
        self.grid = 255 - _map.grid
        self.src_loc = _map.get_indices_from_pos(*src_loc)
        self.src_dir = src_dir  # N - 0, E - 1, S - 2, W - 3
        self.goal = _map.get_indices_from_pos(*goal)
        print(self.src_loc, self.src_dir, self.goal)
        print(src_loc, src_dir, goal)
        self.waypoints = None
        self.num_explored = 0
        self.explored = set()
        self.G = float("inf")

    def neighbors(self, node):
        """Finds neighbours reachable through actions of a particular state"""
        i, j = node.loc
        d = node.dir
        candidates = [
            [i, j - 1, d],
            [i - 1, j, d],
            [i, j + 1, d],
            [i + 1, j, d],
            [i, j - 1, d],
            [i - 1, j, d],
        ]

        candidates = candidates[d : d + 3]
        candidates[0][2] = (d - 1) % 4  # Left
        candidates[2][2] = (d + 1) % 4  # Right

        h, w = self.grid.shape
        result = []
        for ci, cj, cd in candidates:
            if 0 <= ci < h and 0 <= cj < w and self.grid[ci, cj] != 255:
                result.append((ci, cj, cd))

        # print(result)
        return result

    def solve(self):
        """Finds solution, if it exists."""

        # Keep track of no. of states explored
        self.num_explored = 0

        # Initialize the frontier to just the starting position
        frontier = PriorityQueueFrontier()
        cost = int(self.grid[self.src_loc[0], self.src_loc[1]])
        source = Node(
            location=self.src_loc,
            direction=self.src_dir,
            parent=None,
            heuristic=self.manhattan_dist(self.src_loc),
            accum_cost=cost,
        )
        print(self.src_loc, self.src_dir)
        frontier.push(source)

        # Initialise an empty explored set
        self.explored = set()

        # Keep looping until solution found
        while not frontier.empty():

            # Choose a node with the least cost from frontier
            node = frontier.pop()
            self.num_explored += 1
            # print(node.loc, node.dir, node.accum_cost)

            if node.accum_cost >= self.G:
                continue

            # If node is goal, then we have a Solution
            if node.loc == self.goal:
                print("hi", node.accum_cost)
                self.G = node.accum_cost
                self.G_node = node

            # Mark node as explored
            self.explored.add(node.loc)

            # Add neighbour to frontier
            for ni, nj, nd in self.neighbors(node):
                if (ni, nj) not in self.explored:
                    cost = node.accum_cost
                    cost += 1  # Cost of moving
                    cost += self.grid[ni, nj]  # Cost of inflated obstacles
                    if node.dir != nd:  # Cost for turning
                        cost += 10
                    # Each of the above costs can be weighted to prioratize one over the other.
                    child = Node(
                        location=(ni, nj),
                        direction=nd,
                        parent=node,
                        heuristic=self.manhattan_dist((ni, nj)),
                        accum_cost=cost,
                    )
                    frontier.push(child)

        if self.G == float("inf"):
            raise Exception("No Solution\n")
        else:
            self.backtrack(self.G_node)

    def manhattan_dist(self, loc):
        """..."""
        i, j = loc
        gi, gj = self.goal
        return abs(i - gi) + abs(j - gj)
        # return sqrt((si - gi) ** 2 + (sj - gj) ** 2)

    def backtrack(self, node):
        """..."""
        path = []
        while node is not None:
            pos = self.map.get_pos_from_indices(*node.loc)
            path.append((*pos, node.dir))
            node = node.parent
        path.reverse()
        self.waypoints = path
        self.clean()

    def clean(self):
        """..."""
        path = []
        p1 = self.waypoints[0]
        p2 = self.waypoints[1]
        prev = (p1[0] - p2[0], p1[1] - p2[1])
        path.append(p1)
        for i in range(1, len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i + 1]
            curr = (p1[0] - p2[0], p1[1] - p2[1])
            if prev[0] * curr[1] != curr[0] * prev[1]:
                path.append(p1)
            prev = curr
        path.append(p2)
        self.waypoints = path

    def show_path(self):
        """..."""
        img = Image.fromarray(self.map.og).convert("RGB")
        draw = ImageDraw.Draw(img)

        for i in range(len(self.waypoints) - 1):
            p1 = self.map.get_indices_from_pos(*self.waypoints[i][:2])
            p1 = (p1[1] * 16 + 80, p1[0] * 16)
            # p1 = (p1[1], p1[0])
            p2 = self.map.get_indices_from_pos(*self.waypoints[i + 1][:2])
            p2 = (p2[1] * 16 + 80, p2[0] * 16)
            # p2 = (p2[1], p2[0])
            draw.line((p1, p2), fill="yellow", width=10)
            draw.line((p1, p1), fill="red", width=10)
        draw.line((p2, p2), fill="red", width=10)

        _, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(img, cmap="gray")
        plt.show()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("mapfile", help="file containing map in .pgm format")
    args = parser.parse_args()

    _map = Map(args.mapfile)
    _map.downsample()
    _map.show()

    # planner = GlobalPlanner(map, (22, 7), 0, (5, 20))
    # planner = GlobalPlanner(_map, (-1.6, -1.5), 0, (0.6, 1.8))
    planner = GlobalPlanner(_map, (-1.8, -1.5), 0, (0.7, 2.0))
    # planner = GlobalPlanner(_map, (-1.6, -1.5), 0, (-1.0, -0.5))
    print("Solving...")
    planner.solve()
    print("States Explored:", planner.num_explored)
    print("Solution:")
    print(planner.waypoints)
    print(len(planner.waypoints))
    planner.show_path()
