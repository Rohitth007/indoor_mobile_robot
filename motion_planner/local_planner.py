from math import sin, cos, sqrt, pi

import matplotlib.pyplot as plt


class LocalPlanner:
    def __init__(
        self,
        arc_length=0.3,
        velocity=0.05,
        motion_prim_conf=[-0.16, -0.3, "inf", 0.3, 0.16],
    ):
        self.pose = (0, 0, 0)
        self.target = (0, 0, 0)
        self.arc_length = arc_length
        self.velocity = velocity
        self.motion_prim_conf = motion_prim_conf

    def generate_motion_primitives(self):
        self.ends = []
        x, y, th = self.pose
        for r in self.motion_prim_conf:
            if r == "inf":
                x_end = x + self.arc_length * cos(th)
                y_end = y + self.arc_length * sin(th)
                th_end = th
            else:
                phi = self.arc_length / r
                x_end = x + r * (-sin(th) + sin(phi + th))
                y_end = y + r * (cos(th) - cos(phi + th))
                th_end = (th + phi) % (2 * pi)
            self.ends.append((x_end, y_end, th_end))

    def check_collisions(self):
        pass

    def show_motion_prims(self):
        x, y, th = self.pose
        for s in range(0, int(self.arc_length * 100)):
            s /= 100
            for r in self.motion_prim_conf:
                if r == "inf":
                    x_end = x + s * cos(th)
                    y_end = y + s * sin(th)
                else:
                    phi = s / r
                    x_end = x + r * (-sin(th) + sin(phi + th))
                    y_end = y + r * (cos(th) - cos(phi + th))
                plt.scatter(x_end, y_end, c="green")
        plt.show()

    def optimize(self):
        mini = float("inf")
        mini_idx = 0
        for i, pose in enumerate(self.ends):
            d = self.euclidean_dist(pose)
            if d < mini:
                mini = d
                mini_idx = i

        r_opt = self.motion_prim_conf[mini_idx]
        if r_opt == "inf":
            return self.velocity, 0
        else:
            return self.velocity, self.velocity / r_opt

    def euclidean_dist(self, pose):
        x, y, _ = pose
        gx, gy, _ = self.target
        return sqrt((gx - x) ** 2 + (gy - y) ** 2)


if __name__ == "__main__":
    planner = LocalPlanner()
    planner.target = (-1, 1)
    planner.generate_motion_primitives()
    planner.show_motion_prims()
    print("Solution:", planner.optimize())
