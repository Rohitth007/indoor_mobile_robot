from ast import Global
from math import sin, cos, pi, atan2, sqrt
from numpy import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from math import sin, cos, pi


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):

        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):

        theta = state[2]
        l, r = control

        if r != l:
            alpha = (r - l) / w
            R = l / alpha
            g1 = (R + (w / 2)) * (cos(theta + alpha) - cos(theta))
            g2 = (R + (w / 2)) * (sin(theta + alpha) - sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        else:
            # This is for the special case r == l.
            m = array([[1.0, 0.0, -l * sin(theta)], [0.0, 1.0, l * cos(theta)], [0.0, 0.0, 1.0]])

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):

        theta = state[2]
        l, r = tuple(control)

        if r != l:
            alpha = (r - l) / w

            wr = (w * r) / ((r - l) ** 2)
            wl = (w * l) / ((r - l) ** 2)
            r2l = (r + l) / (2 * (r - l))

            g1_l = wr * (sin(theta + alpha) - sin(theta)) - r2l * cos(theta + alpha)
            g2_l = wr * (-cos(theta + alpha) + cos(theta)) - r2l * sin(theta + alpha)
            g3_l = - (1 / w)

            g1_r = -wl * (sin(theta + alpha) - sin(theta)) + r2l * cos(theta + alpha)
            g2_r = -wl * (-cos(theta + alpha) + cos(theta)) + r2l * sin(theta + alpha)
            g3_r = 1 / w

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        else:

            # This is for the special case l == r.
            g1_l = .5 * (cos(theta) + (l / w) * sin(theta))
            g2_l = .5 * (sin(theta) - (l / w) * cos(theta))
            g3_l = - 1 / w

            g1_r = .5 * ((-l / w) * sin(theta) + cos(theta))
            g2_r = .5 * ((l / w) * cos(theta) + sin(theta))
            g3_r = 1 / w

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):

        #subscribe with left and right tick and then cacualte tick difference like earlier
        left, right = control

        alpha_1 = control_motion_factor
        alpha_2 = control_turn_factor

        sigma_l = (alpha_1 * left) ** 2 + (alpha_2 * (left - right)) ** 2
        sigma_r = (alpha_1 * right) ** 2 + (alpha_2 * (left - right)) ** 2
        control_covariance = diag([sigma_l, sigma_r])

        G_t = self.dg_dstate(self.state, control, self.robot_width)
        V = self.dg_dcontrol(self.state, control, self.robot_width)

        self.covariance = dot(G_t, dot(self.covariance, G_t.T)) + dot(V, dot(control_covariance, V.T))

        # --->>> Put your code to compute the new self.state here.
        
        self.state = self.g(self.state, control, self.robot_width)

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):

        x, y, theta = state
        x_m, y_m = landmark
        d = scanner_displacement

        x_l = x + d * cos(theta)
        y_l = y + d * sin(theta)

        delta_x = x_m - x_l
        delta_y = y_m - y_l

        q = (delta_x) ** 2 + (delta_y) ** 2

        dr_dx = -delta_x / sqrt(q)
        dr_dy = -delta_y / sqrt(q)
        dr_dtheta = (d / sqrt(q)) * (delta_x * sin(theta) - delta_y * cos(theta))

        dalpha_dx = delta_y / q
        dalpha_dy = -delta_x / q
        dalpha_dtheta = - (d / q) * (delta_x * cos(theta) + delta_y * sin(theta)) - 1

        return array([[dr_dx, dr_dy, dr_dtheta], [dalpha_dx, dalpha_dy, dalpha_dtheta]])

    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""


        H_t = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        Q = diag([self.measurement_distance_stddev **2, self.measurement_angle_stddev **2])
        K_t = dot(dot(self.covariance, H_t.T), linalg.inv(dot(H_t, dot(self.covariance, H_t.T)) + Q))

        innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi

        self.state += dot(K_t, innovation)   ## new state = old state + K * innovation

        self.covariance = dot(eye(3) - dot(K_t, H_t), self.covariance)

class ticks:
    prev_left_tick=0
    prev_right_tick=0
    left_tick=0
    right_tick=0

class features:
    num_lines=0
    end_points=[]
    radius=[]
    alpha=[]

Tick=ticks()
feature=features()


def callback_left(msg):
    global Tick
    Tick.left_tick=msg.data

def callback_right(msg):
    global Tick
    Tick.right_tick=msg.data

def callback(data):
    global feature
    feature.end_points=data.end_points
    feature.num_lines=data.num_lines
    feature.radius=data.radius_values
    feature.alpha=data.alpha_values

def scanner_to_world(pose, point):
        """Given a robot pose (rx, ry, heading) and a point (x, y) in the
           scanner's coordinate system, return the point's coordinates in the
           world coordinate system."""
        dx = cos(pose[2])
        dy = sin(pose[2])
        x, y = point
        return (x * dx - y * dy + pose[0], x * dy + y * dx + pose[1])
def get_observations(robot_pose, scanner_displacement,map,max_thershold):
    global feature

    #result of the line features extraction comes here 
    #cylinders = find_cylinders(scan, der, jump, min_dist)
    rospy.Subscriber('/feature_from_lidar', features, callback)

    # Compute scanner pose from robot pose.
    scanner_pose = (robot_pose[0] + cos(robot_pose[2]) * scanner_displacement,
                    robot_pose[1] + sin(robot_pose[2]) * scanner_displacement,
                    robot_pose[2])

    # For every detected cylinder which has a closest matching pole in the
    # reference cylinders set, put the measurement (distance, angle) and the
    # corresponding reference cylinder into the result list.
    result = []
    #matching between the coordinates of the walls in world frame and local frame
    for r, alpha in feature.radius,feature.alpha:
        # Compute the angle and distance measurements.
        angle = alpha
        distance = r
        # Compute x, y of cylinder in world coordinates.
        x, y = distance*cos(angle), distance*sin(angle)
        x, y = scanner_to_world(scanner_pose, (x, y))
        # Find closest cylinder in reference cylinder set.
        best_dist_2 = max_thershold * max_thershold
        best_ref = None
        for l in map:
            dx, dy = l[0] - x, l[1] - y
            dist_2 = dx * dx + dy * dy
            if dist_2 < best_dist_2:
                best_dist_2 = dist_2
                best_ref = l
        # If found, add to both lists.
        if best_ref:
            result.append(((distance, angle), best_ref))

    return result

def main():
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([0, 0, 0 ])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Reading the map.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    map = [l[1:3] for l in logfile.landmarks]


    states = []
    covariances = []
    matched_ref_cylinders = []
    while not rospy.is_shutdown():
        
        # Prediction.
        rospy.Subscriber('/left_ticks', Int64, callback_left)
        rospy.Subscriber('/right_ticks', Int64, callback_right)

        tick_difference=[0,0]

        tick_difference[0]=Tick.left_tick-Tick.prev_left_tick
        tick_difference[1]=Tick.right_tick-Tick.prev_right_tick

        control = array(tick_difference[0],tick_difference[1]) * ticks_to_mm

        kf.predict(control)

        # Correction.
        
        observations = get_observations(kf.state, scanner_displacement,map, max_cylinder_distance)

        for j in range(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction_and_correction.txt", "w")
    for i in range(len(states)):
        # Output the center of the scanner, not the center of the robot.
        f.write("F %f %f %f \n" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0]))
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        f.write("E %f %f %f %f \n" % (e + (sqrt(covariances[i][2,2]),)))
        # Also, write matched cylinders.
        write_cylinders(f, "W C ", matched_ref_cylinders[i])

    f.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
