# This script controls the robot base
# on the input meter and angle values

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import tf
from math import radians, copysign, sqrt, pow, pi

global goal
goal = Point()
goal.x = 10.0
goal.y = 0

global start, start_theta
start = Point()
start.x = 0
start.y = 0
start_theta = 0

class TurtlebotController() :
    def __init__(self):
        # Give the node a name
        rospy.init_node('controller', anonymous=False)
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=500)

        # How fast will we update the robot's movement?
        global rate
        rate = 50

        # Set the equivalent ROS rate variable
        global r
        r = rospy.Rate(rate)

        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(1.0)

        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = pi

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def on_mline(self,y):
        if y <= 0.1 and y > -0.1 :
            return True
        else:
            return False

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    # negative values simply move the robot backwards
    # input x will be in meters
    def translate(self, x) :
        move_cmd = Twist()
        # Set the equivalent ROS rate variable

        linear_speed = 0.2

        # Set the travel distance to 1.0 meters
        if x < 0 :
            linear_speed = -linear_speed
        else:
            linear_speed = linear_speed

        # How long should it take us to get there?
        linear_duration = float(x) / linear_speed

        move_cmd.linear.x = linear_speed

        # Move forward for a time to go the desired distance
        ticks = int(linear_duration * rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()


    # negative values simply move the robot clockwise
    # input d will be in degrees
    def rotate(self, angle):
        move_cmd = Twist()
        # Set the equivalent ROS rate variable
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 1.0

        # Set the angulamlr speed
        if angle < 0 :
            move_cmd.angular.z = -angular_speed
        else :
            move_cmd.angular.z = angular_speed

        # convert absolute value of degrees to radians
        angle = (int(abs(angle)) * pi ) / 180

        # Rotate for a time to go goal_angle degrees
        ticks = int(angle * rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # How long should it take to rotate?
        angular_duration = angle / angular_speed

    def stopRobot(self):
        # Stop the robot before the next leg
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
