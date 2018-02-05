import math
import threading
import logging

import math3d
import rospy
from geometry_msgs.msg import TwistWithCovariance, PoseWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from pibot.serial_connection import SerialController

logger = logging.getLogger(__name__)


class PiBotDriver(object):
    def __init__(self, host='10.0.0.1'):
        self.__odom_pub = rospy.Publisher(
            name='/odom',
            data_class=Odometry,
            queue_size=100
        )
        self.__image_pub = rospy.Publisher(
            name='image_raw',
            data_class=Image,
            queue_size=100
        )

        self.__pibot = SerialController(
            address=host
        )

        self.__wheel_radius = 0.02
        self.__wheel_distance = 0.1
        self.__ticks_per_meter = 1000

        self.__image_freq = 5
        self.__odom_freq = 10

        # These are accumulated
        self.__x = 0  # m
        self.__y = 0  # m
        self.__w = 0  # rad

        # Encoder measurements
        self.__lw_ticks = 0
        self.__rw_ticks = 0

        # These are set by the movement commands
        self.__dx = 0
        self.__dy = 0
        self.__dw = 0

        self.__odom_thread = threading.Thread(target=self.__odom_loop)
        # self.__image_thread = threading.Thread(target=self.__image_loop)

        self.__cmd_vel_sub = rospy.Subscriber(
            name='/cmd_vel',
            data_class=Twist,
            callback=self.__cmd_vel_callback
        )

        self.__transform_broadcaster = TransformBroadcaster()

    def __cmd_vel_callback(self, msg):
        # type: (Twist) -> None
        logger.info('Received cmd vel: {}'.format(msg))
        speed_l = (msg.linear.x - msg.angular.z * self.__wheel_distance / 2) * self.__wheel_radius
        speed_r = (msg.linear.x + msg.angular.z * self.__wheel_distance / 2) * self.__wheel_radius
        self.__pibot.set_motor_speeds(
            A=speed_l,
            B=speed_r
        )

    def __odom_loop(self):
        rate = rospy.Rate(self.__odom_freq)
        prev_t = rospy.Time.now()
        while not rospy.is_shutdown():

            logger.info('Updating odom')

            # Get the delta time
            t = rospy.Time.now()
            dt = t - prev_t

            # Get the motor ticks
            lw_ticks, rw_ticks = self.__pibot.get_motor_ticks()

            d_lw_ticks = self.__lw_ticks - lw_ticks
            d_rw_ticks = self.__rw_ticks - rw_ticks

            left_travel = d_lw_ticks / self.__ticks_per_meter
            right_travel = d_rw_ticks / self.__ticks_per_meter

            delta_travel = (left_travel + right_travel) / 2
            delta_theta = (right_travel - left_travel) / self.__wheel_distance

            if right_travel == left_travel:
                delta_x = left_travel * math.cos(self.__w)
                delta_y = right_travel * math.sin(self.__w)
            else:
                radius = self.__wheel_distance / 2 * (right_travel + left_travel) / (right_travel - left_travel)

                # Find the instantaneous center of curvature (ICC)
                icc_x = self.__x - radius * math.sin(self.__w)
                icc_y = self.__y + radius * math.cos(self.__w)

                delta_x = math.cos(delta_theta) * (self.__x - icc_x) - math.sin(delta_theta) * (
                    self.__y - icc_y) + icc_x - self.__x
                delta_y = math.sin(delta_theta) * (self.__x - icc_x) + math.cos(delta_theta) * (
                    self.__y - icc_y) + icc_y - self.__y

            self.__x += delta_x
            self.__y += delta_y
            self.__w = (self.__w + dt) % (2 * math.pi)
            self.__dx = delta_travel / dt
            self.__dy = 0
            self.__dw = delta_theta / dt

            qt = math3d.Orientation().new_rot_z(angle=self.__w).get_quaternion()
            odom = Odometry(
                header=Header(
                    frame_id='world',
                    stamp=t
                ),
                child_frame_id='base_link',
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self.__x,
                            y=self.__y,
                            z=0
                        ),
                        orientation=Quaternion(
                            x=qt.x,
                            y=qt.y,
                            z=qt.z,
                            w=qt.w
                        )
                    ),
                    covariance=[0.01, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.01, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(
                            x=self.__dx,
                            y=self.__dy,
                            z=0
                        ),
                        angular=Vector3(
                            x=0,
                            y=0,
                            z=self.__dw
                        ),
                    ),
                    covariance=[0.01, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.01, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
                )
            )
            self.__odom_pub.publish(odom)
            rate.sleep()
            prev_t = t

    def __image_loop(self):
        rate = rospy.Rate(self.__image_freq)
        while not rospy.is_shutdown():
            im = self.__pibot.get_image_from_camera()
            self.__image_pub.publish(im)
            rate.sleep()
