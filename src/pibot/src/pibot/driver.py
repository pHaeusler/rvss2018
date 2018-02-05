import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from pibot.serial_connection import SerialController


class PiBotDriver(object):

    def __init__(self, robot_ip='10.0.0.1'):

        self.__odom_pub = rospy.Publisher(
            name='/odom',
            data_class=Odometry
        )
        self.__image_pub = rospy.Publisher(
            name='image_raw',
            data_class=Image
        )

        self.__pibot = SerialController(
            address=robot_ip
        )
