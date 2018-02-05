import io
import logging
import re
import socket
import threading

import numpy as np

logger = logging.getLogger(__name__)


# Fill the buffer with all incoming data
def fill_buffer(s, size):
    buf = b''
    # Receive until buffer is full
    while len(buf) < size:
        received_data = s.recv(size)
        buf += received_data
    return buf


class SerialController(object):
    def __init__(self, address):
        # Robot Address
        self.IP_ADDRESS = address
        self.CHUNK_SIZE = 128

        self.TIMEOUT = 10

        # Ports
        self.PORT_MOTORS = 43900
        self.PORT_CAMERAS = 43901
        self.PORT_TAGS = 43902

        # Commands
        self.FN_ARG_SEPARATOR = ','
        self.FN_GET_IMAGE = 'getImageFromCamera'
        self.FN_MOTOR_SPEEDS = 'setMotorSpeeds'
        self.FN_MOTOR_SPEEDS_PROFILE = 'setMotorSpeedsProfile'
        self.FN_MOTOR_TICKS = 'getMotorTicks'
        self.FN_APRIL_TAGS = 'getTags'
        self.FN_MOTOR_ENCODERS = 'getMotorEncoders'
        self.FN_DISPLAY_VALUE = 'setDisplayValue'
        self.FN_DISPLAY_MODE = 'setDisplayMode'
        self.FN_ALL_STOP = 'stopAll'
        self.FN_MOTOR_MOVE = 'setMotorMove'

        # Image details
        self.IM_WIDTH = 320
        self.IM_HEIGHT = 240
        self.IMAGE_SIZE = self.IM_WIDTH * self.IM_HEIGHT * 3

        self.__motor_socket_cond = threading.Condition()

    # Create the required socket, input port number
    def create_socket(self, port):
        # Set up a socket with the specified port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = (self.IP_ADDRESS, port)
        # s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.connect(address)
        return s

    # Set the robot's motor speeds, input tuple (A, B)
    def set_motor_speeds(self, A, B, T=0):

        with self.__motor_socket_cond:

            # Create the socket
            motor_socket = self.create_socket(self.PORT_MOTORS)

            if T != 0:
                # Put together the command and send
                command = "%s,%s,%s,%s,%s" % (self.FN_MOTOR_SPEEDS_PROFILE, str(A), str(B), str(T), str(0))
                motor_socket.sendall(command.encode("utf-8"))

            else:
                command = "%s,%s,%s" % (self.FN_MOTOR_SPEEDS, str(A), str(B))
                motor_socket.sendall(command.encode("utf-8"))

            # Close the socket
            motor_socket.close()

    # Get the robot's motor ticks, output tuple (A, B)
    def get_motor_ticks(self):

        with self.__motor_socket_cond:
            # Create the socket
            motor_socket = self.create_socket(self.PORT_MOTORS)

            # Put together the command and send
            command = self.FN_MOTOR_TICKS + self.FN_ARG_SEPARATOR + 'A'
            motor_socket.sendall(command.encode("utf-8"))

            # Receive ticks
            recv_data = motor_socket.recv(self.CHUNK_SIZE)
            raw_data = recv_data
            # Ensure that data is received
            while not recv_data:
                recv_data = motor_socket.recv(self.CHUNK_SIZE)
                raw_data += recv_data

            # Decode bytes
            str_data = raw_data.decode("utf-8")
            # Regex to convert string to array of numbers
            ticks_list = re.findall('-*[0-9]+', str_data)
            # Convert strings to floats
            ticks = [float(tick) for tick in ticks_list]

            # Close the socket
            motor_socket.close()

            return ticks

    # Get an image from the camera
    def get_image_from_camera(self):
        # Create socket for images and connect
        self.camera_socket = self.create_socket(self.PORT_CAMERAS)

        # Send the request for the image
        self.camera_socket.sendall(self.FN_GET_IMAGE.encode("utf-8"))

        # Set up stream container for image
        image_stream = io.BytesIO()

        # Accumulate image bytes into buffer
        buf = fill_buffer(self.camera_socket, self.IMAGE_SIZE)

        # Write the buffer to the stream
        image_stream.write(buf)

        # Close socket
        self.camera_socket.close()

        # Convert stream to image array
        image = np.fromstring(image_stream.getvalue(), dtype=np.uint8)
        image = np.reshape(image, (self.IM_HEIGHT, self.IM_WIDTH, 3))
        image = image[:, :, ::-1]  # Convert BGR to RGB

        # Show image
        # cv2.imshow('Recieved Image', image)
        # cv2.waitKey(0)
        return image

    # Stop the motors
    def stop(self):
        self.set_motor_speeds(0, 0)

    # Reset: stop motors, reset encoders
    # Not functional - just replicating MATLAB code
    def reset(self):
        motor_socket = self.create_socket(self.PORT_MOTORS)

        command = self.FN_ALL_STOP

        motor_socket.sendall(command.encode('utf-8'))

        motor_socket.close()
