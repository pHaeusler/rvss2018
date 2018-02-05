#!/usr/bin/env python

import argparse
import logging

import rospy
import rospy.impl.rosout

from pibot import driver


def run(host):
    rospy.loginfo('Starting Driver...')
    bot = driver.PiBotDriver(host=host)

    rospy.loginfo('Successfully started')
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('robot_node')

    logging.getLogger('pibot').addHandler(rospy.impl.rosout.RosOutHandler())
    logging.getLogger('pibot').setLevel(logging.INFO)

    _host = rospy.get_param('~host', '10.0.0.20')

    run(host=_host)
