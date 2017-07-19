#!/usr/bin/env python

import argparse
import subprocess
import sys
from threading import Thread

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from sensor_msgs.msg import Joy
from tf import TransformListener

class StateThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["rosrun", "robot_state_publisher", "robot_state_publisher"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_state_running():
    output = subprocess.check_output(["rosnode", "info", "robot_state_publisher"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class LookThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.client = actionlib.SimpleActionClient('head_controller/point_head', PointHeadAction)
        self.client.wait_for_server()
        self.tf = TransformListener()
        self.start()

    def run(self):
        state_thread = None
        if not is_state_running():
            rospy.loginfo("starting state")
            state_thread = StateThread()

        target_frame = '/gripper_link'
        source_frame = '/base_link'

        time = rospy.Time(0);
        self.tf.waitForTransform(source_frame, target_frame, time, rospy.Duration(3.0));
        position, orientation = self.tf.lookupTransform(source_frame, target_frame, time);

        point = PointStamped()
        point.header.frame_id = source_frame
        point.point.x = position[0]
        point.point.y = position[1]
        point.point.z = position[2]

        goal = PointHeadGoal()
        goal.target = point
        goal.min_duration = rospy.Duration.from_sec(10.0)
        goal.max_velocity = 1

        self.client.cancel_all_goals()
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(10.0))
        print self.client.get_goal_status_text()

	if state_thread:
            state_thread.stop()
            rospy.signal_shutdown("done")
            sys.exit(0)
            return

    def stop(self):
        rospy.signal_shutdown("failed")
        sys.exit(0)

class LookTeleop:

    def __init__(self):
        self.look_button = rospy.get_param("~tuck_button", 5)  # default button is the down button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.looking = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.looking:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't tuck
                rospy.loginfo("Stopping look thread")
                self.look_thread.stop()
            return
        try:
            if msg.buttons[self.look_button] > 0 and msg.buttons[self.deadman] > 0:
                self.looking = True
                rospy.loginfo("Starting look thread")
                self.look_thread = LookThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("tuck_button is out of range")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Look at the end-effector, either immediately or as a joystck-controlled server.")
    parser.add_argument("--joystick", action="store_true", help="Run as server that tucks on command from joystick.")
    args, unknown = parser.parse_known_args()

    rospy.init_node("head_point")
    rospy.loginfo("New head point script running")

    if args.joystick:
        t = LookTeleop()
        rospy.spin()
    else:
        rospy.loginfo("Pointing the head...")
        LookThread()
