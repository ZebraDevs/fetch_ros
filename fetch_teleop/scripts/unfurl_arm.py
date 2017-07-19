#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import argparse
import subprocess
import sys
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface

class MoveItThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["roslaunch", "fetch_moveit_sensor", "move_group.launch"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_moveit_running():
    output = subprocess.check_output(["rosnode", "info", "move_group"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class TuckThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.26599299907684326, 0.5012994194717407, 1.2815969776531981, -2.2721004241638183, 2.2433900814910888, -2.7745147650579836, 0.9760874303430176, -2.007323998046875]
        while not rospy.is_shutdown():
            self.client.setPlannerId("RRTConnectkConfigDefault")
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                if move_thread:
                    move_thread.stop()

                # On success quit
                # Stopping the MoveIt thread works, however, the action client
                # does not get shut down, and future unfurls will not work.
                # As a work-around, we die and roslaunch will respawn us.
                rospy.signal_shutdown("done")
                sys.exit(0)
                return

    def stop(self):
        if self.client:
            self.client.get_move_action().cancel_goal()
        # Stopping the MoveIt thread works, however, the action client
        # does not get shut down, and future unfurls will not work.
        # As a work-around, we die and roslaunch will respawn us.
        rospy.signal_shutdown("failed")
        sys.exit(0)

class TuckArmTeleop:

    def __init__(self):
        self.unfurl_button = rospy.get_param("~unfurl_button", 7)  # default button is the down button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.unfurling = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.unfurling:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't unfurl
                rospy.loginfo("Stopping unfurl thread")
                self.unfurl_thread.stop()
            return
        try:
            if msg.buttons[self.unfurl_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Tuck the arm
                    self.unfurling = True
                    rospy.loginfo("Starting unfurl thread")
                    self.unfurl_thread = TuckThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("unfurl_button is out of range")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unfurl the arm, either immediately or as a joystck-controlled server.")
    parser.add_argument("--joystick", action="store_true", help="Run as server that unfurls on command from joystick.")
    args, unknown = parser.parse_known_args()

    rospy.init_node("unfurl_arm")
    rospy.loginfo("New unfurl script running")

    if args.joystick:
        t = TuckArmTeleop()
        rospy.spin()
    else:
        rospy.loginfo("Tucking the arm")
        TuckThread()
