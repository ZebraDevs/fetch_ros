#!/usr/bin/env python

import argparse
import subprocess
import sys
from threading import Thread

import rospy
import numpy
import math
import actionlib
import tf
from tf.listener import TransformListener
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TwistStamped
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from agile_grasp.msg import Grasps
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
import sensor_msgs.point_cloud2 as pc2

speaknspell = None
grasp_todo = None
markers = None

class AgileGrasp():
    _offset = 0.30

    def __init__(self, grasp):
        self._axis = numpy.array([grasp.axis.x, grasp.axis.y, grasp.axis.z])
        self._approach = numpy.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
        self._center = numpy.array([grasp.center.x, grasp.center.y, grasp.center.z])
        self._surface_center = numpy.array([grasp.surface_center.x, grasp.surface_center.y, grasp.surface_center.z])
        self._binormal = numpy.cross(self._axis, -self._approach)

    def orientation(self):

        flip = tf.transformations.rotation_matrix(math.pi, self._approach)[:3, :3].dot(
            tf.transformations.rotation_matrix(math.pi / 2, self._approach)[:3, :3]
        )

        rot = numpy.zeros((4, 4))
        rot[:3, 0] = flip.dot(self._approach)
        rot[:3, 1] = flip.dot(self._axis)
        rot[:3, 2] = numpy.cross(rot[0:3, 0], rot[0:3, 1])
        rot[3, 3] = 1

        quat = tf.transformations.quaternion_from_matrix(rot)
        quat /= numpy.linalg.norm(quat)

        orientation = Quaternion()
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]

        return orientation

    def position(self):
        position = Point()

        position.x = self._center[0] - self._approach[0] * self._offset
        position.y = self._center[1] - self._approach[1] * self._offset
        position.z = self._center[2] - self._approach[2] * self._offset

        return position

    def twist(self, frame):
        twist = TwistStamped()
        twist.header.frame_id = "end_effector_frame"
        twist.twist.linear.x = 0.1
        twist.twist.linear.y = 0
        twist.twist.linear.z = 0

        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        twist.twist.angular.z = 0

        return twist

    def marker(self, frame):
        marker = Marker()
        marker.header.frame_id = frame

        pose = Pose()
        pose.position = self.position()
        pose.orientation = self.orientation()

        marker.pose = pose

        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1
        marker.color.a = 1
        marker.color.g = 0.2
        marker.color.b = 0.2

        return marker

class GripperRequest():

    OPEN_POSITION = 0.115
    CLOSED_POSITION = 0.0


    def __init__(self):
        self.max_effort = 100.0
        self.position = None

        self._sub_pos = rospy.Subscriber('joint_states', JointState, self._set_state)

        self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action',
                                                   GripperCommandAction)
        rospy.loginfo('Waiting for gripper_controller...')
        self.client.wait_for_server()
        rospy.loginfo('...connected.')

    def _set_state(self, joint_state):
        l_gripper_finger_pos = None
        r_gripper_finger_pos = None
        for joint, pos in zip(joint_state.name, joint_state.position):
            if joint == 'l_gripper_finger_joint':
                l_gripper_finger_pos = pos
            if joint == 'r_gripper_finger_joint':
                r_gripper_finger_pos = pos

        if l_gripper_finger_pos and r_gripper_finger_pos:
            self.position = l_gripper_finger_pos + r_gripper_finger_pos

    def set_position(self, position):
        goal = GripperCommandGoal()
        goal.command.max_effort = self.max_effort
        goal.command.position = position
        # Fill in the goal here
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        if self.position < 0.01: # went all the way
            return False
        elif self.position > 0.01 and self.position < 0.1:
            return True

    def open(self):
        self.set_position(self.OPEN_POSITION)

    def close(self):
        return self.set_position(self.CLOSED_POSITION)

class MoveItRequest():

    def __init__(self, group, frame, plan_only = False):
        self._group = group
        self._fixed_frame = frame
        self._action = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self._action.wait_for_server()

        self._listener = TransformListener()

        self._plan_only = plan_only
        self._planner_id = "RRTConnectkConfigDefault"
        self._planning_time = 30.0

    def cancel_goal(self):
        self._action.cancel_goal()

    def moveToPose(self, pose_stamped, gripper_frame, scaling = 0.15):
        g = MoveGroupGoal()
        pose_transformed = self._listener.transformPose(self._fixed_frame, pose_stamped)
        g.request.start_state.is_diff = True
        g.request.planner_id = self._planner_id
        g.request.group_name = self._group
        g.request.allowed_planning_time = self._planning_time
        g.request.num_planning_attempts = 5
        g.request.max_velocity_scaling_factor = scaling
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = self._plan_only
        g.planning_options.look_around = False
        g.planning_options.max_safe_execution_cost = 0.15
        g.planning_options.replan = True

        tolerance = 0.01
        c1 = Constraints()

        c1.position_constraints.append(PositionConstraint())
        c1.position_constraints[0].header.frame_id = self._fixed_frame
        c1.position_constraints[0].link_name = gripper_frame
        b = BoundingVolume()
        s = SolidPrimitive()
        s.dimensions = [tolerance * tolerance]
        s.type = s.SPHERE
        b.primitives.append(s)
        b.primitive_poses.append(pose_transformed.pose)
        c1.position_constraints[0].constraint_region = b
        c1.position_constraints[0].weight = 1.0

        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.frame_id = self._fixed_frame
        c1.orientation_constraints[0].orientation = pose_transformed.pose.orientation
        c1.orientation_constraints[0].link_name = gripper_frame
        c1.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
        c1.orientation_constraints[0].absolute_z_axis_tolerance = tolerance
        c1.orientation_constraints[0].weight = 1.0

        g.request.goal_constraints.append(c1)

        i = 0
        while i < 1:
            self._action.send_goal(g)
            self._action.wait_for_result()
            text = self._action.get_goal_status_text()
            print "====================================="
            print "====================================="
            print "====================================="
            print text
            print "====================================="
            print "====================================="
            print "====================================="
            if text == "Solution was found and executed.":
               return True

            i += 1

        return False

class AgileThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["roslaunch", "agile_grasp", "grasps.launch"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_agile_running():
    output = subprocess.check_output(["rosnode", "info", "find_grasps"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

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

class GraspThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        rospy.loginfo("Checking if move_group is running...")
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()

        rospy.loginfo("Checking if agile_grasp is running...")
        agile_thread = None
        if not is_agile_running():
            rospy.loginfo("starting agile")
            agile_thread = AgileThread()

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveItRequest("arm", "base_link")
        rospy.loginfo("...connected")

        self.gripper = GripperRequest()
        self.gripper.open()

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        # scene = PlanningSceneInterface("base_link")
        # scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        speaknspell.voiceSound("Looking for grasp...").repeat()
        rospy.sleep(2)
        rospy.loginfo("Waiting for grasp...")
        i = 0
        global grasp_todo
        while grasp_todo == None and i < 10:
            rospy.sleep(1)
            i += 1
        grasp = grasp_todo

        if agile_thread:
            agile_thread.stop()

        if i == 10:
            speaknspell.voiceSound("No Grasp Pose Found. Try Again Later.").repeat()
            rospy.loginfo("No grasp pose found")
            rospy.sleep(2)
            return

        rospy.loginfo("Grasp found!")
        speaknspell.voiceSound("Grasp found! Planning to pose.").repeat()
        rospy.sleep(2);

        frame = "/head_camera_rgb_optical_frame"
        pose_pregrasp = PoseStamped()
        pose_pregrasp.header.frame_id = frame
        pose_pregrasp.pose.position = grasp.position()
        pose_pregrasp.pose.orientation = grasp.orientation()

        markerArray = MarkerArray()
        markerArray.markers.append(grasp.marker(frame))
        markers.publish(markerArray)

        while not rospy.is_shutdown():
            result = self.client.moveToPose(pose_pregrasp,
                                            "wrist_roll_link")

            if result:
                speaknspell.voiceSound("Plan was found and executed. Starting grasp!").repeat()
                rospy.sleep(1)
                self.servo = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped)

                twist = grasp.twist(frame)

                last = rospy.Time.now()
                while rospy.Time.now() < last + rospy.Duration(1.5):
                    self.servo.publish(twist)

                if not self.gripper.close():
                    speaknspell.voiceSound("Failed to grasp object. Sorry!").repeat()
                    rospy.sleep(3)

                else:
                    speaknspell.voiceSound("Object has been grasped!").repeat()

                    twist.header.frame_id = "base_link"
                    twist.twist.linear.x = 0
                    twist.twist.linear.y = 0
                    twist.twist.linear.z = 0.05
                    twist.twist.angular.x = 0
                    twist.twist.angular.y = 0
                    twist.twist.angular.z = 0

                    last = rospy.Time.now()
                    while rospy.Time.now() < last + rospy.Duration(2.5):
                        self.servo.publish(twist)

                    twist = TwistStamped()
                    twist.twist.linear.z = 0
                    self.servo.publish(twist)

            else:
                speaknspell.voiceSound("Failed to find plan. Sorry!").repeat()

            if move_thread:
                move_thread.stop()

            rospy.signal_shutdown("done")
            sys.exit(0)
            return

    def stop(self):
        if self.client:
            self.client.cancel_goal()

        speaknspell.voiceSound("Cancelling grasp.").repeat()
        rospy.sleep(3)

        try:
            twist = TwistStamped()
            twist.header.frame_id = "base_link"
            twist.twist.linear.x = 0
            twist.twist.linear.y = 0
            twist.twist.linear.z = 0
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = 0
            self.servo.publish(twist)
        except:
            pass

        rospy.signal_shutdown("failed")
        sys.exit(0)

class GraspTeleop:

    def __init__(self):
        self.grasp_button = rospy.get_param("~grasp_button", 15)  # default button is the down button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.grasping = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.grasping:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't grasp
                rospy.loginfo("Stopping grasp thread")
                self.grasp_thread.stop()
            return
        try:
            if msg.buttons[self.grasp_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Grasp the arm
                    self.grasping = True
                    rospy.loginfo("Starting grasp thread")
                    self.grasp_thread = GraspThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("grasp_button is out of range")


def grasp_callback(grasps):
    global grasp_todo

    grasp_todo = None
    m = float('Inf')
    for grasp in grasps.grasps:
        v = [grasp.surface_center.x, grasp.surface_center.y, grasp.surface_center.z]
        n = numpy.linalg.norm(v)
        if n < m:
            grasp_todo = AgileGrasp(grasp)
            m = n

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Grasp an object that is seen with agile_grasp.")
    parser.add_argument("--joystick", action="store_true", help="Run as server that grasps on command from joystick.")
    args, unknown = parser.parse_known_args()

    rospy.init_node("joystick_grasp")
    rospy.loginfo("New joystick grasp program running.")

    rospy.loginfo("Connecting to sound_play")
    speaknspell = SoundClient()
    rospy.loginfo("Connected!")
    speaknspell.stopAll()

    rospy.loginfo("Connecting to agile_grasp")
    grasp_sub = rospy.Subscriber("/find_grasps/grasps", Grasps, grasp_callback)
    rospy.loginfo("Connected!")

    markers = rospy.Publisher('visualization_marker_array', MarkerArray)

    if args.joystick:
        t = GraspTeleop()
        rospy.spin()
    else:
        rospy.loginfo("Grasping!")
        GraspThread()
