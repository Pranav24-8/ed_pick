#!/usr/bin/env python

import rospy
import math
import sys
from ed_sensor_integration.srv import Update, UpdateRequest
from ed_msgs.srv import SimpleQuery, SimpleQueryRequest
from ed_msgs.msg import EntityInfo
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import (Constraints, MoveItErrorCodes,
                             OrientationConstraint, PickupAction, PickupGoal,
                             PlaceAction, PlaceGoal, PlanningOptions)
from sensor_msgs.msg import JointState
import tf
from iki_manipulation.recognized_object import RecognizedObject

class EdPick:
    def __init__(self):
        allowed_planning_time = 0.0
        ARM_AND_LINEAR_GROUP_NAME = "arm_and_linear"
        ARM_GROUP_NAME = "arm_torso"
        GRIPPER_GROUP = "gripper"

        REFERENCE_FRAME = "base_link"

        PLANNER = "RRTConnectkConfigDefault"
        #PLANNER = "RRTstarkConfigDefault"

        MOVE_GROUP_SERVICE = "/move_group/plan_execution/set_parameters"

        result = False
        upright_constraints = None
        robot = None
        arm = None
        arm_and_linear = None
        database = None
        scene = None
        grasp_generator = None
        place_generator = None
        marker_publisher = None


        self.robot = RobotCommander()
        self.arm = self.robot.get_group("arm_torso")
        self.gripper = self.robot.get_group(GRIPPER_GROUP)
        self.scene = PlanningSceneInterface()

        self.pickup_object = RecognizedObject("part")

    def get_object(self):

        rospy.wait_for_service('/ed/kinect/update')
        self.update_client = rospy.ServiceProxy('/ed/kinect/update', Update)

        self.request_update = UpdateRequest()
        self.request_update.area_description = "on_top_of dinner-table"

        self.result_update = self.update_client(self.request_update)

        print "Found IDs: " % self.result_update.new_ids

        rospy.wait_for_service('/ed/simple_query')
        self.query_client = rospy.ServiceProxy('/ed/simple_query', SimpleQuery)

        self.query_request = SimpleQueryRequest()
        self.query_request.id = self.result_update.new_ids[0]
        self.query_response = self.query_client(self.query_request)

    def pick_object(self):

        # self.object = self.query_response.entities
        self.pickup_object = RecognizedObject("part")
        tmpPoseStamped = PoseStamped()
        tmpPoseStamped.header.frame_id = '/map'
        tmpPoseStamped.header.stamp = rospy.Time(0)
        tmpPoseStamped.pose = self.query_response.entities[0].pose
        self.pickup_obj.pose = tmpPoseStamped
        self.value_x = []
        self.value_y = []
        self.value_z = []

        for j in range(0,len(self.query_response.entities[0].convex_hull)):
            self.value_x.append(self.query_response.entities[0].convex_hull[j].x)
            self.value_y.append(self.query_response.entities[0].convex_hull[j].y)
            self.value_z.append(self.query_response.entities[0].convex_hull[j].z)
        self.dim_x = max(self.value_x) - min(self.value_x)
        self.dim_y = max(self.value_y) - min(self.value_y)
        self.dim_z = max(self.value_z) - min(self.value_z)
        self.pickup_object.dimensions = (self.dim_x, self.dim_y, self.dim_z)
        self.add_to_planning_scene(self.pickup_object)
        self.grasps = self.generate_grasps_for_object(self.pickup_object)
        if not self.pickup(self.pickup_object, self.grasps) == -1:
            print "Executed pick action"
            self.attach_object_to_gripper(self.pickup_object)

    def add_to_planning_scene(self, recognized_object):

        pose_stamped = copy.deepcopy(recognized_object.pose)

        self.scene.add_box(recognized_object.name, pose_stamped, (recognized_object.dimensions[0], recognized_object.dimensions[1], recognized_object.dimensions[2]))

        rospy.loginfo("Object {} added to planning scene".format(recognized_object.name))

        rospy.logdebug("Object {} added with pose \n{} \nand dimensions {}".format(recognized_object.name, pose_stamped, recognized_object.dimensions))

    def attach_object_to_gripper(self, recognized_object):
        rospy.loginfo("Attach object to gripper")

        pose_stamped = copy.deepcopy(recognized_object.pose)

        # self.remove_attached_object(recognized_object.name)
        self.scene.remove_world_object(recognized_object.name)

        self.scene.attach_box(END_EFFECTOR_LINK_NAME, recognized_object.name, pose_stamped, (recognized_object.dimensions[0], recognized_object.dimensions[1], recognized_object.dimensions[2]), GRIPPER_LINKS_TO_IGNORE_COLLISION)

    def pickup(self, pickup_object, grasps):
        supp_surface = 'table',

        rospy.loginfo("Recognized object: " + str(pick_obj))

        self.arm.set_support_surface_name(supp_surface)

        result = self.arm.pick(pick_object.name, grasps)

        self.print_moveit_result(result)

        return result

    def generate_grasps_for_object(self, recognized_object):
        self.recognized_object = recognized_object

        self.base_grasp_pose_stamped = copy.deepcopy(self.recognized_object.pose)
        self.horizontal_center_grasp_pose_stamped = self.base_grasp_pose_stamped
        self.temp_grasp_pose_stamped = self.base_grasp_pose_stamped

        self.grasp_quality = 1.0

        if recognized_object.dimensions[2] < self.high_res_threshold:
            rospy.loginfo("Z axis of object ({} m) is shorter than threshold {}. \
                create denser grasp poses.".format(recognized_object.dimensions[2], self.high_res_threshold))
            self.distance_vertical = self.distance_vertical_default/2.0
            self.padding_from_top_and_bottom = self.padding_from_top_and_bottom_default/2.0

        else:
            rospy.loginfo("Z axis of object ({} m) is longer than threshold {}. \
                create normal distance grasp poses.".format(recognized_object.dimensions[2], self.high_res_threshold))
            self.distance_vertical = self.distance_vertical_default
            self.padding_from_top_and_bottom = self.padding_from_top_and_bottom_default


        self.number_of_poses_vertical = int(
            (recognized_object.dimensions[2] - float(self.padding_from_top_and_bottom)) / float(self.distance_vertical) / 2.0)

        rospy.loginfo("Number of vertical poses " + str(self.number_of_poses_vertical))

        # Generate the central grasp pose. Based on this pose the other poses are generated.


        # check if this is a top grasp (x axis faces in negative z direction of base_link) or a side grasp

        # pose with x downward, z forward
        pose_stamped_compare = PoseStamped()
        pose_stamped_compare.header.frame_id = "base_link"
        pose_stamped_compare.pose.position.x = 0.0
        pose_stamped_compare.pose.position.y = 0.0
        pose_stamped_compare.pose.position.z = 0.0
        pose_stamped_compare.pose.orientation.w = 0.7071
        pose_stamped_compare.pose.orientation.x = 0.0
        pose_stamped_compare.pose.orientation.y = 0.7071
        pose_stamped_compare.pose.orientation.z = 0.0


        angle = self.angle_between_poses(self.temp_grasp_pose_stamped, pose_stamped_compare)


        if angle < pi/4:
            # do a pinch grasp
            rospy.loginfo("This grasp is a top grasp. Do a pinch grasp")
            self.x_offset = -recognized_object.dimensions[0] / 2 + self.palm_to_finger_tips_distance
            self.recognized_object.top_grasp = True

        else:
            rospy.loginfo("This grasp is a side grasp. Do a power grasp")
            self.x_offset = recognized_object.dimensions[0] / 2 + self.additional_offset_in_grasp_direction
            self.recognized_object.top_grasp = False

        self.shift_pose_along_the_grippers_axes(self.temp_grasp_pose_stamped, [-self.x_offset, 0, 0])
        self.create_grasp(self.temp_grasp_pose_stamped)

        # vertical

        for vertical_counter in range(0, self.number_of_poses_vertical + 1):

            if self.recognized_object.rotational_symmetric:

                self.generate_horizontal_grasps(True)

                self.generate_vertical_grasp_upper_part(vertical_counter)


                self.generate_horizontal_grasps(True)

                self.generate_vertical_grasp_lower_part(vertical_counter)

            else:

                self.generate_horizontal_grasps(False)

                self.generate_vertical_grasp_upper_part(vertical_counter)

                self.generate_horizontal_grasps(False)

                self.generate_vertical_grasp_lower_part(vertical_counter)


            rospy.loginfo("Generated vertical pose " + str(vertical_counter))


        rospy.loginfo("Number of generated grasps: " + str(len(self.grasps)))

        for grasp in self.grasps:
            # rospy.logdebug("Publishing grasp z position:" + str(grasp.grasp_pose.pose.position))
            # rospy.loginfo("Grasp quality: %s" % (grasp.grasp_quality))

            return self.grasps

    def generate_vertical_grasp_upper_part(self, vertical_counter):
        self.generate_vertical_grasp("upper", vertical_counter)

    def generate_vertical_grasp_lower_part(self, vertical_counter):
        self.generate_vertical_grasp("lower", vertical_counter)

    def generate_vertical_grasp(self, part, vertical_counter):

        if part == "upper":
            distance_vertical = self.distance_vertical
        elif part == "lower":
            distance_vertical = -self.distance_vertical
        else:
            rospy.logerror('Part %s unkown. Only use "upper" or "lower"' % (part))

        self.horizontal_center_grasp_pose_stamped = copy.deepcopy(self.base_grasp_pose_stamped)
        self.temp_grasp_pose_stamped = self.horizontal_center_grasp_pose_stamped
        marvin_manipulation.transformations_common.shift_pose_along_the_grippers_axes(
            self.temp_grasp_pose_stamped, [0, 0, float(distance_vertical) * float(vertical_counter)])
        self.grasp_quality = 1.0 / (float(vertical_counter)+0.001) #avoid devided by zero
        self.create_grasp(self.temp_grasp_pose_stamped)

    def generate_horizontal_grasps(self, rotational_symmetric):

        if rotational_symmetric:
            rospy.loginfo('Rotational symmetric')

            for horizontal_counter in range(-self.number_of_poses_horizontal, self.number_of_poses_horizontal + 1):
                self.temp_grasp_pose_stamped = copy.deepcopy(self.horizontal_center_grasp_pose_stamped)
                self.temp_grasp_pose_stamped = self.rotate_pose_around_object_center(self.temp_grasp_pose_stamped, self.angle_horizontal * float(horizontal_counter), self.x_offset)
                self.create_grasp(self.temp_grasp_pose_stamped)

        else:
            rospy.loginfo('non symmetric')

            #create grasp facing in negative x direction
            self.temp_grasp_pose_stamped = copy.deepcopy(self.horizontal_center_grasp_pose_stamped)
            self.temp_grasp_pose_stamped = self.rotate_pose_around_object_center(self.temp_grasp_pose_stamped, pi, self.x_offset)
            self.create_grasp(self.temp_grasp_pose_stamped)

    def rotate_pose_around_object_center(self, pose_stamped, angle, distance_to_object_center):

        self.shift_pose_along_the_grippers_axes(pose_stamped, (distance_to_object_center, 0, 0))

        self.rotate_pose_around_axis_by_angle(pose_stamped, angle, (0, 0, 1))

        self.shift_pose_along_the_grippers_axes(pose_stamped,(-distance_to_object_center, 0, 0))

        return pose_stamped

    def create_grasp(self, pose_stamped):
        grasp = Grasp()
        grasp.grasp_pose = copy.deepcopy(pose_stamped)

        # pre grasp gripper configuration
        grasp.pre_grasp_posture.header.frame_id = self.gripper_frame
        grasp.pre_grasp_posture.header.stamp = rospy.Time.now()
        grasp.pre_grasp_posture.joint_names = self.finger_joint_names
        grasp.pre_grasp_posture.points.append(JointTrajectoryPoint(positions=[self.gripper_open_angle_radian,
                                                                              self.gripper_open_angle_radian],
                                                                   effort=[self.max_effort, self.max_effort]))
        # approach
        grasp.pre_grasp_approach.direction.header.frame_id = self.gripper_frame
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.min_distance = 0.04
        grasp.pre_grasp_approach.desired_distance = 0.15

        # grasp gripper configuration
        grasp.grasp_posture.header.frame_id = self.gripper_frame
        grasp.grasp_posture.header.stamp = rospy.Time.now()
        grasp.grasp_posture.joint_names = self.finger_joint_names
        grasp.grasp_posture.points.append(JointTrajectoryPoint(positions=[self.gripper_closed_angle_radian,
                                                                          self.gripper_closed_angle_radian],
                                                               effort=[self.max_effort, self.max_effort]))

        # TODO: add time_from_start here and for gripper open

        # retreat
        grasp.post_grasp_retreat.direction.header.frame_id = "base_link"

        grasp.post_grasp_retreat.direction.vector.z = 1.0

        grasp.post_grasp_retreat.min_distance = 0.03
        grasp.post_grasp_retreat.desired_distance = 0.05

        # objects allowed to touch while approaching
        grasp.allowed_touch_objects = [self.recognized_object.name]
        grasp.grasp_quality = self.grasp_quality

        self.grasps.append(grasp)

    def angle_between_poses(pose_stamped, pose_stamped_compare=None):
        if not pose_stamped_compare:

            #pose with x forward, z upward
            pose_stamped_compare = PoseStamped()
            pose_stamped_compare.header.frame_id = "base_link"
            pose_stamped_compare.pose.position.x = 0.0
            pose_stamped_compare.pose.position.y = 0.0
            pose_stamped_compare.pose.position.z = 0.0
            pose_stamped_compare.pose.orientation.w = 1.0
            pose_stamped_compare.pose.orientation.x = 0.0
            pose_stamped_compare.pose.orientation.y = 0.0
            pose_stamped_compare.pose.orientation.z = 0.0

        quaternion = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z,
                      pose_stamped.pose.orientation.w]

        compare_quaternion = [pose_stamped_compare.pose.orientation.x, pose_stamped_compare.pose.orientation.y,
                              pose_stamped_compare.pose.orientation.z, pose_stamped_compare.pose.orientation.w]

        inner_product = numpy.inner(quaternion, compare_quaternion)

        angle_between_poses = math.degrees(math.acos(2 * inner_product**2 - 1))

        print("Angle between poses: %s" % angle_between_poses)

        return angle_between_poses

    def shift_pose_along_the_grippers_axes(pose_stamped, shift_per_axis_array):
        '''
        Shifts given pose_stamped by the amounts given in shift_per_axis_array

        :param PoseStamped pose_stamped:
        :param () shift_per_axis_array: tupel of translations e.g. (0.5, 0, 0) to shift the pose 0.5m in x direction
        '''

        frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                      pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w),
            PyKDL.Vector(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))

        point = PyKDL.Vector(shift_per_axis_array[0], shift_per_axis_array[1], shift_per_axis_array[2])

        point = frame * point

        pose_stamped.pose.position.x = point.x()
        pose_stamped.pose.position.y = point.y()
        pose_stamped.pose.position.z = point.z()



if __name__ == '__main__':
    rospy.init_node("ed_pick")
    edpick = EdPick()
    edpick.get_object()
    edpick.pick_object()
