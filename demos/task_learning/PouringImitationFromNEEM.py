#!/usr/bin/env python3
import sys
import os

sys.path.append(os.getcwd() + "/../../../src/")
import requests
import pycram
import neem_interface_python
from neem_interface_python.src.rest_neem_interface.neemdata import NEEMData
from pycram.bullet_world import BulletWorld, Object
import pycram.bullet_world_reasoning as btr
import tf
from pycram.robot_descriptions import InitializedRobotDescription as robot_description
from pycram.designators.motion_designator import MotionDesignatorDescription, MoveArmJointsMotion
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.language import macros, par
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
# from pycram.resolver.plans import Arms
from pycram.enums import Arms
# for radians
import math
import numpy as np

import pybullet as p

world = BulletWorld()
world.set_gravity([0, 0, -9.8])

# spawn apartment
# apartment = Object("apartment", "environment", "apartment.urdf")
# apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()
#
# # spawn pr2
# pr2 = Object("pr2", "robot", "pr2.urdf", position=[1.2, 2.5, 0])
# robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

# spawn Milkbox
milk = Object("milk", "milk", "milk.stl", pose=Pose([2.4, 2.5, 1]))
milk_desig = BelieveObject(names=["milk"])

# spawn bowl
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.4, 2.8, 0.98]))
bowl_desig = BelieveObject(names=["bowl"])

# spawn bowl
# breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl", position=[2.5, 1.8, 0.98])
# breakfast_cereal_desig = ObjectDesignatorDescription(names=["breakfast_cereal"])
#
# # spawn SM_CokeBottle
# SM_CokeBottle = Object("SM_CokeBottle", "SM_CokeBottle", "SM_CokeBottle.stl", position=[2.5, 3, 0.95])
# SM_CokeBottle_desig = ObjectDesignatorDescription(names=["SM_CokeBottle"])
#
# # spawn jeroen_cup
# jeroen_cup = Object("jeroen_cup", "jeroen_cup", "jeroen_cup.stl", position=[2.5, 3.1, 1.0])
# jeroen_cup_desig = ObjectDesignatorDescription(names=["jeroen_cup"])
#
# # spawn water in a cup
# waterObj = Object("water", "water", "water.urdf", position=[2.5, 2.8, 0.99])

# p.loadURDF("water.urdf", [2.5, 2.8, 0.99])
# bowl.attach(waterObj)

apartment = Object("apartment", "environment", "apartment.urdf")
pr2 = Object("pr2", "robot", "pr2.urdf", pose=Pose([1.2, 2.5, 0]))
robot_desig = BelieveObject(names=["pr2"])

NEAR_STOVE = [2.5, 2, 0.95]
ON_STOVE = [2.5, 1.5, 0.95]
ON_EDGE_OF_COUNTER_TOP = [2.5, 1.25, 0.95]


# pouring plan from console
def pouring_plan_from_instructions(source_obj, source_obj_desig, destination_obj, destination_obj_desig, pouring_angle,
                                   pouring_time, pouring_hand):
    with simulated_robot:
        print("source obj current pose ", source_obj.pose)
        print("destination obj location ", destination_obj.pose)
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.31]).resolve().perform()

        # perform picking up action
        while True:
            is_successful = do_pick_up(source_obj, source_obj_desig, pouring_hand)
            if (is_successful):
                break

        # perform pouring action
        while True:
            is_successful = do_pour(source_obj_desig, destination_obj, destination_obj_desig, pouring_hand,
                                    pouring_angle, pouring_time)
            if (is_successful):
                break

        # perform placing action
        while True:
            is_successful = do_place(source_obj, source_obj_desig, pouring_hand)
            if (is_successful):
                break

        go_back_to_original_position()


# pouring plan from NEEM
def pouring_plan_from_neems(pouring_hand):
    # get the pouring parameters from the NEEM
    (pouring_angle, pouring_time, source_obj, source_obj_desig, destination_obj,
     destination_obj_desig) = get_data_from_neem()

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.31]).resolve().perform()

        # perform picking up action
        while True:
            is_successful = do_pick_up(source_obj, source_obj_desig, pouring_hand)
            if (is_successful):
                break

        # perform pouring action
        while True:
            is_successful = do_pour(source_obj_desig, destination_obj, destination_obj_desig, pouring_hand,
                                    pouring_angle, pouring_time)
            if (is_successful):
                break

        # perform placing action
        while True:
            is_successful = do_place(source_obj, source_obj_desig, pouring_hand)
            if (is_successful):
                break

        go_back_to_original_position()


def get_data_from_neem():
    # get source container
    source_obj, source_obj_desig = None, None
    destination_obj, destination_obj_desig = None, None
    source = NEEMData().get_source_container_while_grasping().get("Obj")
    print("source container from NEEM ", source)
    # if('jeroen_cup' in source):
    #     source_obj = jeroen_cup
    #     source_obj_desig = jeroen_cup_desig
    # elif('bowl' in source):
    #     source_obj = bowl
    #     source_obj_desig = bowl_desig
    # elif('SM_CokeBottle' in source):
    #     source_obj = SM_CokeBottle
    #     source_obj_desig = SM_CokeBottle_desig
    # elif('breakfast_cereal' in source):
    #     source_obj = breakfast_cereal
    #     source_obj_desig = breakfast_cereal_desig
    if ('milk' in source):
        source_obj = milk
        source_obj_desig = milk_desig

    # get destination container
    destination = NEEMData().get_target_obj_for_pouring().get("Obj")
    print("destination container from NEEM: ", destination)
    # if('jeroen_cup' in destination):
    #     destination_obj = jeroen_cup
    #     destination_obj_desig = jeroen_cup_desig
    # elif('bowl' or 'Bowl' in destination):
    #     destination_obj = bowl
    #     destination_obj_desig = bowl_desig
    # elif('SM_CokeBottle' in destination):
    #     destination_obj = SM_CokeBottle
    #     destination_obj_desig = SM_CokeBottle_desig
    # elif('breakfast_cereal' in destination):
    #     destination_obj = breakfast_cereal
    #     destination_obj_desig = breakfast_cereal_desig
    if ('milk' in destination):
        destination_obj = milk
        destination_obj_desig = milk_desig

    # get pouring angle
    pouring_angle_obj = NEEMData().get_max_pouring_angle_for_source_obj()
    pouring_angle = pouring_angle_obj.get('AngleValue')
    print("pouring_angle: ", pouring_angle)

    # get pouring time
    pouring_time_obj = NEEMData().get_pouring_event_time_duration()
    pouring_time = (pouring_time_obj.get('End') - pouring_time_obj.get('Begin'))
    print("pouring_time:", pouring_time)
    return pouring_angle, pouring_time, source_obj, source_obj_desig, destination_obj, destination_obj_desig


def do_place(source_obj, source_obj_desig, pouring_hand):
    with simulated_robot:
        place_nav_pose = None
        while True:
            place_nav_pose = calculate_robot_pose(source_obj_desig, pouring_hand)
            if (place_nav_pose != None):
                break
        # print('place_nav_pose: ', place_nav_pose)
        # go close to the destination container
        print("Navigate to place navigation pose")
        NavigateAction(target_locations=[place_nav_pose.pose]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=Pose(source_obj.original_pose.position_as_list(), [0, 0, 0, 1]))
        print('place pose: ', place_pose)
        print('perform place action')
        try:
            PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pouring_hand]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            return True
        except Exception as e:
            print("Exception while doing placing:", e)
            return False


def do_pour(source_obj_desig, destination_obj, destination_obj_desig, pouring_hand, pouring_angle, pouring_time):
    with simulated_robot:

        quaternion = tf.transformations.quaternion_from_euler(math.radians(pouring_angle), 0, 0, axes="sxyz")
        # print('perform quaternion', quaternion)
        pour_pose = None
        while True:
            pour_pose = calculate_pour_pose(destination_obj_desig, pouring_hand, quaternion)
            if (pour_pose != None):
                break
        # print('pouring pose: ', pour_pose)
        # go close to the destination container
        print("Navigate to pour pose")
        NavigateAction(target_locations=[pour_pose.pose]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # calculate tilting pose
        tilting_pose = SemanticCostmapLocation.Location(
            pose=Pose(destination_obj.original_pose.position_as_list(), list(quaternion)))

        # print('tilting pose: ', tilting_pose)
        revert_tilting_pose = SemanticCostmapLocation.Location(
            pose=Pose(destination_obj.original_pose.position_as_list(), [0.0, 0, 0, 1]))
        # print('revert tilting pose: ', revert_tilting_pose)
        print('perform pouring action')
        # do pouring by tilting, and accept time interval
        try:
            PourAction(source_obj_desig, pouring_location=[tilting_pose.pose],
                       revert_location=[revert_tilting_pose.pose],
                       arms=[pouring_hand], wait_duration=pouring_time).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            return True
        except Exception as e:
            print("Exception while doing pouring:", e)
            return False


def do_pick_up(source_obj, source_obj_desig, pouring_hand):
    with simulated_robot:

        pickup_pose = None
        while True:
            pickup_pose = calculate_robot_pose(source_obj_desig, pouring_hand)
            if (pickup_pose != None):
                break
        # print("pickup pose: ", pickup_pose)
        print("Navigate to pickup pose")
        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Perform pickup action")
        try:
            PickUpAction(object_designator_description=source_obj_desig, arms=[pouring_hand],
                         grasps=["front"]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            return True
        except Exception as e:
            print("exception while picking up:", e)
            return False


def calculate_robot_pose(source_obj_desig, pouring_hand):
    robot_pose = CostmapLocation(target=source_obj_desig.resolve(), reachable_for=robot_desig.resolve(),
                                 reachable_arm=pouring_hand).resolve()
    # check if the calculated hand is the correct hand, otherwise return false
    if pouring_hand in robot_pose.reachable_arms:
        return robot_pose
    else:
        print("Not possible to perform action with given arm: ", pouring_hand, ". Please try one more time")
        return None


def calculate_pour_pose(destination_obj_desig, pouring_hand, quaternion):
    pour_pose = CostmapLocation(target=destination_obj_desig.resolve(), reachable_for=robot_desig.resolve(),
                                reachable_arm=pouring_hand).resolve()
    # check if the calculated hand is the correct hand, otherwise return false
    if pouring_hand in pour_pose.reachable_arms:
        return pour_pose
    else:
        print("Not possible to pour with given arm: ", pouring_hand, ". Please try one more time")
        return None


def Park_Arms_Action():
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()


# only use this method internally not from ipython terminal
def go_back_to_original_position():
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        final_pose = SemanticCostmapLocation.Location(pose=Pose([1, 2.5, 0], [0.0, 0, 0, 1.0]))
        print('final pose: ', final_pose)
        # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[final_pose.pose]).resolve().perform()


def pickup_plan(source_obj, source_obj_desig, grasp_arm=None, grasp_type=None):
    print("Given grasp type", grasp_type)
    print("Given grasp arm", grasp_arm)

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.31]).resolve().perform()
        pickup_pose = CostmapLocation(
            target=source_obj_desig.resolve(), reachable_for=robot_desig.resolve(),
            reachable_arm=grasp_arm).resolve()
        print("pickup pose: ", pickup_pose)
        pickup_arm = None
        if grasp_arm in pickup_pose.reachable_arms:
            pickup_arm = grasp_arm
        else:
            print("Not possible to pickup with given arm: ", grasp_arm, ". Please try one more time")
            return

        print('pickup_arm', pickup_arm)
        print("Navigate to pickup pose")
        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Perform pickup action")
        PickUpAction(object_designator_description=source_obj_desig, arms=[pickup_arm],
                     grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()


def putdown_plan(location, source_obj_desig, pickup_arm):
    print("putdown location", location)

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.31]).resolve().perform()

        place_pose = CostmapLocation(target=Pose(location, [0, 0, 0, 1]), reachable_for=robot_desig.resolve()).resolve()
        # print('place pose: ', place_pose)
        # 
        print("Navigate to place pose")
        NavigateAction(target_locations=[place_pose.pose]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=Pose(location, [0, 0, 0, 1]))
        print('place pose: ', place_pose)
        print('perform place action')
        PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # print('perform place action')
        # putting_down_pose = SemanticCostmapLocation.Location(pose=[location, [0.0, 0, 0, 1.0]])
        # PlaceAction(source_obj_desig, target_locations=[putting_down_pose.pose], arms=[pickup_arm]).resolve().perform()
        # 
        # ParkArmsAction([Arms.BOTH]).resolve().perform()


def move_plan(x, y, z, yaw):
    with simulated_robot:
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw, axes="sxyz")
        print("moving pr2 to the pose: ", x, y, z)
        print("moving pr2 to the orientation: ", quaternion)
        NavigateAction(target_locations=[Pose([x, y, z], quaternion)]).resolve().perform()
