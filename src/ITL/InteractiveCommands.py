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


#
# # spawn water in a cup
# waterObj = Object("water", "water", "water.urdf", pose=Pose([2.4, 2.8, 0.98]))
#
# p.loadURDF("water.urdf", [2.5, 2.8, 0.98])
# bowl.attach(waterObj)





NEAR_STOVE = [2.5, 2, 0.95]
ON_STOVE = [2.5, 1.5, 0.95]
ON_EDGE_OF_COUNTER_TOP = [2.5, 1.25, 0.95]


class BootstrapInstructions:
    """
    Low-level interface to PyCRAM, which provides the easy interaction methods in Python.
    TODO: Hook it up with GUI where we can provide options as dropbox where user can select an environment
    as well as robot agent (right now mainly pr2).
    """

    def __init__(self, environment_name, robot_agent_name, activity_type):
        self.environment = None
        self.robot_agent = None
        self.robot_desig = None

        self.milk = None
        self.bowl = None
        self.breakfast_cereal = None
        self.SM_Cup = None
        self.jeroen_cup = None
        self.milk_desig = None
        self.bowl_desig = None
        self.breakfast_cereal_desig = None
        self.SM_Cup_desig = None
        self.jeroen_cup_desig = None

        self.load_environment(environment_name)
        self.load_robot_agent(robot_agent_name)
        self.load_assets(activity_type)

        # self.pouring_plan_from_instructions(self.milk, self.milk_desig,
        #                                     self.bowl, self.bowl_desig,
        #                                     [120,0,0], 5,
        #                                     'right', [2,2,20])

    def load_environment(self, environment_name):
        if environment_name == "Apartment":
            # spawn an apartment
            self.environment = Object("apartment", "environment", "apartment.urdf")
        elif environment_name == "Kitchen":
            # spawn a Kitchen
            self.environment = Object("kitchen", "environment", "kitchen.urdf")

    def load_robot_agent(self, robot_agent_name):
        if robot_agent_name == "PR2":
            self.robot_agent = Object("pr2", "robot", "../../resources/cached/pr2.urdf", pose=Pose([1.2, 2.5, 0]))
            self.robot_desig = BelieveObject(names=["pr2"])

    def load_assets(self, activity_type):
        if activity_type == "Pouring":
            # spawn Milkbox
            self.milk = Object("milk", "milk", "milk.stl", pose=Pose([2.4, 2.5, 1]))
            self.milk_desig = BelieveObject(names=["milk"])


            # spawn bowl
            self.bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.4, 2.8, 0.98]))
            self.bowl_desig = BelieveObject(names=["bowl"])

            # spawn bowl
            self.breakfast_cereal = Object("breakfast_cereal", "breakfast_cereal", "breakfast_cereal.stl",
                                      pose=Pose([2.5, 1.8, 0.98]))
            self.breakfast_cereal_desig = BelieveObject(names=["breakfast_cereal"])

            # spawn SM_Cup
            # self.SM_Cup = Object("SM_Cup", "SM_Cup", "SM_Cup.stl", pose=Pose([2.5, 1.9, 0.98]))
            # self.SM_Cup_desig = BelieveObject(names=["SM_Cup"])

            #
            # # spawn SM_CokeBottle
            # SM_CokeBottle = Object("SM_CokeBottle", "SM_CokeBottle", "SM_CokeBottle.stl", pose=Pose([2.5, 3, 0.95]))
            # SM_CokeBottle_desig = BelieveObject(names=["SM_CokeBottle"])
            #
            # # spawn jeroen_cup
            self.jeroen_cup = Object("jeroen_cup", "jeroen_cup", "jeroen_cup.stl", pose=Pose([2.5, 3.1, 1.0]))
            self.jeroen_cup_desig = BelieveObject(names=["jeroen_cup"])

    # pouring plan from console
    def pouring_plan_from_instructions(self, source_obj, source_obj_desig, destination_obj, destination_obj_desig, pouring_angle,
                                       pouring_time, pouring_hand, pouring_pose):
        with simulated_robot:
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            MoveTorsoAction([0.31]).resolve().perform()

            # perform picking up action
            pick_up_counter = 0
            while True:
                is_successful = self.do_pick_up(source_obj_desig, pouring_hand)
                pick_up_counter += 1
                if is_successful or pick_up_counter > 3:
                    break

            # perform pouring action
            pour_counter = 0
            while True:
                is_successful = self.do_pour(source_obj_desig, destination_obj, destination_obj_desig, pouring_hand,
                                        pouring_angle, pouring_time, pouring_pose)
                pour_counter += 1
                if is_successful or pour_counter > 3:
                    break

            # perform placing action
            place_counter = 0
            while True:
                is_successful = self.do_place(source_obj, source_obj_desig, pouring_hand)
                place_counter += 1
                if place_counter > 3:
                    self.putdown_plan(ON_STOVE, source_obj_desig, pouring_hand)
                    break
                if is_successful:
                    break

            self.go_back_to_original_position()

    def do_pick_up(self, source_obj_desig, pouring_hand):
        with simulated_robot:

            pickup_pose = None
            while True:
                pickup_pose = self.calculate_robot_pose(source_obj_desig, pouring_hand)
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
                print("Picking up the object successful")
                return True
            except Exception as e:
                print("exception while picking up:", e)
                return False

    def do_place(self, source_obj, source_obj_desig, pouring_hand):
        with simulated_robot:
            place_nav_pose = None
            while True:
                place_nav_pose = self.calculate_robot_pose(source_obj_desig, pouring_hand)
                if (place_nav_pose != None):
                    break
            # print('place_nav_pose: ', place_nav_pose)
            # go close to the destination container
            print("Navigate to place navigation pose")
            NavigateAction(target_locations=[place_nav_pose.pose]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

            place_pose = SemanticCostmapLocation.Location(
                pose=Pose(source_obj.original_pose.position_as_list(), [0, 0, 0, 1]))
            print('perform place action')
            try:
                PlaceAction(source_obj_desig, target_locations=[place_pose.pose],
                            arms=[pouring_hand]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                return True
            except Exception as e:
                print("Exception while doing placing:", e)
                return False

    def calculate_robot_pose(self, source_obj_desig, pouring_hand):
        robot_pose = CostmapLocation(target=source_obj_desig.resolve(), reachable_for=self.robot_desig.resolve(),
                                     reachable_arm=pouring_hand).resolve()
        # check if the calculated hand is the correct hand, otherwise return false
        if pouring_hand in robot_pose.reachable_arms:
            return robot_pose
        else:
            print("Not possible to perform action with given arm: ", pouring_hand, ". Please try one more time")
            return None

    def calculate_pour_pose(self, destination_obj_desig, pouring_hand, quaternion):
        pour_pose = CostmapLocation(target=destination_obj_desig.resolve(), reachable_for=self.robot_desig.resolve(),
                                    reachable_arm=pouring_hand).resolve()
        # check if the calculated hand is the correct hand, otherwise return false
        if pouring_hand in pour_pose.reachable_arms:
            return pour_pose
        else:
            print("Not possible to pour with given arm: ", pouring_hand, ". Please try one more time")
            return None

    def Park_Arms_Action(self):
        with simulated_robot:
            ParkArmsAction([Arms.BOTH]).resolve().perform()

    # only use this method internally not from ipython terminal
    def go_back_to_original_position(self):
        with simulated_robot:
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            final_pose = SemanticCostmapLocation.Location(pose=Pose([1, 2.5, 0], [0.0, 0, 0, 1.0]))
            # print('final pose: ', final_pose)
            # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
            NavigateAction(target_locations=[final_pose.pose]).resolve().perform()

    def do_pour(self, source_obj_desig, destination_obj, destination_obj_desig, pouring_hand: str, pouring_angle,
                pouring_time: float, pouring_pose):
        with simulated_robot:
            # calculate quaternion from the pouring angle data
            quaternion = tf.transformations.quaternion_from_euler(math.radians(pouring_angle[0]),
                                                                  math.radians(pouring_angle[1]),
                                                                  math.radians(pouring_angle[2]), axes="sxyz")
            # print('perform quaternion', quaternion)
            pour_pose = None
            while True:
                pour_pose = self.calculate_pour_pose(destination_obj_desig, pouring_hand, quaternion)
                if (pour_pose != None):
                    break
            # print('pouring pose: ', pour_pose)
            # go close to the destination container
            print("Navigate to pour pose")
            NavigateAction(target_locations=[pour_pose.pose]).resolve().perform()
            ParkArmsAction([Arms.BOTH]).resolve().perform()

            # calculate tilting pose
            destination_obj_pose = destination_obj.original_pose.position_as_list()
            print("dest pose", destination_obj_pose)
            # take a note: pouring_pose is in cm, and we should convert it to meter for pybullet.
            source_obj_pose = [destination_obj_pose[0] - pouring_pose[0] * 0.01,
                               destination_obj_pose[1] - pouring_pose[1] * 0.01,
                               destination_obj_pose[2] + pouring_pose[2] * 0.01]

            print("source_obj_pose", source_obj_pose)
            tilting_pose = SemanticCostmapLocation.Location(
                pose=Pose(source_obj_pose, list(quaternion)))

            # print('tilting pose: ', tilting_pose)
            revert_tilting_pose = SemanticCostmapLocation.Location(
                pose=Pose(source_obj_pose, [0.0, 0, 0, 1]))
            # print('revert tilting pose: ', revert_tilting_pose)
            print('perform pouring action')
            # do pouring by tilting, and accept time interval
            try:
                PourAction(source_obj_desig, pouring_location=[tilting_pose.pose],
                           revert_location=[revert_tilting_pose.pose],
                           arms=[pouring_hand], wait_duration=pouring_time).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                print("Pouring successful")
                return True
            except Exception as e:
                print("Exception while doing pouring:", e)
                return False

    def putdown_plan(self, location, source_obj_desig, pickup_arm):
        with simulated_robot:
            ParkArmsAction([Arms.BOTH]).resolve().perform()
            MoveTorsoAction([0.31]).resolve().perform()

            place_pose = CostmapLocation(target=Pose(location, [0, 0, 0, 1]),
                                         reachable_for=self.robot_desig.resolve()).resolve()

            print("Navigate to place pose")
            NavigateAction(target_locations=[place_pose.pose]).resolve().perform()

            place_pose = SemanticCostmapLocation.Location(pose=Pose(location, [0, 0, 0, 1]))
            # print('place pose: ', place_pose)
            print('perform place action')
            PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

            ParkArmsAction([Arms.BOTH]).resolve().perform()

