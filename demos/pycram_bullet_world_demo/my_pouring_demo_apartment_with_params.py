import sys
import os
print(os.getcwd())
sys.path.append(os.getcwd() + "/../../../src/")

import pycram
from pycram.bullet_world import BulletWorld, Object
import pycram.bullet_world_reasoning as btr
import tf
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from pycram.designators.motion_designator import MotionDesignatorDescription, MoveArmJointsMotion
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.language import macros, par
from pycram.designators.location_designator import *
from pycram.designators.action_designator import *
# from pycram.resolver.plans import Arms
from pycram.enums import Arms



world = BulletWorld()

# spawn apartment
apartment = Object("apartment", "environment", "apartment.urdf")
apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()

# spawn pr2
pr2 = Object("pr2", "robot", "pr2.urdf", position=[1.2, 2.5, 0])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

# spawn Milkbox
milk = Object("milk", "milk", "milk.stl", position=[2.5, 2.5, 1])
milk_desig = ObjectDesignatorDescription(names=["milk"])
print("milk", milk.original_pose[0])

# spawn bowl
bowl = Object("bowl", "bowl", "bowl.stl", position=[2.5, 2.8, 0.98])
bowl_desig = ObjectDesignatorDescription(names=["bowl"])

# spawn SM_CokeBottle
SM_CokeBottle = Object("SM_CokeBottle", "SM_CokeBottle", "SM_CokeBottle.stl", position=[2.5, 3, 0.95])
SM_CokeBottle_desig = ObjectDesignatorDescription(names=["SM_CokeBottle"])

NEAR_STOVE = [2.5,2,0.95]
ON_STOVE = [2.5,1.5,0.95]
ON_EDGE_OF_COUNTER_TOP = [2.5,1.25,0.95]


# def reset_plan():
#     ParkArmsAction([Arms.BOTH]).resolve().perform()
#     NavigateAction(target_locations=[pr2.original_pose[0]]).resolve().perform()

# pouring plan begins
def pouring_plan(source_obj, source_obj_desig, destination_obj, destination_obj_desig, pouring_angle, pouring_time):

    with simulated_robot:
        print("source obj current pose ", source_obj.pose)
        print("destination obj location ", destination_obj.pose)


        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.3]).resolve().perform()
        pickup_pose = CostmapLocation(target=[[source_obj.pose[0],source_obj.pose[1],source_obj.pose[2]],[0,0,0,1]], reachable_for=robot_desig).resolve()
        print("pickup pose: ", pickup_pose)
        pickup_arm = pickup_pose.reachable_arms[0]
        print('pickup_arm', pickup_arm)
        print("Navigate to pickup pose")
        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        print("Perform pickup action")
        PickUpAction(object_designator_description=source_obj_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # navigate to destination obj first before doing pouring
        # print("Navigate to destination obj")
        # NavigateAction(target_locations=[[destination_obj.pose[0],destination_obj.pose[1],destination_obj.pose[2]],[0,0,0,1]]).resolve().perform()

        # do pouring by tilting, and accept time interval and speed as well
        quaternion = tf.transformations.quaternion_from_euler(pouring_angle, 0, 0, axes="sxyz")
        print('perform quaternion', quaternion)
        tilting_pose = SemanticCostmapLocation.Location(pose=[[destination_obj.pose[0],destination_obj.pose[1],destination_obj.pose[2]], quaternion])
        print('tilting pose: ', tilting_pose)
        revert_tilting_pose = SemanticCostmapLocation.Location(pose=[[destination_obj.pose[0],destination_obj.pose[1],destination_obj.pose[2]], [0.0, 0, 0, 1]])
        print('revert tilting pose: ', revert_tilting_pose)
        print('perform pouring action')
        PourAction(source_obj_desig, pouring_location=[tilting_pose.pose], revert_location=[revert_tilting_pose.pose],
                   arms=[pickup_arm], wait_duration= pouring_time).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=[source_obj.original_pose[0],[0,0,0,1]])
        print('place pose: ', place_pose)
        print('perform place action')
        PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

        go_back_to_original_position()


def Park_Arms_Action():
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

# only use this method internally not from ipython terminal
def go_back_to_original_position():
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        final_pose = SemanticCostmapLocation.Location(pose=[[1, 2.5, 0], [0.0, 0, 0, 1.0]])
        print('final pose: ', final_pose)
        # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[final_pose.pose]).resolve().perform()


def pickup_plan(source_obj, source_obj_desig, grasp_arm=None, grasp_type=None):
    print("Given grasp type", grasp_type)
    print("Given grasp arm", grasp_arm)

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.3]).resolve().perform()
        pickup_pose = CostmapLocation(target=[[source_obj.pose[0],source_obj.pose[1],source_obj.pose[2]],[0,0,0,1]], reachable_for=robot_desig, reachable_arm=grasp_arm).resolve()
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
        PickUpAction(object_designator_description=source_obj_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()


def putdown_plan(location, source_obj_desig, pickup_arm):
    print("putdown location", location)

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        MoveTorsoAction([0.3]).resolve().perform()

        place_pose = CostmapLocation(target=[location, [0.0, 0, 0, 1.0]], reachable_for=robot_desig).resolve()
        # print('place pose: ', place_pose)
        # 
        print("Navigate to place pose")
        NavigateAction(target_locations=[place_pose.pose]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=[location, [0.0, 0, 0, 1.0]])
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

        NavigateAction(target_locations=[[[x,y,z], quaternion]]).resolve().perform()