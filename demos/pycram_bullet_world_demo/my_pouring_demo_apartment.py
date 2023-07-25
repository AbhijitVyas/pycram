import sys
import os
sys.path.append(os.getcwd() + "/../../src/")
import macropy
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
from pycram.resolver.plans import Arms
from pycram.enums import Arms



world = BulletWorld()

# spawn apartment
apartment = Object("apartment", "environment", "apartment.urdf")
apartment_desig = ObjectDesignatorDescription(names=['apartment']).resolve()

# spawn pr2
pr2 = Object("pr2", "robot", "pr2.urdf", position=[1, 2.5, 0])
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

# pouring plan begins
def pouring_plan(source_obj, source_obj_desig, destination_obj, destination_obj_desig, pouring_angle):
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.3]).resolve().perform()

        pickup_pose = CostmapLocation(target=source_obj_desig.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = pickup_pose.reachable_arms[0]
        print('pickup_arm', pickup_arm)

        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        PickUpAction(object_designator_description=source_obj_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # do pouring by tilting, and accept time interval and speed as well
        quaternion = tf.transformations.quaternion_from_euler(pouring_angle, 0, 0, axes="sxyz")
        print('quaternion', quaternion)

        # tilting_pose = SemanticCostmapLocation.Location(pose=[[2.52, 2.86, 1.1], quaternion])
        # revert_tilting_pose = SemanticCostmapLocation.Location(pose=[[2.52, 2.86, 1.1], [0.0, 0, 0, 1]])
        tilting_pose = SemanticCostmapLocation.Location(pose=[destination_obj.original_pose[0], quaternion])
        revert_tilting_pose = SemanticCostmapLocation.Location(pose=[destination_obj.original_pose[0], [0.0, 0, 0, 1]])

        PourAction(source_obj_desig, pouring_location=[tilting_pose.pose], revert_location=[revert_tilting_pose.pose],
                   arms=[pickup_arm], wait_duration= 5).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        place_pose = SemanticCostmapLocation.Location(pose=[source_obj.original_pose[0], [0.0, 0, 0, 1.0]])

        PlaceAction(source_obj_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        end_pose = SemanticCostmapLocation.Location(pose=[[1, 2.5, 0], [0.0, 0, 0, 1.0]])
        # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[end_pose.pose]).resolve().perform()

pouring_plan(milk, milk_desig, bowl, bowl_desig, 90)