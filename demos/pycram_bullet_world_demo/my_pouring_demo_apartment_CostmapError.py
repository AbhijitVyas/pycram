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

pr2 = Object("pr2", "robot", "pr2.urdf", position=[1, 2.5, 0])
robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()

# spawn Milkbox
milk = Object("milk", "milk", "milk.stl", position=[2.5, 2.5, 1])
milk_desig = ObjectDesignatorDescription(names=["milk"])

# spawn bowl
bowl = Object("bowl", "bowl", "bowl.stl", position=[2.5, 2.8, 0.98])
bowl_desig = ObjectDesignatorDescription(names=["bowl"])

# pouring plan begins
while 1:
    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([0.3]).resolve().perform()

        pickup_pose = CostmapLocation(target=milk_desig.resolve(), reachable_for=robot_desig).resolve()
        pickup_arm = pickup_pose.reachable_arms[0]
        print('pickup_arm', pickup_arm)

        NavigateAction(target_locations=[pickup_pose.pose]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
        PickUpAction(object_designator_description=milk_desig, arms=[pickup_arm], grasps=["front"]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # do pouring by tilting, and accept time interval and speed as well
        quaternion = tf.transformations.quaternion_from_euler(90, 0, 0, axes="sxyz")
        print('quaternion', quaternion)

        pouring_pose = CostmapLocation(target=bowl_desig.resolve(), reachable_for=robot_desig).resolve()
        pouring_arm = pouring_pose.reachable_arms
        print('pouring_pose', pouring_pose)

        NavigateAction(target_locations=[pouring_pose.pose]).resolve().perform()
        tilting_pose = SemanticCostmapLocation.Location(pose=[[2.52, 2.86, 1.1], quaternion])
        # tilting_pose = SemanticCostmapLocation.Location(pose=[tilting_pose_cal, quaternion])
        # revert_tilting_pose = SemanticCostmapLocation.Location(pose=[tilting_pose_cal, [0.0, 0, 0, 1]])
        revert_tilting_pose = CostmapLocation([[2.4, 2.8, 1], [0, 0, 0, 1]], reachable_for=robot_desig,
                                        reachable_arm=pickup_arm).resolve()
        PourAction(milk_desig, pouring_location=[tilting_pose.pose], revert_location=[revert_tilting_pose.pose],
                   arms=[pickup_arm], wait_duration= 5).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # think about where to place the milk karton
        # apartment_desig = ObjectDesignatorDescription(names=['apartment'])
        # milk_desig = ObjectDesignatorDescription(names=["milk"])
        # place_island = SemanticCostmapLocation("sink", apartment_desig.resolve(), milk_desig.resolve()).resolve()
        # place_stand = CostmapLocation(place_island.pose, reachable_for=robot_desig, reachable_arm=pickup_arm).resolve()

        place_pose = SemanticCostmapLocation.Location(pose=[[2.5, 2.5, 1], [0.0, 0, 0, 1.0]])
        # NavigateAction(target_locations=[place_pose.pose]).resolve().perform()

        PlaceAction(milk_desig, target_locations=[place_pose.pose], arms=[pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        end_pose = SemanticCostmapLocation.Location(pose=[[1, 2.5, 0], [0.0, 0, 0, 1.0]])
        # end_pose = CostmapLocation(target=robot_desig.resolve(), reachable_for=robot_desig).resolve()
        NavigateAction(target_locations=[end_pose.pose]).resolve().perform()