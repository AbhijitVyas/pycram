import unittest
from pycram.designators import action_designator, object_designator
from pycram.robot_descriptions import robot_description
from pycram.process_module import simulated_robot
import pycram.enums
import test_bullet_world
import numpy as np


class TestActionDesignatorGrounding(test_bullet_world.BulletWorldTest):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = action_designator.MoveTorsoAction([0.3])
        self.assertEqual(description.ground().position, 0.3)
        with simulated_robot:
            description.resolve().perform()
        self.assertEqual(self.world.robot.get_joint_state(robot_description.torso_joint), 0.3)

    def test_set_gripper(self):
        description = action_designator.SetGripperAction(["left"], ["open", "close"])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().motion, "open")
        self.assertEqual(len(list(iter(description))), 2)
        with simulated_robot:
            description.resolve().perform()
        for joint, state in robot_description.get_static_gripper_chain("left", "open").items():
            self.assertEqual(self.world.robot.get_joint_state(joint), state)

    def test_release(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.ReleaseAction(["left"], object_description)
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_grip(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GripAction(["left"], object_description, [0.5])
        self.assertEqual(description.ground().gripper, "left")
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_park_arms(self):
        description = action_designator.ParkArmsAction([pycram.enums.Arms.BOTH])
        self.assertEqual(description.ground().arm, pycram.enums.Arms.BOTH)
        with simulated_robot:
            description.resolve().perform()
        for joint, pose in robot_description.get_static_joint_chain("right", "park").items():
            self.assertEqual(self.world.robot.get_joint_state(joint), pose)
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            self.assertEqual(self.world.robot.get_joint_state(joint), pose)

    def test_navigate(self):
        description = action_designator.NavigateAction([([0, 0, 0], [0, 0, 0, 1])])
        self.assertEqual(description.ground().target_location, ([0, 0, 0], [0, 0, 0, 1]))

    def test_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PickUpAction(object_description, ["left"], ["front"])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            action_designator.NavigateAction.Action(([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            action_designator.MoveTorsoAction.Action(0.3).perform()
            description.resolve().perform()
        self.assertTrue(object_description.resolve().bullet_world_object in self.robot.attachments.keys())

    def test_place(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [([1.3, 1, 0.9], [0, 0, 0, 1])], ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            action_designator.NavigateAction.Action(([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            action_designator.MoveTorsoAction.Action(0.3).perform()
            action_designator.PickUpAction.Action(object_description.resolve(), "left", "front").perform()
            description.resolve().perform()
        self.assertFalse(object_description.resolve().bullet_world_object in self.robot.attachments.keys())

    def test_look_at(self):
        description = action_designator.LookAtAction([[1, 0, 1]])
        self.assertEqual(description.ground().target, [1, 0, 1])
        with simulated_robot:
            description.resolve().perform()
        # TODO: Needs a way to test the approximate looking direction of the robot

    def test_detect(self):
        self.milk.set_position([1.5, 0, 1.2])
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.DetectAction(object_description)
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            detected_object = description.resolve().perform()
        self.assertEqual(detected_object.name, "milk")
        self.assertEqual(detected_object.type, "milk")
        self.assertEqual(detected_object.bullet_world_object, self.milk)

    def test_open(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.OpenAction(object_description, ["left"], [1])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_close(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.CloseAction(object_description, ["left"])
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_transport(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.TransportAction(object_description, ["left"], [([-0.9, 1, 0.95], [0, 0, 1, 0])])
        with simulated_robot:
            action_designator.MoveTorsoAction([0.2]).resolve().perform()
            description.resolve().perform()
        self.assertEqual(description.ground().object_designator.name, "milk")
        dist = np.linalg.norm(np.array(self.milk.get_position()) - np.array([-0.9, 1, 0.95]))
        self.assertTrue(dist < 0.01)


if __name__ == '__main__':
    unittest.main()
