import logging
from numbers import Number

#import roslibpy
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped, Transform, Vector3, Point
from typing import List, Tuple, Union, Optional



def pose_stamped2tuple(pose_stamped: PoseStamped) -> Tuple[List[float], List[float]]:
    if type(pose_stamped) is PoseStamped:
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        return tuple(((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))


def ensure_pose(pose: Pose) -> Union[Pose, None]:
    if type(pose) is Pose:
        return pose
    elif (type(pose) is list or type(pose) is tuple) and is_list_pose(pose):
        pose = list2pose(pose)
        return pose
    elif (type(pose) is list or type(pose) is tuple) and is_list_position(pose):
        point = list2point(pose)
        return Pose(point, Quaternion(0, 0, 0, 1))
    else:
        logging.error("(helper) Cannot convert pose since it is no Pose object or valid list pose.")
        return None


def tf2tfstamped(source_frame: str,
                 target_frame: str,
                 tf: Transform,
                 time: Optional[rospy.rostime.Time] = None) -> TransformStamped:
    tf_time = time if time else rospy.Time.now()
    header = Header(0, tf_time, source_frame)
    return TransformStamped(header, target_frame, tf)


def pose2tfstamped(source_frame: str,
                   target_frame: str,
                   pose: Pose,
                   time: Optional[rospy.rostime.Time] = None) -> TransformStamped:
    tf = pose2tf(pose)
    if tf:
        return tf2tfstamped(source_frame, target_frame, tf, time)


def list2tfstamped(source_frame: str,
                   target_frame: str,
                   pose_list: Tuple[List[float], List[float]],
                   time: Optional[rospy.rostime.Time] = None) -> TransformStamped:
    tf = list2tf(pose_list)
    if tf:
        return tf2tfstamped(source_frame, target_frame, tf, time)


def pose2tf(pose: Pose) -> Transform:
    if pose and pose.position and pose.orientation:
        p = pose.position
        return Transform(Vector3(p.x, p.y, p.z), pose.orientation)


def list2tf(pose_list: Tuple[List[float], List[float]]) -> Transform:
    p, q = list2vector3_and_quaternion(pose_list)
    if p and q:
        return Transform(p, q)


def list2pose(pose_list: Tuple[List[float], List[float]]) -> Pose:
    p, q = list2point_and_quaternion(pose_list)
    if p and q:
        return Pose(p, q)


def list2point(pos_list: List[float]) -> Point:
    if len(pos_list) == 3:
        return Point(pos_list[0], pos_list[1], pos_list[2])


def list2vector3_and_quaternion(pose_list: Tuple[List[float], List[float]]) -> Tuple[Vector3, Quaternion]:
    if len(pose_list) == 2 and len(pose_list[0]) == 3 and len(pose_list[1]) == 4:
        pos = pose_list[0]
        orient = pose_list[1]
        vector = Vector3(pos[0], pos[1], pos[2])
        quaternion = Quaternion(orient[0], orient[1], orient[2], orient[3])
        return vector, quaternion


def list2point_and_quaternion(pose_list: Tuple[List[float], List[float]]) -> Tuple[Point, Quaternion]:
    if len(pose_list) == 2 and len(pose_list[0]) == 3 and len(pose_list[1]) == 4:
        pos = pose_list[0]
        orient = pose_list[1]
        point = Point(pos[0], pos[1], pos[2])
        quaternion = Quaternion(orient[0], orient[1], orient[2], orient[3])
        return point, quaternion


def is_list_pose(list_pose: Tuple[List[float], List[float]]) -> bool:
    if len(list_pose) == 2 and is_list_position(list_pose[0]):
        return len(list_pose[1]) == 4 and all(isinstance(x, Number) for x in list_pose[1])


def is_list_position(list_pos: List[float]) -> bool:
    return len(list_pos) == 3 and all(isinstance(x, Number) for x in list_pos)
