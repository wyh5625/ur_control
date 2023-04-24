import math
import numpy as np

import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

gripper_length = 0.089
gripper_length_collision = gripper_length - 0.005

conveyer_board_offset_x = 1.07
conveyer_board_offset_y = 0.2


def add_conveyer(self, timeout=4):
    conveyer_length = 4
    conveyer_width = 0.486
    conveyer_z = 0.8

    box_size = [conveyer_length + 0.2, conveyer_width + 0.02, conveyer_z]
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"
    box_pose.pose.position.x = conveyer_board_offset_x - conveyer_length / 2
    box_pose.pose.position.y = conveyer_board_offset_y - conveyer_width / 2
    box_pose.pose.position.z = 0 - box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]
    box_name = "conveyer"

    self.scene.add_box(box_name, box_pose, size=tuple(box_size))
    ret_conveyer = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    return ret_conveyer


def add_camerabox(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"

    box_name = "camera_box"

    box_size = (0.3, 0.1, 0.5)
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
    box_pose.pose.position.x = -0.15
    box_pose.pose.position.y = 0.20 + box_size[1] / 2
    box_pose.pose.position.z = box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    self.scene.add_box(box_name, box_pose, size=box_size)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"
    return self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)

def add_gripper(self, timeout=2):
    # return True
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"

    box_name = "gripper"

    box_size = (0.06,0.06, gripper_length_collision)
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    # self.scene.add_cylinder(box_name, box_pose, height=box_size[2], radius=box_size[0])
    self.scene.add_box(box_name, box_pose, size=box_size)
    ret1 = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    if ret1:
        eef_link = 'tool0'
        self.scene.attach_box(eef_link, box_name, touch_links=['wrist_3_link'])
        return self.wait_for_state_update(box_name, object_is_attached=True, object_is_known=False, timeout=timeout)


def add_attach_objectbox(self, timeout=2):
    # return True
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"

    box_name = "object_box"

    box_size = (0.15, 0.15, 0.01)
    # quat = quaternion_from_euler(*np.radians([0, 90, 90])) #obj2ee
    quat = quaternion_from_euler(*np.radians([0, 0, 0]))
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = gripper_length_collision + box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    self.scene.add_box(box_name, box_pose, size=box_size)

    ret1 = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    if ret1:
        eef_link = 'tool0'
        self.scene.attach_box(eef_link, box_name, touch_links=[eef_link, 'wrist_3_link'])
        return self.wait_for_state_update(box_name, object_is_attached=True, object_is_known=False, timeout=timeout)


def remove_object_box(self, timeout=2):
    # return True
    eef_link = 'tool0'
    box_name = "object_box"
    self.scene.remove_attached_object(eef_link, name=box_name)
    ret1 = self.wait_for_state_update(box_name, object_is_known=True, object_is_attached=False, timeout=timeout)
    self.scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_name, object_is_attached=False, object_is_known=False, timeout=timeout)


def setup_scene(self):
    add_conveyer(self)
    add_camerabox(self)
    add_gripper(self)