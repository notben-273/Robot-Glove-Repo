#!/usr/bin/env python3
"""
OBB Collision Detection Node
Checks oriented bounding boxes of finger segments against a static target box.
Publishes to /finger_collision when overlap detected.
Scalable to five fingers by adding entries to FINGER_SEGMENTS.

Publish format (std_msgs/String JSON):
  {"finger": "index", "segment": "finger_seg1", "penetration": 0.012, "in_collision": true}
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros
import numpy as np
import json


# ── Robot geometry definition ─────────────────────────────────────────────────
# Each entry: 'finger_name': [(link_name, (half_x, half_y, half_z), local_center)]
# half_* = half the box size in metres
# local_center = visual origin offset from the link frame (xyz from URDF visual origin)
#
# To scale to five fingers, add more entries here — no other code changes needed.
FINGER_SEGMENTS = {
    'index': [
        # (link_name,   half-extents,              local_centre_offset)
        ('finger_seg1', (0.025, 0.025, 0.15),     (0.0, 0.0, 0.15)),
    ],
}

# Target box definition (matches URDF target_box)
# Position is relative to base_link (from URDF joint origin)
TARGET_LINK       = 'target_box'
TARGET_HALF_EXTENTS = (0.04, 0.04, 0.04)   # half of (0.08, 0.08, 0.08)
TARGET_LOCAL_CENTRE = (0.0,  0.0,  0.0)

# Reference frame for all collision checks
WORLD_FRAME = 'base_link'
# ─────────────────────────────────────────────────────────────────────────────


def get_transform_matrix(tf_stamped):
    """Convert a TransformStamped to a 4x4 numpy matrix."""
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    # Quaternion to rotation matrix
    x, y, z, w = r.x, r.y, r.z, r.w
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def obb_vs_obb(c1, axes1, half1, c2, axes2, half2):
    """
    Separating Axis Theorem (SAT) test for two oriented bounding boxes.
    Returns (overlapping: bool, min_penetration_depth: float)
    c1, c2     : centre positions (3,) in world frame
    axes1, axes2 : rotation matrices (3x3) — columns are box axes
    half1, half2 : half-extents (3,)
    """
    d = c2 - c1
    min_pen = float('inf')

    # 15 potential separating axes: 3 from box1, 3 from box2, 9 cross products
    test_axes = []
    for i in range(3):
        test_axes.append(axes1[:, i])
        test_axes.append(axes2[:, i])
    for i in range(3):
        for j in range(3):
            cross = np.cross(axes1[:, i], axes2[:, j])
            norm  = np.linalg.norm(cross)
            if norm > 1e-6:
                test_axes.append(cross / norm)

    for axis in test_axes:
        # Project both boxes onto axis
        r1 = sum(half1[i] * abs(np.dot(axes1[:, i], axis)) for i in range(3))
        r2 = sum(half2[i] * abs(np.dot(axes2[:, i], axis)) for i in range(3))
        overlap = r1 + r2 - abs(np.dot(d, axis))
        if overlap < 0:
            return False, 0.0   # separating axis found → no collision
        min_pen = min(min_pen, overlap)

    return True, min_pen


class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher   = self.create_publisher(String, '/finger_collision', 10)

        # Timer — check collisions at 30Hz
        self.create_timer(1.0 / 30.0, self.check_collisions)

        # Track previous collision state to avoid spamming
        self.prev_collision = {finger: False for finger in FINGER_SEGMENTS}

        self.get_logger().info('Collision detector started.')

    def get_obb(self, link_name, half_extents, local_centre):
        """
        Look up a link transform and return its OBB in world frame.
        Returns (centre_world, rotation_matrix, half_extents) or None on failure.
        """
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                WORLD_FRAME, link_name, rclpy.time.Time()
            )
        except Exception:
            #self.get_logger().warn(f'TF lookup failed for {link_name}: {e}')
            return None

        T     = get_transform_matrix(tf_stamped)
        R     = T[:3, :3]
        t_world = T[:3, 3]

        # Transform local centre offset into world frame
        local_c = np.array(local_centre)
        centre_world = t_world + R @ local_c

        return centre_world, R, np.array(half_extents)

    def check_collisions(self):
        # Get target box OBB once per tick
        target_obb = self.get_obb(TARGET_LINK, TARGET_HALF_EXTENTS, TARGET_LOCAL_CENTRE)
        if target_obb is None:
            return
        t_centre, t_axes, t_half = target_obb

        for finger_name, segments in FINGER_SEGMENTS.items():
            finger_in_collision = False
            max_penetration     = 0.0
            colliding_segment   = None

            for (link_name, half_extents, local_centre) in segments:
                seg_obb = self.get_obb(link_name, half_extents, local_centre)
                if seg_obb is None:
                    continue
                s_centre, s_axes, s_half = seg_obb

                colliding, penetration = obb_vs_obb(
                    s_centre, s_axes, s_half,
                    t_centre, t_axes, t_half
                )

                if colliding and penetration > max_penetration:
                    finger_in_collision = True
                    max_penetration     = penetration
                    colliding_segment   = link_name

            # Publish on state change OR continuously while colliding
            if finger_in_collision:
                msg = String()
                msg.data = json.dumps({
                    'finger':      finger_name,
                    'segment':     colliding_segment,
                    'penetration': round(max_penetration, 4),
                    'in_collision': True
                })
                self.publisher.publish(msg)
                self.get_logger().info(
                    f'COLLISION: {finger_name} ({colliding_segment}) '
                    f'penetration={max_penetration:.4f}m'
                )

            elif self.prev_collision[finger_name]:
                # Finger just left the box — publish release event
                msg = String()
                msg.data = json.dumps({
                    'finger':      finger_name,
                    'segment':     None,
                    'penetration': 0.0,
                    'in_collision': False
                })
                self.publisher.publish(msg)
                self.get_logger().info(f'RELEASE: {finger_name} left target box')

            self.prev_collision[finger_name] = finger_in_collision


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
