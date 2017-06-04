#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import fetch_api
import rospy

import acciobot_main.msg
import std_msgs.msg

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class CollisionHandler(object):
    def __init__(self):
        self.planning_scene = PlanningSceneInterface('base_link')

    def callback(self, box_msg):
        # self.planning_scene.clear()

        # self.planning_scene.removeCollisionObject('one_box')
        for i, collision in enumerate(box_msg.collisions):
            first_collision = collision

            self.planning_scene.addBox('one_box' + str(i),
                first_collision.scale.x, first_collision.scale.y, first_collision.scale.z,
                first_collision.pose.position.x, first_collision.pose.position.y, first_collision.pose.position.z,
            )

    def clear_callback(self, clear_msg):
        if clear_msg.data:
            print("CLEARING PLANNING SCNEE")
            self.planning_scene.clear()

# published mock point cloud is found in rosrun applications point_cloud_demo.py
# run perception point_cloud_demo w/ published mock_point_cloud
# run collision handler

def main():
    rospy.init_node('collision_handler')
    wait_for_time()

    handler = CollisionHandler()
    item_sub = rospy.Subscriber('accio_collisions', acciobot_main.msg.CollisionList, handler.callback)
    clear_sub = rospy.Subscriber('accio_clear_collisions', std_msgs.msg.Bool, handler.clear_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
