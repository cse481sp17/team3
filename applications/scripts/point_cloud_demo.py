#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import perception
import rospy
import math


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def copy_point_cloud(cloud, dx, dy):
    values = point_cloud2.read_points(cloud)
    points = []
    for v in values:
        new_point = (v[0] if math.isnan(v[0]) else v[0] + dx, v[1] if math.isnan(v[1]) else v[1] + dy, v[2], v[3])
        #if not math.isnan(v[0]):
        #    print(v, new_point)
        #    break
        points.append(v)
    new_cloud = point_cloud2.create_cloud(cloud.header, cloud.fields, points)
    return new_cloud

def combine_clouds(clouds):
    points = []
    for cloud in clouds:
        values = point_cloud2.read_points(cloud)
        for v in values:
            points.append(v)
    new_cloud = point_cloud2.create_cloud(clouds[0].header, clouds[0].fields, points)
    return new_cloud

def main():
    rospy.init_node('publish_saved_cloud')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print 'Publishes a saved point cloud to a latched topic.'
        print 'Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag [optional frame]'
        return
    path = argv[1]
    camera = perception.MockCamera()
    cloud = camera.read_cloud(path)

    if cloud is None:
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    #new_cloud = copy_point_cloud(cloud, 10, 10)
    #new_cloud2 = copy_point_cloud(cloud, 4, 2)
    #final = combine_clouds([new_cloud])#, new_cloud2])

    final = cloud
    topic = 'mock_point_cloud'
    pub = rospy.Publisher(topic, PointCloud2, queue_size=1)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        final.header.stamp = rospy.Time.now()
        if len(argv) > 2:
            final.header.frame_id = argv[2]
        pub.publish(final)
        rate.sleep()


if __name__ == '__main__':
    main()
