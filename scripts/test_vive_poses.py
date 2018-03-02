#!/usr/bin/env python

import rospy
import tf
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

"""
Publish a frame 3D pose as a PoseStamped continuously.

Author: Spencer Barclay <Spencer.Barclay at wsu.edu>
"""


class PublishPoseStamped(object):
    def __init__(self, posestamped_name,
                 rate,
                 verbose=False):
        """
        Class to publish a frame as a PoseStamped.
        :param posestamped str: frame that will be published its
                pose as PoseStamped.
        :
        :param rate int: rate at which to compute and publish the pose.
        :param verbose bool: print to screen the transformations.
        """
        self.tf_lp = tf.TransformListener()
        topic_name = posestamped_name.replace('/', '')
        self.pose_pub = rospy.Publisher(topic_name + '_pose',
                                        PoseStamped, queue_size=1)
        self.posestamped_name = posestamped_name
        self.rate = rospy.Rate(rate)
        self.verbose = verbose

    def pub_poses(self, pose, from_frame):
        """
        Transform the 'pose' from frame 'from_frame'
         to frame 'to_frame'

        :param geometry_msgs/Pose pose: 3D Pose to transform.
        :param str from_frame: frame that the pose belongs to.
        :param str to_frame: to what frame transform.
        """
        ps = PoseStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame
        ps.pose = pose
        transform_ok = False
        min_time_in_between_warns = rospy.Duration(5.0)
        last_warn = rospy.Time.now() - min_time_in_between_warns
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_lp.transformPose(from_frame, ps)
                transform_ok = True
            except tf.ExtrapolationException as e:
                if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                    rospy.logwarn(
                        "Exception on transforming pose... trying again \n(" +
                        str(e) + ")")
                    last_warn = rospy.Time.now()
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_lp.getLatestCommonTime(
                    from_frame)
            except tf.LookupException as e:
                if rospy.Time.now() > (last_warn + min_time_in_between_warns):
                    rospy.logwarn(
                        "Exception on transforming pose... trying again \n(" +
                        str(e) + ")")
                    last_warn = rospy.Time.now()
                rospy.sleep(1.0)

        target_ps.header.stamp = rospy.Time.now()
        return target_ps

    def run(self):
        ps = Pose()
        ps.orientation.w = 1.0  # Quaternion must be correct
        while not rospy.is_shutdown():
            # We transform a pose with reference frame
            # self.frame_to_posestamped
            # which is 0.0, 0.0, 0.0
            # to the reference frame to get it's pose
            tfed_ps = self.pub_poses(ps,
                                    self.posestamped_name,)
            self.pose_pub.publish(tfed_ps)
            if self.verbose:
                print(tfed_ps)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('cah_posestamped')
    argv = rospy.myargv(sys.argv)
    posestamped = argv[1]
    cahps = PublishPoseStamped(posestamped,
                               10,
                               verbose=False)
    cahps.run()
