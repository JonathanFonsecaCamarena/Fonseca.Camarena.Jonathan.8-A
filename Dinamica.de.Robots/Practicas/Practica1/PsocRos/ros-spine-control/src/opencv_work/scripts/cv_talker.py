#!/usr/bin/python2.7

# NOTE: on Ubuntu 18.04, specifying /usr/bin/env python uses python 3,
# while specifying /usr/bin/python2.7 uses python 2.7. We need 2.7.

# Simple talker demo that publishes numpy_msg(Floats) messages
# to the 'cv_data' topic

import rospy
import roslib
from opencv_object_tracker import tracker_init, tracker_main, auto_homography
import cv2
import numpy as np
from numpy.linalg import inv, norm
from opencv_work.msg import SpineState

roslib.load_manifest('opencv_work')


def talker():

    # set publish node to 'cv_data' and initialize publisher node as 'cv_talker'
    rospy.init_node('cv_talker', anonymous=False)
    # pub = rospy.Publisher('cv_data', numpy_msg(Floats), queue_size=10)
    pub = rospy.Publisher('cv_data', SpineState, queue_size=10)
    rate = rospy.Rate(100)  # 10hz
    d1 = 0.82  # distance in cm between dowel pin 1 (leftmost) and the local vertebra origin, as of 2019-05-15
    originpix = np.empty([2, 1]).astype(int)  # initialize array for pixel coordinates of origin

    while not rospy.is_shutdown():

        # wrap everything around a keyboard interrupt catcher
        try:
            # run initalization
            (detector, vs, Q, uvec, K, D, key) = tracker_init()

            # while still tracking
            while not key == ord("q"):

                # run tracker
                (blob_detection_data, key) = tracker_main(detector, K, D, vs, originpix)
                pix_com_data = blob_detection_data['pix_com']

                # calculate true COM points using homography matrix
                pt1 = np.array(pix_com_data[0, :])  # location of dowel pin 1 in pixel frame
                pt2 = np.array(pix_com_data[1, :])  # location of dowel pin 2 in pixel frame
                pt1h = np.r_[np.transpose([pt1]), np.array([[1]])]  # homogeneous coordinates
                pt2h = np.r_[np.transpose([pt2]), np.array([[1]])]
                pt1true = np.dot(Q, pt1h)[0:2, :] / np.dot(Q, pt1h)[2, :]   # location of dowel pin 1 in global frame (must divide by homography scaling factor)
                pt2true = np.dot(Q, pt2h)[0:2, :] / np.dot(Q, pt2h)[2, :]  # location of dowel pin 2 in global frame (divide by homography scaling factor)
                mag = np.linalg.norm(pt2true - pt1true)  # magintude of vector between pins
                duvec = (pt2true - pt1true) / mag if mag != 0 else np.full(2, np.nan)  # unit vector between dowel pin locations
                origin = pt1true - np.transpose([duvec * d1]) if not np.isnan(duvec).any() else np.array([[[0], [0]]])  # global vertebra origin
                # angle between dowel pin vector and world frame x-unit vector (caluclated in initialization)
                angle = np.math.atan2(np.linalg.det([np.stack((uvec.reshape(1, 2)[0], duvec.reshape(1, 2)[0]))]),
                                      np.dot(uvec.reshape(1, 2), duvec))  # angle of vertebra rotation

                # re-calculate origin in pixel coordinates and visualize on frame
                originh = np.concatenate((origin[0], np.array([[1]])), axis=0)
                originpix = (np.dot(inv(Q), originh) / np.dot(inv(Q), originh)[2, :])[0:2, :].astype(int)

                # nominal_origin = [32.84, 15.24]
                # publish data
                message = SpineState(rotation=np.degrees(angle), comx=float(origin[0, 0][0]), comy=float(origin[0, 1][0]))
                pub.publish(message)
                rate.sleep()

            vs.release()
            print('[END OF TRACKING]')

            # close all windows and leave loop
            cv2.destroyAllWindows()
            break

        except KeyboardInterrupt:
            print("Exiting.")
            break


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
