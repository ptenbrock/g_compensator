#!/usr/bin/env python
import collections
import threading
#
import rospy
import numpy as np
import PyKDL as kdl
import tf2_py as tf2
import tf2_ros
import tf_conversions

import geometry_msgs.msg as geometry_msgs
from std_srvs.srv import Empty, EmptyResponse
from g_compensation.msg import CalibratorStatus
from g_compensation.srv import Calibrate, CalibrateResponse

from g_compensator import tf2_buffer, mean_wrench, wrench_msg_to_kdl, wrench_kdl_to_msg, init_transform, get_frame


if __name__ == '__main__':
    rospy.init_node('g_calibrator', anonymous=True)

    pub = rospy.Publisher('calibrator_status', CalibratorStatus, queue_size=1)

    # start filling the tf buffer
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # get parameters
    gravity_frame = rospy.get_param('~gravity_frame', 'world')
    wrench_topic = rospy.get_param('~wrench_topic', 'wrench')

    namespace = rospy.get_param('~namespace', '')
    if namespace and not namespace.endswith('/'):
        namespace = namespace + '/'

    wrench_topic = namespace + wrench_topic

    # tare
    wrench_buffer = collections.deque(maxlen=50)
    measurements = []
    transformations = []
    direction_vectors = []
    force_frame = None
    run_add_measurement = threading.Event()

    # sync services
    service_sync = threading.Lock()
    tf_ready = False

    def wrench_cb(msg):
        global force_frame, tf_ready

        time = rospy.Time(0)  # alternative: msg.header.stamp
        force_frame = msg.header.frame_id

        # get transforms: gravity -> sensor
        try:
            tf_S_B = get_frame(force_frame, gravity_frame, time)
            tf_ready = True
        except tf2.TransformException as e:
            rospy.logerr(e)
            init_transform(force_frame, gravity_frame)
            return

        # low pass filter wrench
        wrench_buffer.append( wrench_msg_to_kdl(msg) )

        # add new measurement
        if run_add_measurement.is_set():
            measurements.append( mean_wrench(wrench_buffer) )
            transformations.append(tf_S_B)
            # transform z axis of current sensor frame to stationary frame
            direction_vectors.append(np.fromiter(tf_S_B.Inverse() * kdl.Vector(0, 0, 1), np.float, 3))
            run_add_measurement.clear()
            rospy.loginfo("Added measurement")

        # publish num of measurements and quality of current position
        # current z-axis of sensor frame in stationary coordinates
        current_z_axis = (np.fromiter(tf_S_B.Inverse() * kdl.Vector(0, 0, 1), np.float, 3))
        min_diff_angle = np.Inf
        for former_z_axis in direction_vectors:
            diff_angle = np.arccos( (np.dot(current_z_axis, former_z_axis)) / (np.linalg.norm(current_z_axis) * np.linalg.norm(former_z_axis)) )
            if diff_angle < min_diff_angle:
                min_diff_angle = diff_angle
        # bad for angle < 45deg(pi/4), good for angle > 80deg(pi/2*8/9), lin between
        quality_of_position = 1.0/(np.pi/2*8/9 - np.pi/4) * (min_diff_angle - np.pi/4)# linear part
        quality_of_position = np.min([np.max([0, min_diff_angle]), 1.0])
        pub.publish(EstimatorStatus(len(measurements), quality_of_position))

    def add_measurement(req):
        while not tf_ready:
            rospy.sleep(0.1)
        with service_sync:
            rospy.sleep(0.5) # let the low pass filter do its work
            run_add_measurement.set()
            return EmptyResponse()

    def reset_measurements(req):
        while not tf_ready:
            rospy.sleep(0.1)
        with service_sync:
            measurements = []
            transformations = []
            return EmptyResponse()

    def estimate_params(req):
        while not tf_ready:
            rospy.sleep(0.1)
        with service_sync:
            m = np.NaN
            goodness_of_fit = np.NaN
            if len(measurements) >= 1:

                def get_values_from_measurements(meas1, trafo1, meas2=None, trafo2=None):
                    f_d = meas1.force
                    t_d = meas1.torque
                    if meas2 is not None:
                        f_d = f_d - meas2.force
                        t_d = t_d - meas2.torque

                    A_l = np.array([[0, f_d.z(), -f_d.y()], [-f_d.z(), 0, f_d.x()], [f_d.y(), -f_d.x(), 0]])
                    b_l = np.array([t_d.x(), t_d.y(), t_d.z()])

                    T_d = tf_conversions.toMatrix(trafo1)
                    if trafo2 is not None:
                        T_d = T_d - tf_conversions.toMatrix(trafo2)
                    f_d = np.fromiter(f_d, np.float, 3)

                    return (A_l, b_l, T_d[0:3, 0:3], f_d)

                A_l_total = np.zeros((0,3))
                b_l_total = np.zeros((0))
                T_d_total = np.zeros((0,3))
                f_d_total = np.zeros((0))

                if len(measurements) == 1:
                    (A_l_total, b_l_total, T_d_total, f_d_total) = get_values_from_measurements(measurements[0], transformations[0])

                if len(measurements) >= 2:
                    for i in range(1,len(measurements)):
                        (A_l, b_l, T_d, f_d) = get_values_from_measurements(measurements[i-1], transformations[i-1], measurements[i], transformations[i])

                        A_l_total = np.vstack((A_l_total, A_l))
                        b_l_total = np.hstack((b_l_total, b_l))

                        T_d_total = np.vstack((T_d_total, T_d))
                        f_d_total = np.hstack((f_d_total, f_d))

                (l,_,rank_l,_) = np.linalg.lstsq(A_l_total, b_l_total)
                print 'rank(l): ', rank_l
                print 'l: ', l

                (f_g,_,_,_) = np.linalg.lstsq(T_d_total, f_d_total)
                if np.linalg.norm(f_g) > 0:
                    relevance_of_m = np.abs(f_g[2])/np.linalg.norm(f_g)
                else:
                    relevance_of_m = 0.0
                print 'relevance(m): ', relevance_of_m
                m = - f_g[2] / 9.81
                print 'm: ', m

                if rank_l < 3:
                    goodness_of_fit = 0.0
                else:
                    goodness_of_fit = relevance_of_m

                static_transformStamped = geometry_msgs.TransformStamped()

                static_transformStamped.header.stamp = rospy.Time.now()
                static_transformStamped.header.frame_id = force_frame
                static_transformStamped.child_frame_id = 'estimated_com_frame'

                static_transformStamped.transform.translation.x = float(l[0])
                static_transformStamped.transform.translation.y = float(l[1])
                static_transformStamped.transform.translation.z = float(l[2])

                static_transformStamped.transform.rotation.x = float(0)
                static_transformStamped.transform.rotation.y = float(0)
                static_transformStamped.transform.rotation.z = float(0)
                static_transformStamped.transform.rotation.w = float(1)

                broadcaster.sendTransform(static_transformStamped)

            return CalibrateResponse( m, goodness_of_fit)

    # pubs'n'subs
    rospy.Subscriber(wrench_topic, geometry_msgs.WrenchStamped, wrench_cb, queue_size=1)
    rospy.Service('add_measurement', Empty, add_measurement)
    rospy.Service('reset_measurements', Empty, reset_measurements)
    rospy.Service('estimate_params', Calibrate, estimate_params)

    # run
    rospy.loginfo("Running parameter estimator")
    rospy.spin()

    rospy.loginfo("Shutting down parameter estimator")
