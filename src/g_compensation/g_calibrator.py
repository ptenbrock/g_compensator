#!/usr/bin/env python
import collections
import threading

import rospy
import numpy as np
import PyKDL as kdl
import tf2_py as tf2
import tf2_ros
import tf_conversions

import geometry_msgs.msg as geometry_msgs

from g_compensator import tf2_buffer, mean_wrench, wrench_msg_to_kdl, wrench_kdl_to_msg, init_transform, get_frame


class Calibrator:
    def __init__(self, wrench_topic, quality_of_pos_callback, gravity_frame='world'):
        # start filling the tf buffer
        self.tf2_listener = tf2_ros.TransformListener(tf2_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # tare
        self.wrench_buffer = collections.deque(maxlen=50)
        self.measurements = []
        self.transformations = []
        self.direction_vectors = []

        self.force_frame = None
        self.gravity_frame = gravity_frame
        self.tf_ready = False
        self.run_add_measurement = threading.Event()

        self.quality_of_pos_callback = quality_of_pos_callback

        # sync data access
        self.data_sync = threading.Lock()

        rospy.Subscriber(wrench_topic, geometry_msgs.WrenchStamped, lambda msg: self.wrench_cb(msg), queue_size=1)

    def add_measurement(self):
        while not self.tf_ready:
            rospy.sleep(0.1)
        with self.data_sync:
            self.run_add_measurement.set()

    def reset_measurements(self):
        with self.data_sync:
            self.measurements = []
            self.transformations = []
            self.direction_vectors = []

    def wrench_cb(self, msg):
        time = rospy.Time(0)  # alternative: msg.header.stamp
        self.force_frame = msg.header.frame_id

        # get transforms: gravity -> sensor
        try:
            tf_S_B = get_frame(self.force_frame, self.gravity_frame, time)
            self.tf_ready = True
        except tf2.TransformException as e:
            rospy.logerr(e)
            init_transform(self.force_frame, self.gravity_frame)
            return

        # low pass filter wrench
        self.wrench_buffer.append( wrench_msg_to_kdl(msg) )

        # add new measurement
        if self.run_add_measurement.is_set():
            self.append_measurement(tf_S_B)
            self.run_add_measurement.clear()
            rospy.loginfo("Added measurement")

        quality_of_position = self.get_quality_of_position(tf_S_B)
        self.quality_of_pos_callback(len(self.measurements), quality_of_position)

    def append_measurement(self, tf_S_B):
        self.measurements.append( mean_wrench(self.wrench_buffer) )
        self.transformations.append(tf_S_B)
        # transform z axis of current sensor frame to stationary frame
        self.direction_vectors.append(np.fromiter(tf_S_B.Inverse() * kdl.Vector(0, 0, 1), np.float, 3))

    def get_quality_of_position(self, tf_S_B):
        # determine num of measurements and quality of current position
        # current z-axis of sensor frame in stationary coordinates
        current_z_axis = (np.fromiter(tf_S_B.Inverse() * kdl.Vector(0, 0, 1), np.float, 3))
        min_diff_angle = np.Inf

        with self.data_sync:
            for former_z_axis in self.direction_vectors:
                diff_angle = np.arccos( (np.dot(current_z_axis, former_z_axis)) / (np.linalg.norm(current_z_axis) * np.linalg.norm(former_z_axis)) )
                if diff_angle < min_diff_angle:
                    min_diff_angle = diff_angle
        # bad for angle < 45deg(pi/4), good for angle > 80deg(pi/2*8/9), lin between
        quality_of_position = 1.0/(np.pi/2*8/9 - np.pi/4) * (min_diff_angle - np.pi/4)# linear part
        quality_of_position = np.min([np.max([0, min_diff_angle]), 1.0])
        return quality_of_position

    def calibrate(self):
        while not self.tf_ready:
            rospy.sleep(0.1)
        with self.data_sync:
            mass = None
            lever = None
            quality_of_fit = None
            if len(self.measurements) >= 1:

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

                if len(self.measurements) == 1:
                    (A_l_total, b_l_total, T_d_total, f_d_total) = get_values_from_measurements(self.measurements[0], self.transformations[0])

                if len(self.measurements) >= 2:
                    for i in range(1,len(self.measurements)):
                        (A_l, b_l, T_d, f_d) = get_values_from_measurements(self.measurements[i-1], self.transformations[i-1], self.measurements[i], self.transformations[i])

                        A_l_total = np.vstack((A_l_total, A_l))
                        b_l_total = np.hstack((b_l_total, b_l))

                        T_d_total = np.vstack((T_d_total, T_d))
                        f_d_total = np.hstack((f_d_total, f_d))

                (lever,_,rank_l,_) = np.linalg.lstsq(A_l_total, b_l_total)
                # print 'rank(lever): ', rank_l
                # print 'lever: ', lever

                (f_g,_,_,_) = np.linalg.lstsq(T_d_total, f_d_total)
                if np.linalg.norm(f_g) > 0:
                    quality_of_mass = np.abs(f_g[2])/np.linalg.norm(f_g)
                else:
                    quality_of_mass = 0.0
                # print 'quality(mass): ', quality_of_mass
                mass = - f_g[2] / 9.81
                # print 'mass: ', mass

                if rank_l < 3:
                    quality_of_fit = 0.0
                else:
                    quality_of_fit = quality_of_mass

                static_transformStamped = geometry_msgs.TransformStamped()

                static_transformStamped.header.stamp = rospy.Time.now()
                static_transformStamped.header.frame_id = self.force_frame
                static_transformStamped.child_frame_id = 'calibrated_com_frame'

                static_transformStamped.transform.translation.x = float(lever[0])
                static_transformStamped.transform.translation.y = float(lever[1])
                static_transformStamped.transform.translation.z = float(lever[2])

                static_transformStamped.transform.rotation.x = float(0)
                static_transformStamped.transform.rotation.y = float(0)
                static_transformStamped.transform.rotation.z = float(0)
                static_transformStamped.transform.rotation.w = float(1)

                self.broadcaster.sendTransform(static_transformStamped)

            return (mass, lever, quality_of_fit)
