#!/usr/bin/env python
import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from g_compensation.msg import CalibratorStatus
from g_compensation.srv import Calibrate, CalibrateResponse
from g_calibrator import Calibrator


if __name__ == '__main__':
    rospy.init_node('g_calibrator_node', anonymous=True)

    pub = rospy.Publisher('calibrator_status', CalibratorStatus, queue_size=1)

    def quality_of_pos_callback(no_measurements, quality_of_position):
        pub.publish(CalibratorStatus(num_measurements = no_measurements, position_quality = quality_of_position))

    # get parameters
    gravity_frame = rospy.get_param('~gravity_frame', 'world')
    wrench_topic = rospy.get_param('~wrench_topic', 'wrench')

    namespace = rospy.get_param('~namespace', '')
    if namespace and not namespace.endswith('/'):
        namespace = namespace + '/'

    wrench_topic = namespace + wrench_topic

    calibrator = Calibrator(wrench_topic, quality_of_pos_callback, gravity_frame=gravity_frame)

    def add_measurement(req):
        while not calibrator.tf_ready:
            rospy.sleep(0.1)
        rospy.sleep(0.5) # let the low pass filter do its work
        calibrator.add_measurement()
        return EmptyResponse()

    def reset_measurements(req):
        while not calibrator.tf_ready:
            rospy.sleep(0.1)
        calibrator.reset_measurements()
        return EmptyResponse()

    def calibrate(req):
        (mass, lever, quality_of_fit) = calibrator.calibrate()
        if mass is None:
            mass = np.NaN
        if quality_of_fit is None:
            quality_of_fit = np.NaN
        if lever is None:
            lever = np.array([np.NaN, np.NaN, np.NaN])
        return CalibrateResponse( mass=mass, lever=lever, quality_of_fit=quality_of_fit)


    # pubs'n'subs
    rospy.Service('add_measurement', Empty, add_measurement)
    rospy.Service('reset_measurements', Empty, reset_measurements)
    rospy.Service('calibrate', Calibrate, calibrate)

    # run
    rospy.loginfo("Running g_calibrator")
    rospy.spin()

    rospy.loginfo("Shutting down g_calibrator")
