#include "g_compensation/g_compensator_class.h"

#include <kdl_conversions/kdl_msg.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

GCompensator::GCompensator(ros::NodeHandle nh) : nh_(nh), tf_listener_(buf_), last_updated_row_(0)
{
    getStaticParameters();

    pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("wrench_compensated", 1);

    // rospy.Subscriber('wrench', geometry_msgs.WrenchStamped, wrench_cb,
    //                  queue_size = 1)
    //     rospy.Service('tare', Empty, tare_cb)

    sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("wrench", 1, boost::bind(&GCompensator::wrenchCB, this, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    srv_tare_ = nh_.advertiseService("tare", &GCompensator::tareCB, this);

    gravity_ = KDL::Wrench(KDL::Vector(0, 0, -9.81 * mass_), KDL::Vector(0, 0, 0));
    tare_offset_ = KDL::Wrench(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0));

    wrench_buffer_.resize(buffer_size_);

    sleep(1);
    while (!getTransform(com_frame_, gravity_frame_, tf_gravity_))
    {
        ROS_WARN_STREAM_ONCE("Waiting for transform " << gravity_frame_ << " -> " << com_frame_ << "..");
        sleep(2);
    }
    ROS_INFO_STREAM("Ready to compensate incoming wrench on topic " << sub_.getTopic());
}

void GCompensator::getStaticParameters()
{
    ros::NodeHandle n_static_params("~");
    std::size_t error = 0;
    error += !rosparam_shortcuts::get("g_compensator", n_static_params, "mass", mass_);
    error += !rosparam_shortcuts::get("g_compensator", n_static_params, "com_frame", com_frame_);
    rosparam_shortcuts::shutdownIfError("g_compensator", error);

    if (!rosparam_shortcuts::get("g_compensator", n_static_params, "gravity_frame", gravity_frame_))
    {
        ROS_WARN("Continuing with default gravity_frame 'world'.");
        gravity_frame_ = "world"; // default value
    };
    if (!rosparam_shortcuts::get("g_compensator", n_static_params, "buffer_size", buffer_size_))
    {
        buffer_size_ = 50; //default value
        ROS_WARN("Continuing with default buffer_size '50'.");
    };
}

bool GCompensator::getTransform(const std::string &target, const std::string &source, KDL::Frame &transform)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = buf_.lookupTransform(target, source, ros::Time(0));
        tf::transformMsgToKDL(transformStamped.transform, transform);
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        return false;
    }
}

void GCompensator::wrenchCB(geometry_msgs::WrenchStampedConstPtr msg)
{
    // global tare_offset

    //     time = rospy.Time(0)  # alternative: msg.header.stamp
    //     force_frame = msg.header.frame_id

    //     # get transforms: gravity -> com -> sensor
    //     try:
    //         tf_gravity = get_frame(com_frame, gravity_frame, time)
    //         tf_com = get_frame(force_frame, com_frame, time)
    //     except tf2.TransformException as e:
    //         rospy.logerr(e)
    //         init_transform(com_frame, gravity_frame)
    //         init_transform(force_frame, com_frame)
    //         return

    if (getTransform(com_frame_, gravity_frame_, tf_gravity_) && getTransform(msg->header.frame_id, com_frame_, tf_com_))
    {
        //compute gravity at com : change coordinate system(rotate)
        //     gravity_at_com = tf_gravity.M * gravity
        gravity_at_com_ = tf_gravity_.M * gravity_;

        //     # compute gravity at sensor: screw theory transform
        //     # this covers coordinate change and translation-lever
        //     gravity_at_sensor = tf_com * gravity_at_com
        gravity_at_sensor_ = tf_com_ * gravity_at_com_;

        //     # compensate
        tf::wrenchMsgToKDL(msg->wrench, message_wrench_);
        compensated_ = message_wrench_ - gravity_at_sensor_;

        if (last_updated_row_ >= buffer_size_)
        {
            last_updated_row_ = 0;
        }

        wrench_buffer_[last_updated_row_] = compensated_;
        last_updated_row_++;

        //     # Tare
        //     if run_tare.is_set():
        //         tare_offset = mean_wrench(wrench_buffer)
        //         run_tare.clear()
        //         rospy.loginfo("Tared sensor")
        if (run_tare_)
        {
            tare_offset_ = calcBufferMean();
            run_tare_ = false;
        }

        //     compensated = compensated - tare_offset
        compensated_ = compensated_ - tare_offset_;

        //     # publish
        //     compensated_msg = geometry_msgs.WrenchStamped(
        //         header=msg.header, wrench=wrench_kdl_to_msg(compensated))
        //     pub.publish(compensated_msg)

        compensated_msg_.header = msg->header;
        tf::wrenchKDLToMsg(compensated_, compensated_msg_.wrench);
        pub_.publish(compensated_msg_);
    }
    else
    {
        ROS_ERROR_STREAM("Could not get transform " << gravity_frame_ << " -> " << com_frame_ << " or " << msg->header.frame_id << " -> " << com_frame_ << "..");
        sleep(1);
    }
}

KDL::Wrench GCompensator::calcBufferMean()
{
    KDL::Wrench mean_sum = KDL::Wrench::Zero();
    size_t size = wrench_buffer_.size();
    for (auto wrench : wrench_buffer_)
    {
        mean_sum += wrench;
    }
    return mean_sum / size;
}

bool GCompensator::tareCB(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    run_tare_ = true;
    return true;
}