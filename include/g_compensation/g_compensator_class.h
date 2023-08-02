#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <tf2_ros/transform_listener.h>

#include <kdl/frames.hpp>

#include <atomic>

class GCompensator
{
public:
    explicit GCompensator(ros::NodeHandle nh);

private:
    void getStaticParameters();

    bool getTransform(const std::string &target, const std::string &source, KDL::Frame &transform);

    void wrenchCB(geometry_msgs::WrenchStampedConstPtr msg);

    KDL::Wrench calcBufferMean();

    bool tareCB(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

private:
    ros::NodeHandle nh_;
    double mass_;
    std::string gravity_frame_, com_frame_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    tf2_ros::Buffer buf_;
    tf2_ros::TransformListener tf_listener_;
    KDL::Wrench gravity_;
    KDL::Wrench tare_offset_;
    std::vector<KDL::Wrench> wrench_buffer_;
    KDL::Frame tf_gravity_, tf_com_;
    std::atomic_bool run_tare_ = {false};
    ros::ServiceServer srv_tare_;

    int buffer_size_;
    int last_updated_row_;

    KDL::Wrench gravity_at_com_;
    KDL::Wrench gravity_at_sensor_;
    KDL::Wrench message_wrench_;
    KDL::Wrench compensated_;
    geometry_msgs::WrenchStamped compensated_msg_;
};