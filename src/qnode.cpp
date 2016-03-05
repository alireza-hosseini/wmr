/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date March 2016
 **/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include "../include/wmr_ros/qnode.hpp"
#include "WMR/wmrsettings.h"
#include "WMR/wmrrobotconnector.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"

namespace wmr_ros {

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {

    ROS_INFO("Initializing WMR robot controller...");
    ros::init(init_argc,init_argv,"wmr_ros");
    if ( ! ros::master::check() ) {
        return false;
    }
    wmrRobot = new WmrRobot;

    ros::start();
    nh_ = new ros::NodeHandle;

    std::string robot_ip = nh_->param<std::string>("robot_ip","192.168.1.1");
    int command_port = nh_->param<int>("command_port",6000);
    int status_port  = nh_->param<int>("robot_status_port",5003);
    int sensors_port = nh_->param<int>("sensors_port",5002);

    tf_prefix_ = tf::getPrefixParam(*nh_);
    frame_id_odom_ = tf::resolve(tf_prefix_, "odom");
    frame_id_base_link_ = tf::resolve(tf_prefix_, "base_link");

    WmrSettings wmrSettings(QHostAddress(QString(robot_ip.c_str())),command_port,status_port,sensors_port);
    WmrRobotConnector wmrRobotConnector(&wmrSettings,wmrRobot);
    ROS_INFO_STREAM("Connecting to robot:\nip address: "<<robot_ip<<"\n"<<
                    "command port: "<<command_port<<"\n"<<
                    "status port: "<<status_port<<"\n"<<
                    "sensors port: "<<sensors_port);
    wmrRobotConnector.connectRobot();

    cmd_vel_sub_ = nh_->subscribe("/cmd_vel",1000,(boost::function<void(const geometry_msgs::Twist::ConstPtr&)>)
                                  boost::bind(&QNode::cmdVelCallback, this,_1));
    odometery_pub_ = nh_->advertise<nav_msgs::Odometry>("/odom",50);
    ir_proximity_sensors_pub_ = nh_->advertise<sensor_msgs::PointCloud>("ir",50);

    start();
    return true;
}

void QNode::run() {

    ros::Rate loop_rate(WMR_FREQUENCY);
    while ( ros::ok() ) {
        publishOdometry();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void QNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
    cmd_vel_ = *cmd_vel_msg.get();
}

void QNode::publishOdometry() {

    odom_trans_.header.stamp = ros::Time::now();
    odom_trans_.header.frame_id = frame_id_odom_;
    odom_trans_.child_frame_id = frame_id_base_link_;

    WmrPose pos = wmrRobot->getPose();

    odom_trans_.transform.translation.x = pos.getX();
    odom_trans_.transform.translation.y = pos.getY();
    odom_trans_.transform.translation.z = 0.0;
    odom_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTheta());

    odom_broadcaster_.sendTransform(odom_trans_);

    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.header.frame_id = frame_id_odom_;
    odom_msg_.child_frame_id = frame_id_base_link_;

    odom_msg_.twist.twist.linear.x = wmrRobot->getLinearVel();
    odom_msg_.twist.twist.linear.y = 0;
    odom_msg_.twist.twist.linear.z = 0;
    odom_msg_.twist.twist.angular.x = 0;
    odom_msg_.twist.twist.angular.y = 0;
    odom_msg_.twist.twist.angular.z = wmrRobot->getRotationalVel();
    tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTheta()), tf::Vector3(pos.getX(),
       pos.getY(), 0)), odom_msg_.pose.pose);

    odometery_pub_.publish(odom_msg_);

}

void QNode::publishIRProximity(){
    for(int i=0; i<IR_SENSORS_NUM; i++) {
        //todo
    }
}

}  // namespace wmr_ros
