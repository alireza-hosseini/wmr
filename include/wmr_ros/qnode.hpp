/*
    SOSCO, Wheeled Mobile Robot(WMR)
    Copyright (C) 2016  Alireza Hosseini

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file /include/wmr_ros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date March 2016
 **/

#ifndef wmr_ros_QNODE_HPP_
#define wmr_ros_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include "WMR/wmrrobot.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "tf/transform_broadcaster.h"

namespace wmr_ros {

#define WMR_FREQUENCY 50

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg);

    void publishOdometry();
    void publishIRProximity();

private:
	int init_argc;
	char** init_argv;

    WmrRobot *wmrRobot;

    geometry_msgs::Twist cmd_vel_;

    ros::Subscriber cmd_vel_sub_;

    ros::Publisher odometery_pub_;
    ros::Publisher ir_proximity_sensors_pub_;

    ros::NodeHandle *nh_;

    tf::TransformBroadcaster odom_broadcaster_;

    geometry_msgs::TransformStamped odom_trans_;
    nav_msgs::Odometry odom_msg_;

    std::string frame_id_odom_;
    std::string frame_id_base_link_;
    std::string tf_prefix_;
};

}  // namespace wmr_ros

#endif /* wmr_ros_QNODE_HPP_ */
