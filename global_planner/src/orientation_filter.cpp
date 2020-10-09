/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, David V. Lu!!
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of David V. Lu nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David V. Lu!!
 *********************************************************************/
#include <global_planner/orientation_filter.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <cmath>

namespace global_planner
{

    void set_angle(geometry_msgs::PoseStamped *pose, double angle)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        tf2::convert(q, pose->pose.orientation);
    }

    void OrientationFilter::processPath(const geometry_msgs::PoseStamped &start,
                                        std::vector<geometry_msgs::PoseStamped> &path)
    {
        int n = path.size();
        if (n == 0)
            return;
        switch (omode_)
        {
        case FORWARD:
        {
            for (int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }
            break;
        }
        case BACKWARD:
        {
            for (int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI));
            }
            break;
        }
        case LEFTWARD:
        {
            for (int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) - M_PI_2));
            }
            break;
        }
        case RIGHTWARD:
        {
            for (int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI_2));
            }
            break;
        }
        case INTERPOLATE:
        {
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, n - 1);
            break;
        }
        case ADAPTIVE:
        {
            path[0].pose.orientation = start.pose.orientation;

            int rotation_point__ = rotation_point_;
            if (n <= 2)
            {
                ROS_WARN("Forward mode, path too short");

                for (int i = 0; i < n - 1; i++)
                {
                    setAngleBasedOnPositionDerivative(path, i);
                }
                break;
            }
            else if (rotation_point_ >= n)
            {
                rotation_point__ = n - 1;
                ROS_WARN("Shrinking point to: %d", rotation_point__);
            }

            bool direction = adaptive(path, 0, rotation_point__);
            if (direction)
            {
                ROS_INFO("Forward mode");

                for (int i = 0; i < n - 1; i++)
                {
                    setAngleBasedOnPositionDerivative(path, i);
                }
            }
            else
            {
                ROS_INFO("Backward mode");
                for (int i = 0; i < n - 1; i++)
                {
                    setAngleBasedOnPositionDerivative(path, i);
                    set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI));
                }
            }

            break;
        }
        case FORWARDTHENINTERPOLATE:
        {
            for (int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }

            int i = n - 3;
            const double last = tf2::getYaw(path[i].pose.orientation);
            while (i > 0)
            {
                const double new_angle = tf2::getYaw(path[i - 1].pose.orientation);
                double diff = fabs(angles::shortest_angular_distance(new_angle, last));
                if (diff > 0.35)
                    break;
                else
                    i--;
            }

            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, i, n - 1);
            break;
        }
        }
    }

    void OrientationFilter::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped> &path, int index)
    {
        int index0 = std::max(0, index - window_size_);
        int index1 = std::min((int)path.size() - 1, index + window_size_);

        double x0 = path[index0].pose.position.x,
               y0 = path[index0].pose.position.y,
               x1 = path[index1].pose.position.x,
               y1 = path[index1].pose.position.y;

        double angle = atan2(y1 - y0, x1 - x0);
        set_angle(&path[index], angle);
    }

    void OrientationFilter::interpolate(std::vector<geometry_msgs::PoseStamped> &path,
                                        int start_index, int end_index)
    {
        const double start_yaw = tf2::getYaw(path[start_index].pose.orientation),
                     end_yaw = tf2::getYaw(path[end_index].pose.orientation);
        double diff = angles::shortest_angular_distance(start_yaw, end_yaw);
        double increment = diff / (end_index - start_index);
        for (int i = start_index; i <= end_index; i++)
        {
            double angle = start_yaw + increment * i;
            set_angle(&path[i], angle);
        }
    }

    bool OrientationFilter::adaptive(std::vector<geometry_msgs::PoseStamped> &path,
                                     int start_index, int rotation_index)
    {
        double diff{};
        ROS_INFO("Using adaptive filter");
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time::now(),
                                                        ros::Duration(0.20));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        const double robot_x = transformStamped.transform.translation.x;
        const double robot_y = transformStamped.transform.translation.y;
        double robot_yaw = tf2::getYaw(transformStamped.transform.rotation);
    
        const double end_x = path[rotation_index].pose.position.x;
        const double end_y = path[rotation_index].pose.position.y;
    
        double delta_x = end_x - robot_x;
        double delta_y = end_y - robot_y;
        
        double end_angle = atan2(delta_y, delta_x);

        if (robot_yaw < 3.141592653589793238463)
        {
            robot_yaw = robot_yaw;
        }
        else
        {
            robot_yaw = -2 * 3.141592653589793238463 + robot_yaw;
        }

        try
        {
            diff = angles::shortest_angular_distance(robot_yaw, end_angle);
        }
        catch (...)
        {
            ROS_WARN("Error in calculating angle using forward mode");
            return true;
        }

        if (std::abs(diff) < 3.141592653589793238463 / 2) // rad
        {
            return true; //forward
        }
        else
        {
            return false; //backward
        }
    }

}; // namespace global_planner
