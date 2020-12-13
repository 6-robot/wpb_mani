/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
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
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <wpb_mani_behaviors/Coord.h>
#include <sensor_msgs/JointState.h>

static ros::Publisher waypoint_pub;
static ros::Publisher plane_height_pub;
static std_msgs::Float64 plane_height_msg;
static ros::Publisher grab_box_pub;
static geometry_msgs::Pose grab_box_msg;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static sensor_msgs::JointState gripper_ctrl_msg;

#define STEP_READY                     0
#define STEP_GOTO_WP1           1
#define STEP_BOX_DETECT       2
#define STEP_GRAB_BOX            3
#define STEP_GOTO_WP2           4
#define STEP_STRETCH_OUT    5
#define STEP_DROP_BOX            6
#define STEP_DONE                       7
int step = STEP_READY;
int deley_count = 0;

void BoxCoordCB(const wpb_mani_behaviors::Coord::ConstPtr &msg)
{
    if(step == STEP_BOX_DETECT)
    {
        // 获取盒子检测结果
        int box_num = msg->name.size();
        ROS_INFO("[BoxCoordCB] box_num = %d",box_num);
        for(int i = 0;i<box_num;i++)
        {
            ROS_INFO("[BoxCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[i].c_str(),msg->x[i],msg->y[i],msg->z[i]);
        }
        int grab_box_index = 0;
        grab_box_msg.position.x = msg->x[grab_box_index];
        grab_box_msg.position.y = msg->y[grab_box_index];
        grab_box_msg.position.z = msg->z[grab_box_index];
        grab_box_pub.publish(grab_box_msg);
        step = STEP_GRAB_BOX;
    }
}

void NaviResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(step == STEP_GOTO_WP1)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint 1!");
            plane_height_msg.data = 0.22;      //载物台高度(单位:米)
            plane_height_pub.publish(plane_height_msg);
            step = STEP_BOX_DETECT;
        }
    }
     
    if(step == STEP_GOTO_WP2)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint 2!");
            mani_ctrl_msg.position[0] = 0;          //旋转底座(单位:弧度)
            mani_ctrl_msg.position[1] = 1.57;    //根关节(单位:弧度)
            mani_ctrl_msg.position[2] = -0.94;   //肘关节(单位:弧度)
            mani_ctrl_msg.position[3] = -0.59;   //腕关节(单位:弧度)
            mani_ctrl_pub.publish(mani_ctrl_msg);
            deley_count = 0;
            step = STEP_STRETCH_OUT;
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(step == STEP_GRAB_BOX)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[GrabResultCB] grab_box done!");
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "2";
            waypoint_pub.publish(waypoint_msg);
            step = STEP_GOTO_WP2;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_mobile_manipulation");

    ros::NodeHandle n;
    waypoint_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber navi_res_sub = n.subscribe("/waterplus/navi_result", 10, NaviResultCB);
    plane_height_pub = n.advertise<std_msgs::Float64>("/wpb_mani/plane_height", 10);
    ros::Subscriber box_result_sub = n.subscribe("/wpb_mani/boxes_3d", 10 , BoxCoordCB);
    grab_box_pub = n.advertise<geometry_msgs::Pose>("/wpb_mani/grab_box", 10);
    ros::Subscriber grab_res_sub = n.subscribe("/wpb_mani/grab_result", 10, GrabResultCB);
    mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 10);

    mani_ctrl_msg.name.resize(4);
    mani_ctrl_msg.position.resize(4);
    mani_ctrl_msg.velocity.resize(4);
    mani_ctrl_msg.name[0] = "joint1";
    mani_ctrl_msg.name[1] = "joint2";
    mani_ctrl_msg.name[2] = "joint3";
    mani_ctrl_msg.name[3] = "joint4";
    for(int i=0;i<4;i++)
    { 
        mani_ctrl_msg.position[i] = 0;
        mani_ctrl_msg.velocity[i] = 10;
    }
    
    gripper_ctrl_msg.name.resize(1);
    gripper_ctrl_msg.position.resize(1);
    gripper_ctrl_msg.velocity.resize(1);
    gripper_ctrl_msg.name[0] = "gripper";
    gripper_ctrl_msg.position[0] = 0.7;
    gripper_ctrl_msg.velocity[0] = 10;

    sleep(1);

    ros::Rate r(10);
    while(ros::ok())
    {
        if(step == STEP_READY)
        {
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "1";
            waypoint_pub.publish(waypoint_msg);
            step = STEP_GOTO_WP1;
        }

        if(step == STEP_STRETCH_OUT)
        {
            deley_count ++;
            if(deley_count > 10*5)
            {
                gripper_ctrl_msg.position[0] = 0.7;     //手爪指间距(单位:米)
                mani_ctrl_pub.publish(gripper_ctrl_msg);
                deley_count = 0;
                step = STEP_DROP_BOX;
            }
        }

        if(step == STEP_DROP_BOX)
        {
            deley_count ++;
            if(deley_count > 10*2)
            {
                mani_ctrl_msg.position[0] = 0;            //旋转底座(单位:弧度)
                mani_ctrl_msg.position[1] = -1.57;    //根关节(单位:弧度)
                mani_ctrl_msg.position[2] = 1.35;     //肘关节(单位:弧度)
                mani_ctrl_msg.position[3] = 0.24;     //腕关节(单位:弧度)
                mani_ctrl_pub.publish(mani_ctrl_msg);
                step = STEP_DONE;
            }
        }

        ros::spinOnce();
        r.sleep();
    }
    

    return 0;
}
