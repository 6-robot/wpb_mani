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
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <wpb_mani_behaviors/Coord.h>
#include <math.h>

// 抓取参数调节（单位：米）
static float grab_y_offset = 0.0f;                  //机器人对准物品，横向位移偏移量
static float grab_z_offset = -0.01f;                  //手臂抬起高度的补偿偏移量
static float grab_forward_offset = 0.0f;    //手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float grab_gripper_value = 0.05;    //抓取物品时，手爪闭合后的手指间距

static float target_dist = 0.6;
static float target_x_k = 0.5; 
static float target_y_k = 0.5; 

static ros::Publisher box_track_pub;
static geometry_msgs::Pose box_track_msg;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static ros::Publisher grab_result_pub;
static std_msgs::String grab_result_msg;

static float box_track_x = 0.0;
static float box_track_y = 0.0;
static float box_track_z = 0.0;
static float reachout_x_offset = 0.0;

#define STEP_READY                     0
#define STEP_TAKE_AIM              1
#define STEP_REACH_OUT         2
#define STEP_FORWARD              3
#define STEP_GRAB_BOX            4
#define STEP_TAKE_OVER          5
#define STEP_BACKWARD           6
#define STEP_DONE                       7
int step = STEP_READY;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void ReachOut(float inGrabZ)
{
    float mani_base_height = 0.25;    // link3高度
    float joint3_lenght = 0.128;                           // 第一节臂长度
    float joint4_lenght = 0.124 + 0.024;            // 第二节臂长度
    sensor_msgs::JointState mani_ctrl_msg;
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
        mani_ctrl_msg.velocity[i] = 0.35;
    }
    mani_ctrl_msg.velocity[1] = 1.05;
    mani_ctrl_msg.velocity[3] = 0.5;
    // 下限
    float min_grab_z = mani_base_height - joint4_lenght;
    if(inGrabZ < min_grab_z)
    {
        inGrabZ = min_grab_z;
    }
    float max_grab_z = mani_base_height + joint3_lenght;
     if(inGrabZ > max_grab_z)
    {
        inGrabZ = max_grab_z;
    }
    // 计算第一节臂的俯仰角
    float angle = 0;
    float z_offset = inGrabZ - mani_base_height;

    // 解方程 joint3_lenght*cos(angle) - joint4_lenght*sin(angle) = z_offset
    float tmp = sqrtf(joint3_lenght * joint3_lenght + joint4_lenght * joint4_lenght);
    float b = asin(joint3_lenght/tmp);
    angle = b - asin(z_offset/tmp);

    reachout_x_offset = joint3_lenght*sin(angle) + joint4_lenght*cos(angle) - joint3_lenght - 0.02;

    mani_ctrl_msg.position[1] = angle;
    mani_ctrl_msg.position[3] = -angle;
    
    mani_ctrl_pub.publish(mani_ctrl_msg);
}

void ManiGripper(float inGripperVal)
{
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(1);
    mani_ctrl_msg.position.resize(1);
    mani_ctrl_msg.velocity.resize(1);
    mani_ctrl_msg.name[0] = "gripper";
    mani_ctrl_msg.position[0] = inGripperVal;
    mani_ctrl_msg.velocity[0] = 1.0;
    mani_ctrl_pub.publish(mani_ctrl_msg);
}

void BoxCoordCB(const wpb_mani_behaviors::Coord::ConstPtr &msg)
{
    if(step == STEP_TAKE_AIM)
    {
        // 获取盒子检测结果
        box_track_x = msg->x[0];
        box_track_y = msg->y[0];
        box_track_z = msg->z[0];
        ROS_INFO("[BoxCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[0].c_str(),msg->x[0],msg->y[0],msg->z[0]);
    }
}

void GrabBoxCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 目标物品的坐标
    box_track_x = msg->position.x;
    box_track_y = msg->position.y;
    box_track_z = msg->position.z;
    box_track_msg.position.x = box_track_x;
    box_track_msg.position.y = box_track_y;
    box_track_msg.position.z = box_track_z;
    box_track_pub.publish(box_track_msg);
    ROS_WARN("[GrabBoxCallback] x = %.2f y= %.2f ,z= %.2f " ,box_track_x, box_track_y, box_track_z);
    step = STEP_TAKE_AIM;
}

float VelFixed(float inValue)
{
    float retValue = inValue;
    if(retValue > 0.5)
    {
        retValue = 0.5;
    }
    if(retValue < -0.5)
    {
        retValue = -0.5;
    }
    return retValue;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_grab_box");

    ros::NodeHandle nh;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpb_mani/behaviors", 10);
    box_track_pub =  nh.advertise<geometry_msgs::Pose>("/wpb_mani/box_track", 10);
    ros::Subscriber box_result_sub = nh.subscribe("/wpb_mani/boxes_3d", 10 , BoxCoordCB);
    ros::Subscriber track_sub = nh.subscribe("/wpb_mani/grab_box", 10, GrabBoxCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);
    grab_result_pub = nh.advertise<std_msgs::String>("/wpb_mani/grab_result", 30);

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/target_y_k", target_y_k);
    //ROS_WARN("[grab_box] target_y_k = %f",target_y_k);

    tf::TransformListener tf_listener; 
    tf::StampedTransform ar_transform;

    ros::Time time = ros::Time(0);
    float grab_z = 0.15;
    float gripper = 0;
    int nCount = 0;
    ros::Rate r(30);
    while(nh.ok())
    {
        // 对准目标盒子
        if(step == STEP_TAKE_AIM)
        {
            float diff_x = box_track_x - target_dist;
            float diff_y = box_track_y - grab_y_offset;

            float vx = VelFixed(diff_x * target_x_k);
            float vy = VelFixed(diff_y * target_y_k); 

            VelCmd(vx,vy,0);
            ROS_WARN("diff_y = %.3f",diff_y);
            if(fabs(diff_x) < 0.01 && fabs(diff_y) < 0.005)
            {
                VelCmd(0,0,0);
                std_msgs::String behavior_msg;
                behavior_msg.data = "box_track stop";
                behaviors_pub.publish(behavior_msg);
                nCount = 0;
                step = STEP_REACH_OUT;
            }
        }

        // 伸手准备抓取
        if(step == STEP_REACH_OUT)
        {
            if(nCount == 0)
            {
                ReachOut(box_track_z + grab_z_offset);
                ManiGripper(0.7);
            }
            nCount ++;
            if(nCount > 6* 30)
            {
                nCount = 0;
                step = STEP_FORWARD;
            }
        }

        // 前进进行抓取
        if(step == STEP_FORWARD)
        {
            VelCmd(0.1,0,0);
            nCount ++;
            if(nCount > 1.0* 30)
            {
                nCount = 0;
                step = STEP_GRAB_BOX;
            }
        }

        // 闭合手爪
        if(step == STEP_GRAB_BOX)
        {
            VelCmd(0,0,0);
            ManiGripper(grab_gripper_value);
            nCount ++;
            if(nCount > 2* 30)
            {
                nCount = 0;
                step = STEP_TAKE_OVER;
            }
        }

        // 收回手臂
        if(step == STEP_TAKE_OVER)
        {
            VelCmd(0,0,0);
            sensor_msgs::JointState mani_ctrl_msg;
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
                mani_ctrl_msg.velocity[i] = 0.35;
            }
            mani_ctrl_msg.position[1] = -1.4;
            mani_ctrl_msg.position[2] = 1.4;
            mani_ctrl_msg.velocity[1] = 1.05;
            mani_ctrl_msg.velocity[3] = 0.21;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 3* 30)
            {
                nCount = 0;
                step = STEP_BACKWARD;
            }
        }

        // 后退
        if(step == STEP_BACKWARD)
        {
            VelCmd(-0.1,0,0);
            nCount ++;
            if(nCount > 1* 30)
            {
                nCount = 0;
                step = STEP_DONE;
            }
        }

        // 结束
        if(step == STEP_DONE)
        {
            if(nCount < 5)
            {
                VelCmd(0,0,0);
                grab_result_msg.data = "done";
                grab_result_pub.publish(grab_result_msg);
            }
            nCount ++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
