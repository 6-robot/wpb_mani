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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <math.h>

static int arManiPos[5];
static int arManiSpeed[5];
static float arFakeJointPos[10];
static float arTargetJointPos[10];

static double kAngleToDegree = 18000/3.1415926;
void JointCtrlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std_msgs::Float64 joint_pos_msg;
    int nNumJoint = msg->position.size();
    for(int i=0;i<nNumJoint;i++)
    {
        if(msg->name[i] == "joint1")
        {
           arTargetJointPos[6] = msg->position[i];
        }
        if(msg->name[i] == "joint2")
        {
           arTargetJointPos[7] = msg->position[i];
        }
        if(msg->name[i] == "joint3")
        {
           arTargetJointPos[8] = msg->position[i];
        }
        if(msg->name[i] == "joint4")
        {
           arTargetJointPos[9] = msg->position[i];
        }
        if(msg->name[i] == "gripper")
        {
           //arTargetJointPos[10] = msg->position[i];
        }
    }
     ROS_INFO("------------------------------");
    for(int i=0;i<nNumJoint;i++)
    {
        ROS_INFO("[wpb_mani] %d - %s = %.2f  vel= %.2f  ctlr= %d", i, msg->name[i].c_str(),msg->position[i],msg->velocity[i],arManiPos[i]);
    }
    ROS_INFO("------------------------------");
}

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
// 响应 Move_Group 的轨迹执行
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
    int nrOfPoints = goal->trajectory.points.size();
    ROS_WARN("Trajectory with %d positions received ", nrOfPoints);
    int nExecIndex = nrOfPoints - 1;
    int nPos =  goal->trajectory.points[nExecIndex].positions.size();
    ROS_WARN("Num of Joints =  %d  ", nPos);
    if(nPos > 5) nPos = 5;
    for(int i=0;i<nPos;i++)
    {
        arTargetJointPos[i+6] = goal->trajectory.points[nExecIndex].positions[i];
    }


    as->setSucceeded();
}

typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;
// 响应 GripperCommand 回调函数
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;
    float maxEffort = goal->command.max_effort;

    // 执行指令
   
	as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_mani_dummy");
    ros::NodeHandle n;
    ros::Subscriber mani_ctrl_sub = n.subscribe("/wpb_mani/joint_ctrl",30,&JointCtrlCallback);

    TrajectoryServer tserver(n, "wpb_mani_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
    tserver.start();
    GripperServer gserver(n, "wpb_mani_gripper_controller/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_INFO("GripperActionServer: Starting");
    gserver.start();

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(30.0);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState joint_msg;
    std::vector<std::string> joint_name(11);
    std::vector<double> joint_pos(11);
    for(int i=0;i<5;i++)
    {
        arManiPos[i] = 0;
        arManiSpeed[i] = 2000;
    }
    for(int i=0;i<11;i++)
    {
        arFakeJointPos[i] = 0;
        arTargetJointPos[i] = 0;
    }

    ros::NodeHandle n_param("~");
    joint_name[0] = "front_left_wheel_joint";
    joint_name[1] = "front_right_wheel_joint";
    joint_name[2] = "back_right_wheel_joint";
    joint_name[3] = "back_left_wheel_joint";
    joint_name[4] = "kinect_height";
    joint_name[5] = "kinect_pitch";
    joint_name[6] = "joint1";
    joint_name[7] = "joint2";
    joint_name[8] = "joint3";
    joint_name[9] = "joint4";
    joint_name[10] = "gripper";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    joint_pos[5] = -0.3f;
    joint_pos[6] = 0.0f;
    joint_pos[7] = 0.0f;
    joint_pos[8] = 0.0f;
    joint_pos[9] = 0.0f;
    joint_pos[10] = 0.0f;
    n_param.getParam("zeros/kinect_height", joint_pos[4]);
    n_param.getParam("zeros/kinect_pitch", joint_pos[5]);

    while(n.ok())
    {
        float step = 0.01;
        for(int i=0;i<11;i++)
        {
            if(arFakeJointPos[i] < arTargetJointPos[i])
            {
                arFakeJointPos[i] += step;
            }
             if(arFakeJointPos[i] > arTargetJointPos[i])
            {
                arFakeJointPos[i]  -= step;
            }
        }
        
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.header.seq ++;

        // wheels tf
        for(int i=0;i<4;i++)
        {
              joint_pos[i] = arFakeJointPos[i];
        }

        // mani tf
        for(int i=0;i<5;i++)
        {
            joint_pos[i+6] = arFakeJointPos[i+6];
        }
        joint_msg.name = joint_name;
        joint_msg.position = joint_pos;
        joint_state_pub.publish(joint_msg);
        // ROS_WARN("mani pos pub");

        ros::spinOnce();
        r.sleep();
    }
}
