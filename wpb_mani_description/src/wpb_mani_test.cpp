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
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <vector>
#include <string>

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double fJointAngle[6];
static int nJointSpeed[6];

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_mani_test");
    ROS_INFO("[wpb_mani_test]");
    ros::NodeHandle n;
    for(int i=0;i<5;i++)
    {
        fJointAngle[i] = 0;
        nJointSpeed[i] = 1500;
    }

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",10);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(11);
    std::vector<double> joint_pos(11);

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
    joint_pos[5] = 0.0f;
    joint_pos[6] = 0.0f;
    joint_pos[7] = 0.0f;
    joint_pos[8] = 0.0f;
    joint_pos[9] = 0.0f;
    joint_pos[10] = 0.0f;

    int nCount = 0;
    ros::Rate r(30);
    while(n.ok())
    {
        nCount += 5;
        joint_pos[0] = (float)nCount*0.01;
        joint_pos[1] = (float)nCount*-0.01;
        joint_pos[2] = (float)nCount*-0.01;
        joint_pos[3] = (float)nCount*0.01;

        msg.header.stamp = ros::Time::now();
        msg.header.seq ++;
        msg.name = joint_name;
        msg.position = joint_pos;
        joint_state_pub.publish(msg);
        
        ros::spinOnce();
        r.sleep();
    }
}