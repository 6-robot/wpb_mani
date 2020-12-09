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

#define CMD_WAIT_SEC 10

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_joint_control");

    ros::NodeHandle n;
    ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);
    sleep(2);

    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(5);
    ctrl_msg.position.resize(5);
    ctrl_msg.velocity.resize(5);
    ctrl_msg.name[0] = "joint1";
    ctrl_msg.name[1] = "joint2";
    ctrl_msg.name[2] = "joint3";
    ctrl_msg.name[3] = "joint4";
    ctrl_msg.name[4]= "gripper";
    for(int i=0;i<5;i++)
    {
        ctrl_msg.position[i] = 0;
        ctrl_msg.velocity[i] = 0.5;
    }

    int nCount = 0;
    ros::Rate r(0.1);
    
    while(ros::ok())
    {
        ROS_INFO("[wpb_mani_joint_control] nCount = %d", nCount);
        switch(nCount)
        {
        case 0:  //变成初始状态
            ctrl_msg.position[0] = 0;     //旋转底座(单位:弧度)
            ctrl_msg.position[1] = 0;     //根关节(单位:弧度)
            ctrl_msg.position[2] = 0;     //肘关节(单位:弧度)
            ctrl_msg.position[3] = 0;     //腕关节(单位:弧度)
            ctrl_msg.position[4] = 0;     //手指间距(单位:米)
            break;

        case 1: //向前伸展
            ctrl_msg.position[0] = 0;     //旋转底座(单位:弧度)
            ctrl_msg.position[1] = 1.57;     //根关节(单位:弧度)
            ctrl_msg.position[2] = -0.67;     //肘关节(单位:弧度)
            ctrl_msg.position[3] = -0.9;     //腕关节(单位:弧度)
            ctrl_msg.position[4] = 0.07;     //手指间距(单位:米)
            break;

        case 2:  //回到初始状态
            ctrl_msg.position[0] = 0;     //旋转底座(单位:弧度)
            ctrl_msg.position[1] = 0;     //根关节(单位:弧度)
            ctrl_msg.position[2] = 0;     //肘关节(单位:弧度)
            ctrl_msg.position[3] = 0;     //腕关节(单位:弧度)
            ctrl_msg.position[4] = 0;     //手指间距(单位:米)
            break;

        case 3:  //向后折叠
            ctrl_msg.position[0] = 0;     //旋转底座(单位:弧度)
            ctrl_msg.position[1] = -1.57;     //根关节(单位:弧度)
            ctrl_msg.position[2] = 1.2;     //肘关节(单位:弧度)
            ctrl_msg.position[3] = 0.4;     //腕关节(单位:弧度)
            ctrl_msg.position[4] = 0.05;     //手指间距(单位:米)
            break;
        }
        mani_ctrl_pub.publish(ctrl_msg);    //发送指令
        nCount ++;
        if(nCount > 3)
        {
            nCount = 0;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}