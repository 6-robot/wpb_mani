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
#include "driver/WPB_Mani_driver.h"
#include <math.h>

static CWPB_Mani_driver m_wpb_mani;
static int nLastMotorPos[4];
static int arManiPos[5];
static int arManiSpeed[5];
static geometry_msgs::Twist lastVel;
void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpb_mani] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    m_wpb_mani.Velocity(msg->linear.x,msg->linear.y,msg->angular.z);

    lastVel.linear.x = msg->linear.x;
    lastVel.linear.y = msg->linear.y;
    lastVel.angular.z = msg->angular.z; 
}

static double kAngleToDegree = 18000/3.1415926;
void JointCtrlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std_msgs::Float64 joint_pos_msg;
    int nNumJoint = msg->position.size();
    for(int i=0;i<nNumJoint;i++)
    {
        if(msg->name[i] == "joint1")
        {
           arManiPos[0] = msg->position[i] * kAngleToDegree;
           arManiSpeed[0] = msg->velocity[i] * kAngleToDegree;
        }
        if(msg->name[i] == "joint2")
        {
           arManiPos[1] = msg->position[i] * kAngleToDegree;
           arManiSpeed[1] = msg->velocity[i] * kAngleToDegree;
        }
        if(msg->name[i] == "joint3")
        {
           arManiPos[2] = msg->position[i] * kAngleToDegree;
           arManiSpeed[2] = msg->velocity[i] * kAngleToDegree;
        }
        if(msg->name[i] == "joint4")
        {
           arManiPos[3] = msg->position[i] * kAngleToDegree;
           arManiSpeed[3] = msg->velocity[i] * kAngleToDegree;
        }
        if(msg->name[i] == "gripper")
        {
            //手爪
            int nGripperVal = 12000 - msg->position[i]*100*2000;
            if(nGripperVal < 0)
                nGripperVal -= 1000;
            if(nGripperVal < -4000)
                nGripperVal = -4000;
            if(nGripperVal > 10000)
                nGripperVal = 10000;
            arManiPos[4] = nGripperVal;
            arManiSpeed[4] = msg->velocity[i] * kAngleToDegree;
        }
    }
    // for(int i=0;i<nNumJoint;i++)
    // {
    //     ROS_INFO("[wpb_mani] %d - %s = %.2f  vel= %.2f  ctlr= %d", i, msg->name[i].c_str(),msg->position[i],msg->velocity[i],arManiPos[i]);
    // }
    m_wpb_mani.SetManiSpd(arManiSpeed[0] , arManiSpeed[1] , arManiSpeed[2] , arManiSpeed[3] , arManiSpeed[4] );
    m_wpb_mani.SendManiPos( arManiPos[0] , arManiPos[1] , arManiPos[2] , arManiPos[3] , arManiPos[4] );
}

static geometry_msgs::Pose2D pose_diff_msg;
void CtrlCallback(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pose_diff reset");
    if( nFindIndex >= 0 )
    {
        pose_diff_msg.x = 0;
        pose_diff_msg.y = 0;
        pose_diff_msg.theta = 0;
        //ROS_WARN("[pose_diff reset]");
    }
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
    if(nPos > 4) nPos = 4;
    for(int i=0;i<nPos;i++)
    {
        ROS_WARN("[%d] pos= %.2f ", i,goal->trajectory.points[nExecIndex].positions[i]);
        arManiPos[i] = goal->trajectory.points[nExecIndex].positions[i] * kAngleToDegree;
    }

    m_wpb_mani.SendManiPos( arManiPos[0] , arManiPos[1] , arManiPos[2] , arManiPos[3] , arManiPos[4] );

    as->setSucceeded();
}

typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;
// 响应 GripperCommand 回调函数
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;
    float maxEffort = goal->command.max_effort;

    // 执行指令
    int nGripperVal = 12000 - goal->command.position*100*2000;
    if(nGripperVal < 0)
        nGripperVal -= 1000;
    if(nGripperVal < -4000)
        nGripperVal = -4000;
    if(nGripperVal > 10000)
        nGripperVal = 10000;
    arManiPos[4] = nGripperVal;

    ROS_WARN("[executeGripper]gapSize = %f, nGripperVal = %d", gapSize, nGripperVal);

    m_wpb_mani.SendManiPos( arManiPos[0] , arManiPos[1] , arManiPos[2] , arManiPos[3] , arManiPos[4] );
	as->setSucceeded();
}

static double fKVx =1.0f/(1900*2*4.5);
static double fKVy = 1.0f/(2200*2*4.4);
static double fKVz = 1.0f/(740*4*4.84);
static float fSumX =0;
static float fSumY =0;
static float fSumZ =0;
static float fOdomX =0;
static float fOdomY =0;
static float fOdomZ =0;
static double fManiPosK = 0.01*M_PI/180;
static double fWheelPosK = -0.1*M_PI/180;
static geometry_msgs::Pose2D lastPose;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_mani_core");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",10,&CmdVelCallback);
    ros::Subscriber mani_ctrl_sub = n.subscribe("/wpb_mani/joint_ctrl",30,&JointCtrlCallback);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu >("imu/data_raw", 100);

    TrajectoryServer tserver(n, "wpb_mani_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
    tserver.start();
    GripperServer gserver(n, "wpb_mani_gripper_controller/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_INFO("GripperActionServer: Starting");
    gserver.start();

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_wpb_mani.Open(strSerialPort.c_str(),115200);

    bool bImu;
    n_param.param<bool>("imu", bImu, false);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(20.0);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState joint_msg;
    std::vector<std::string> joint_name(11);
    std::vector<double> joint_pos(11);
    for(int i=0;i<5;i++)
    {
        arManiPos[i] = 0;
        arManiSpeed[i] = 2000;
    }

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

    ros::Publisher odom_pub;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    ros::Subscriber ctrl_sub = n.subscribe("/wpb_mani/ctrl",10,&CtrlCallback);
    ros::Publisher pose_diff_pub = n.advertise<geometry_msgs::Pose2D>("/wpb_mani/pose_diff",1);
    pose_diff_msg.x = 0;
    pose_diff_msg.y = 0;
    pose_diff_msg.theta = 0;

    lastPose.x = lastPose.y = lastPose.theta = 0;
    lastVel.linear.x = lastVel.linear.y = lastVel.linear.z = lastVel.angular.x = lastVel.angular.y = lastVel.angular.z = 0;
    nLastMotorPos[0] = nLastMotorPos[1] = nLastMotorPos[2] = nLastMotorPos[3] = 0;
    while(n.ok())
    {
        m_wpb_mani.ReadNewData();
        m_wpb_mani.nParseCount ++;
        //ROS_INFO("[m_wpb_mani.nParseCount]= %d",m_wpb_mani.nParseCount);
        if(m_wpb_mani.nParseCount > 100)
        {
            m_wpb_mani.arMotorPos[0] =0; nLastMotorPos[0] = 0;
            m_wpb_mani.arMotorPos[1] =0; nLastMotorPos[1] = 0;
            m_wpb_mani.arMotorPos[2] =0; nLastMotorPos[2] = 0;
            m_wpb_mani.arMotorPos[3] =0; nLastMotorPos[3] = 0;
            m_wpb_mani.nParseCount = 0;
            //ROS_INFO("empty");
        }
        
        last_time = current_time;
        current_time = ros::Time::now();

        double fVx,fVy,fVz;
        double fPosDiff[4];
        if(nLastMotorPos[0] != m_wpb_mani.arMotorPos[0] || nLastMotorPos[1] != m_wpb_mani.arMotorPos[1] || nLastMotorPos[2] != m_wpb_mani.arMotorPos[2] || nLastMotorPos[3] != m_wpb_mani.arMotorPos[3])
        //if(true)
        {
            //ROS_WARN("[M0] %d    [M1] %d    [M2] %d    [M3] %d",m_wpb_mani.arMotorPos[0],m_wpb_mani.arMotorPos[1],m_wpb_mani.arMotorPos[2],m_wpb_mani.arMotorPos[3]);
            fPosDiff[0] = (double)(m_wpb_mani.arMotorPos[0] - nLastMotorPos[0]); 
            fPosDiff[1] = (double)(m_wpb_mani.arMotorPos[1] - nLastMotorPos[1]);
            fPosDiff[2] = (double)(m_wpb_mani.arMotorPos[2] - nLastMotorPos[2]);
            fPosDiff[3] = (double)(m_wpb_mani.arMotorPos[3] - nLastMotorPos[3]);
            //ROS_INFO("[D0] %.2f    [D1] %.2f   [D2] %.2f    [D3] %.2f",fPosDiff[0],fPosDiff[1],fPosDiff[2],fPosDiff[3]);
            int nMaxDiff = 1000;
            if(
                fabs( fPosDiff[0] ) > nMaxDiff || fabs( fPosDiff[1] ) > nMaxDiff || fabs( fPosDiff[2] ) > nMaxDiff || fabs( fPosDiff[3] ) > nMaxDiff 
            )
            {
                ROS_WARN("[M0]= %d    last =  %d",m_wpb_mani.arMotorPos[0],nLastMotorPos[0]); 
                ROS_WARN("[M1]= %d    last =  %d",m_wpb_mani.arMotorPos[1],nLastMotorPos[1]); 
                ROS_WARN("[M2]= %d    last =  %d",m_wpb_mani.arMotorPos[2],nLastMotorPos[2]); 
                ROS_WARN("[M3]= %d    last =  %d",m_wpb_mani.arMotorPos[3],nLastMotorPos[3]);  
                ROS_WARN("-----------------------------------"); 
            }

            
            fVx = (fPosDiff[1] - fPosDiff[0]) * fKVx;
            fVy = (fPosDiff[1] - fPosDiff[2]) *fKVy;
            fVz = (fPosDiff[0] + fPosDiff[1] + fPosDiff[2] + fPosDiff[3])*fKVz;
            double fTimeDur = current_time.toSec() - last_time.toSec();
            //方案一 通过电机码盘解算里程计数据（位移/时间=速度）
            fVx = fVx/(fTimeDur);
            fVy = fVy/(fTimeDur);
            fVz = fVz/(fTimeDur);
            //ROS_WARN("[Velocity]    fVx=%.2f   fVy=%.2f    fVz=%.2f",fVx,fVy,fVz);
            // 方案二 直接把下发速度当作里程计积分依据
            // fVx = lastVel.linear.x;
            // fVy = lastVel.linear.y;
            // fVz = lastVel.angular.z;
            
            double dx = (lastVel.linear.x*cos(lastPose.theta) - lastVel.linear.y*sin(lastPose.theta))*fTimeDur;
            double dy = (lastVel.linear.x*sin(lastPose.theta) + lastVel.linear.y*cos(lastPose.theta))*fTimeDur;

            lastPose.x += dx;
            lastPose.y += dy;
            lastPose.theta += (fVz*fTimeDur);

            double pd_dx = (lastVel.linear.x*cos(pose_diff_msg.theta) - lastVel.linear.y*sin(pose_diff_msg.theta))*fTimeDur;
            double pd_dy = (lastVel.linear.x*sin(pose_diff_msg.theta) + lastVel.linear.y*cos(pose_diff_msg.theta))*fTimeDur;
            pose_diff_msg.x += pd_dx;
            pose_diff_msg.y += pd_dy;
            pose_diff_msg.theta += (fVz*fTimeDur);

            odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,lastPose.theta);
            //updata transform
            odom_trans.header.stamp = current_time;
            odom_trans.transform.translation.x = lastPose.x;
            odom_trans.transform.translation.y = lastPose.y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(lastPose.theta);

            //filling the odometry
            odom.header.stamp = current_time;
            //position
            odom.pose.pose.position.x = lastPose.x;
            odom.pose.pose.position.y = lastPose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            //velocity
            odom.twist.twist.linear.x = fVx;
            odom.twist.twist.linear.y = fVy;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = fVz;

            //plublishing the odometry and new tf
            broadcaster.sendTransform(odom_trans);
            odom_pub.publish(odom);

            lastVel.linear.x = fVx;
            lastVel.linear.y = fVy;
            lastVel.angular.z = fVz;

            nLastMotorPos[0] = m_wpb_mani.arMotorPos[0];
            nLastMotorPos[1] = m_wpb_mani.arMotorPos[1];
            nLastMotorPos[2] = m_wpb_mani.arMotorPos[2];
            nLastMotorPos[3] = m_wpb_mani.arMotorPos[3];
        }
        else
        {
            odom_trans.header.stamp = ros::Time::now();
            //plublishing the odometry and new tf
            broadcaster.sendTransform(odom_trans);
            odom.header.stamp = ros::Time::now();
            odom_pub.publish(odom);
            //ROS_INFO("[odom] zero");
        }

        pose_diff_pub.publish(pose_diff_msg);
        //ROS_INFO("[pose_diff_msg] x= %.2f  y=%.2f  th= %.2f", pose_diff_msg.x,pose_diff_msg.y,pose_diff_msg.theta);
    
        if(bImu == true)
        {
            //imu
            sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu";
            imu_msg.orientation.x = m_wpb_mani.fQuatX;
            imu_msg.orientation.y = m_wpb_mani.fQuatY;
            imu_msg.orientation.z = m_wpb_mani.fQuatZ;
            imu_msg.orientation.w = m_wpb_mani.fQuatW;

            imu_msg.angular_velocity.x = m_wpb_mani.fGyroX;
            imu_msg.angular_velocity.y = m_wpb_mani.fGyroY;
            imu_msg.angular_velocity.z = m_wpb_mani.fGyroZ;

            imu_msg.linear_acceleration.x = m_wpb_mani.fAccX;
            imu_msg.linear_acceleration.y = m_wpb_mani.fAccY;
            imu_msg.linear_acceleration.z = m_wpb_mani.fAccZ;

            imu_pub.publish(imu_msg);
        }

        // wheels tf
        for(int i=0;i<4;i++)
        {
              joint_pos[i] = m_wpb_mani.arMotorPos[i]*fWheelPosK;
        }

        // mani tf
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.header.seq ++;
        for(int i=0;i<5;i++)
        {
            joint_pos[i+6] = m_wpb_mani.arManiPosRecv[i]*fManiPosK;
        }
        // gripper
        joint_pos[10]  = 0.02 - (float)(m_wpb_mani.arManiPosRecv[4]+2000)/400000;
        joint_msg.name = joint_name;
        joint_msg.position = joint_pos;
        joint_state_pub.publish(joint_msg);
        // ROS_WARN("mani pos pub");

        ros::spinOnce();
        r.sleep();
    }
}
