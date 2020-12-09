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
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <math.h>
#include <vector>

using namespace std;

static ros::Publisher front_left_wheel_pub;
static ros::Publisher front_right_wheel_pub;
static ros::Publisher back_left_wheel_pub;
static ros::Publisher back_right_wheel_pub;
static std_msgs::Float64 wheel_speed_msg;

static ros::Publisher joint1_pub;
static ros::Publisher joint2_pub;
static ros::Publisher joint3_pub;
static ros::Publisher joint4_pub;
static ros::Publisher gripper_pub;
static ros::Publisher gripper_sub_pub;
static ros::Publisher kinect_height_pub;
static ros::Publisher kinect_pitch_pub;
static ros::Publisher joint_state_pub;
static std_msgs::Float64 joint_pos_msg;
static geometry_msgs::Pose2D pose_reset;
static geometry_msgs::Pose2D pose_cur;
static  tf::Transform tf_odom_reset;
static tf::TransformBroadcaster* tf_broadcast;
static float joint_state_position[12];

typedef struct
{
    float position[4];
    int velocity[4];
}st_wpm_pose;

void InitGripperPosVal();
int CalGripperPos(float inGapSize);
void ManiGripper(float inGripperVal);

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static double fJointAngle[5];
static int nJointSpeed[5];
static double pos_send[4];
static int vel_send[4];
static float wheel_speed[4]; 
static float wheel_position[4]; 
static st_wpm_pose tmpPose;
static vector<st_wpm_pose> arPose;
static int nExecIndex = 0;
static bool bExecPath = false;
static bool bExecToGoal = true;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;

bool poseArrived()
{
    bool bArrived = true;
    for(int i=0;i<4;i++)
    {
        double fAngleDiff = fabs(arPose[nExecIndex].position[i] - joint_state_position[i+6]);
        if(fAngleDiff > 0.1)
        {
            //未运动到目标点
            //ROS_INFO("posArrived i=%d targe=%.2f recv=%.2f fDegDiff=%.2f",i,arPose[nExecIndex].position[i],m_mani.nRecvJointPos[i+1]*0.01,fDegDiff);
            bArrived = false;
        }
    }
    return bArrived;
}

// 响应 Move_Group 的轨迹执行
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
    arPose.clear();
    int nrOfPoints = goal->trajectory.points.size();
    ROS_WARN("Trajectory with %d positions received \n", nrOfPoints);
    //是否快速执行
    if(bExecToGoal == false)
    {
        nExecIndex = 0;
    }
    else
    {
        nExecIndex = nrOfPoints - 1;
    }
    //解析关键帧列表
    for(int i=0; i<nrOfPoints; i++)
    {
        int nPos =  goal->trajectory.points[i].positions.size();
        //ROS_WARN("point[%d] hase %d positions\n",i, nPos);
        if(nPos > 4)
        nPos = 4;
        for(int j=0;j<nPos;j++)
        {
            tmpPose.position[j] = goal->trajectory.points[i].positions[j] ;
            tmpPose.velocity[j] = 10;
        }
        arPose.push_back(tmpPose);
    }
    
    bExecPath = true;
    
    ros::Rate r(30);
    while( bExecPath == true)
    {
        if(poseArrived() == true)
        {
            nExecIndex ++;
        }
        if(nExecIndex >= arPose.size())
        {
            // 执行完毕
            bExecPath = false;
            ROS_INFO("ExecPath done!");
        }
        else
        {
            //未执行完毕
            for(int i=0;i<4;i++)
            {
                pos_send[i] = arPose[nExecIndex].position[i];
                vel_send[i] = arPose[nExecIndex].velocity[i];
            }
            
            // 速度优化
            double fTmpPosDiff[4];
            for(int i=0;i<4;i++)
            {
                fTmpPosDiff[i] = fabs(pos_send[i] - joint_state_position[i]);
            }
            // 找出位置差最大的值,以它为最大速度
            double fDiffMax = 0;
            int nDiffMaxIndex = 0;
            for(int i=0;i<4;i++)
            {
                if(fTmpPosDiff[i] > fDiffMax)
                {
                    fDiffMax = fTmpPosDiff[i];
                    nDiffMaxIndex = i;
                }
            }
            // 计算运动速度
            int nMaxVelocity = 10;
            if(fDiffMax > 0)
            {
                for(int i=0;i<4;i++)
                {
                    double tmpVel = fTmpPosDiff[i];
                    tmpVel /= fDiffMax;
                    tmpVel *= nMaxVelocity;
                    vel_send[i] = tmpVel;
                    if(vel_send[i] < 2)
                        vel_send[i] = 2;
                }
            }
            
            for(int i=0;i<4;i++)
            {
                fJointAngle[i+1] = pos_send[i];
                nJointSpeed[i+1] = vel_send[i];
                //ROS_INFO("[executeTrajectory] %d - s_pos=%.2f d_pos=%.2f spd=%d", i, pos_send[i],goal->trajectory.points[nExecIndex].positions[i] * fAngToDeg,vel_send[i]);  //显示规划路径的角度值
            }
            //ROS_WARN("point= %d   nExecIndex= %d",nrOfPoints, nExecIndex);
            std_msgs::Float64 joint_pos_msg;
            joint_pos_msg.data =  pos_send[0] ;
            joint1_pub.publish(joint_pos_msg);
            joint_pos_msg.data =  pos_send[1] ;
            joint2_pub.publish(joint_pos_msg);
            joint_pos_msg.data =  pos_send[2] ;
            joint3_pub.publish(joint_pos_msg);
            joint_pos_msg.data =  pos_send[3] ;
            joint4_pub.publish(joint_pos_msg);
        }

        r.sleep();
    }
    as->setSucceeded();
}

// 响应 GripperCommand 回调函数
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;
    float maxEffort = goal->command.max_effort;

    int nGripperPos = CalGripperPos(gapSize);
    ROS_WARN("[executeGripper]gapSize = %f, gripprPos = %d", gapSize, nGripperPos);

    // 执行指令
    ManiGripper(nGripperPos);

    // 监测执行目标是否完成
    while(ros::ok())
    {
        int nDiff = abs(nGripperPos - joint_state_position[4]);
        if(nDiff < 100)
        {
            break;
        }
    }

	as->setSucceeded();
}

float CalGripperSize(float inJointPos)
{
    float retVal = inJointPos;
    retVal = retVal /4-0.01;
    return retVal;
}

void ManiGripper(float inGripperVal)
{
    joint_pos_msg.data = inGripperVal;
    gripper_pub.publish(joint_pos_msg);
    gripper_sub_pub.publish(joint_pos_msg);
}

void JointCtrlCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_WARN("JointCtrlCallback");
    std_msgs::Float64 joint_pos_msg;
    int nNumJoint = msg->position.size();
    for(int i=0;i<nNumJoint;i++)
    {
        if(msg->name[i] == "joint1")
        {
           joint_pos_msg.data = msg->position[i] ;
           joint1_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "joint2")
        {
           joint_pos_msg.data = msg->position[i] ;
           joint2_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "joint3")
        {
           joint_pos_msg.data = msg->position[i] ;
           joint3_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "joint4")
        {
           joint_pos_msg.data = msg->position[i] ;
           joint4_pub.publish(joint_pos_msg);
        }
        if(msg->name[i] == "gripper")
        {
             //手爪
            float fGripperVal = CalGripperSize(msg->position[i]);
            ManiGripper(fGripperVal);
        }
        //ROS_INFO("[wpr1_mani_cb] %d - %s = %.2f", i, msg->name[i].c_str(),msg->position[i]);
    }
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
        pose_reset.x = pose_cur.x;
        pose_reset.y = pose_cur.y;
        pose_reset.theta = pose_cur.theta;
        tf_odom_reset.setOrigin( tf::Vector3(pose_reset.x, pose_reset.y, 0.0) );
        tf_odom_reset.setRotation( tf::createQuaternionFromRPY(0,0,pose_reset.theta) );
    }
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pose_cur.x = msg->pose.pose.position.x;
    pose_cur.y = msg->pose.pose.position.y;
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    pose_cur.theta = tf::getYaw(q);
    //ROS_WARN("[Odom]  ( %.2f  %.2f )   %.2f",pose_cur.x,pose_cur.y,pose_cur.theta);
    tf_broadcast->sendTransform(tf::StampedTransform(tf_odom_reset, ros::Time::now(), "/odom", "/pose_reset"));
}

void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int joints_num = msg->name.size();
    //ROS_WARN("---------------------------------------");
    //ROS_WARN("joints_num = %d",joints_num);
    for(int i=0;i<joints_num;i++)
    {
        joint_state_position[i] = msg->position[i];
        //ROS_WARN("[%d]  %s = %.2f",i,msg->name[i].c_str(),msg->position[i]);
    }
    joint_state_pub.publish(msg);
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpb_mani_sim] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    float k_linear_x = -1.0f/3;
    float linear_x = msg->linear.x* k_linear_x;
	wheel_speed[0] = -linear_x;
	wheel_speed[1] = linear_x;
	wheel_speed[2] = linear_x;
	wheel_speed[3] = -linear_x;

	//shif left right
    float k_linear_y = k_linear_x*-1.15;
    float linear_y = msg->linear.y* k_linear_y;
	wheel_speed[0] += linear_y;
	wheel_speed[1] += linear_y;
	wheel_speed[2] += -linear_y;
	wheel_speed[3] += -linear_y;

	//Turning 
    float k_angular_z = -1.0/3.5;
    float angular_z =  msg->angular.z* k_angular_z;
	wheel_speed[0] += angular_z;
	wheel_speed[1] += angular_z;
	wheel_speed[2] += angular_z;
	wheel_speed[3] += angular_z;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_mani_sim");
    ros::NodeHandle nh_param("~");
    ros::NodeHandle n;
    tf_broadcast = new tf::TransformBroadcaster;
    tf_odom_reset.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf_odom_reset.setRotation( tf::createQuaternionFromRPY(0,0,0) );
    tf::TransformListener listener;
    tf::StampedTransform tf_diff;

    front_left_wheel_pub = n.advertise<std_msgs::Float64>("/wpb_mani/front_left_controller/command",10);
    front_right_wheel_pub = n.advertise<std_msgs::Float64>("/wpb_mani/front_right_controller/command",10);
    back_left_wheel_pub = n.advertise<std_msgs::Float64>("/wpb_mani/back_left_controller/command",10);
    back_right_wheel_pub = n.advertise<std_msgs::Float64>("/wpb_mani/back_right_controller/command",10);
    joint1_pub = n.advertise<std_msgs::Float64>("/wpb_mani/joint1_position_controller/command",10);
    joint2_pub = n.advertise<std_msgs::Float64>("/wpb_mani/joint2_position_controller/command",10);
    joint3_pub = n.advertise<std_msgs::Float64>("/wpb_mani/joint3_position_controller/command",10);
    joint4_pub = n.advertise<std_msgs::Float64>("/wpb_mani/joint4_position_controller/command",10);
    gripper_pub = n.advertise<std_msgs::Float64>("/wpb_mani/gripper_position_controller/command",10);
    gripper_sub_pub = n.advertise<std_msgs::Float64>("/wpb_mani/gripper_sub_position_controller/command",10);
    kinect_height_pub = n.advertise<std_msgs::Float64>("/wpb_mani/kinect_height_position_controller/command",10);
    kinect_pitch_pub = n.advertise<std_msgs::Float64>("/wpb_mani/kinect_pitch_position_controller/command",10);

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",10,&CmdVelCallback);
    ros::Subscriber joint_ctrl_sub = n.subscribe("/wpb_mani/joint_ctrl",30,&JointCtrlCallback);
    ros::Subscriber ctrl_sub = n.subscribe("/wpb_mani/ctrl",10,&CtrlCallback);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCallback);
    ros::Publisher pose_diff_pub = n.advertise<geometry_msgs::Pose2D>("/wpb_mani/pose_diff",1);
    ros::Subscriber joint_state_sub = n.subscribe("/wpb_mani/joint_states", 1, JointStatesCallback);
    joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    pose_diff_msg.x = 0;
    pose_diff_msg.y = 0;
    pose_diff_msg.theta = 0;

    sleep(8);

    TrajectoryServer tserver(n, "wpb_mani_controller/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_WARN("TrajectoryActionServer: Starting");
    tserver.start();
    InitGripperPosVal();
    GripperServer gserver(n, "wpb_mani_gripper_controller/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_WARN("GripperActionServer: Starting");
    gserver.start();

    //机械臂基座旋转关节
    joint_pos_msg.data = 0.0;
    joint1_pub.publish(joint_pos_msg);
    //机械臂根关节
    joint_pos_msg.data =  -1.4;
    joint2_pub.publish(joint_pos_msg);
    //机械臂肘关节
    joint_pos_msg.data =  1.4;
    joint3_pub.publish(joint_pos_msg);
    //机械臂腕关节
    joint_pos_msg.data = 0;
    joint4_pub.publish(joint_pos_msg);
    //手爪
    ManiGripper(0.05);

    //电机转速
    for(int i=0;i<4;i++)
    {
        wheel_position[i] = 0;
        wheel_speed[i] = 0;
    }

    //Azure Kinect
    float kinect_height = 0.0f;
    nh_param.getParam("zeros/kinect_height", kinect_height);
    joint_pos_msg.data = kinect_height;
    kinect_height_pub.publish(joint_pos_msg);
    float kinect_pitch = 0.0f;
    nh_param.getParam("zeros/kinect_pitch", kinect_pitch);
    joint_pos_msg.data = kinect_pitch;
    kinect_pitch_pub.publish(joint_pos_msg);
    ROS_WARN("wpb_mani_joints_inited!");

    ros::Rate r(20.0);

    while(n.ok())
    {
        bool res = listener.waitForTransform("/pose_reset","/base_footprint",ros::Time(0), ros::Duration(1.0));
        if(res == true)
        {
            listener.lookupTransform("/pose_reset","/base_footprint",ros::Time(0),tf_diff);

            pose_diff_msg.x = tf_diff.getOrigin().x();
            pose_diff_msg.y = tf_diff.getOrigin().y();
            tf::Quaternion pose_quat = tf_diff.getRotation ();
            float pose_yaw = tf::getYaw(pose_quat);
            pose_diff_msg.theta = pose_yaw - pose_reset.theta;
            pose_diff_pub.publish(pose_diff_msg);
            // ROS_INFO("[pose_diff_msg] x= %.2f  y=%.2f  th= %.2f", pose_diff_msg.x,pose_diff_msg.y,pose_diff_msg.theta);
        }

        for(int i=0;i<4;i++)
        {
            wheel_position[i]  += wheel_speed[i];
        }
        wheel_speed_msg.data = wheel_position[0];
        front_left_wheel_pub.publish(wheel_speed_msg);
        wheel_speed_msg.data = wheel_position[1];
        front_right_wheel_pub.publish(wheel_speed_msg);
        wheel_speed_msg.data = wheel_position[2];
        back_right_wheel_pub.publish(wheel_speed_msg);
        wheel_speed_msg.data = wheel_position[3];
        back_left_wheel_pub.publish(wheel_speed_msg);
        //printf("wheel_speed= 1:%.2f   2:%.2f   3:%.2f   4:%.2f\n", wheel_position[0], wheel_position[0], wheel_position[0], wheel_position[0]);

        ros::spinOnce();
        r.sleep();
    }
    delete tf_broadcast;
}