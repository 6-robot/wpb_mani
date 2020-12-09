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
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_mani_arm_forward_kinematics");
    ROS_WARN("wpb_mani_arm_forward_kinematics start!");

    // 从系统参数“robot_description”中获取机器人运动学模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    // 获取运动规划组对象，“wpb_arm”为规划组名称
    const robot_state::JointModelGroup *joint_model_group =kinematic_model->getJointModelGroup("wpb_arm");

    // 将机器人手臂关节角度值作为输入量，计算手臂末端的姿态
    std::vector<double> joint_values;
    joint_values.push_back(0.0); //"joint1";
    joint_values.push_back(0.7); //"joint2";
    joint_values.push_back(0.57); //"joint3";
    joint_values.push_back(-1.35); //"joint4";
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    printf("输入关节角度：\n");
    for(int i=0;i<4;i++)
    {
        printf("joint%d = %f\n" , i+1 , joint_values[i]);
    }

    // 获取运动学正解，参数为需要获取位姿的link（ 查看urdf文件可以知道：end_effector_link 为机械臂手爪指尖的link ）
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector_link");

    // 获取指尖空间坐标
    float x = end_effector_state.translation().data()[0];
    float y = end_effector_state.translation().data()[1];
    float z = end_effector_state.translation().data()[2];
    printf("输出指尖坐标 = ( %f , %f , %f)\n", x , y , z);

    // 将手爪指尖朝向转换成欧拉角
    Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles ( 2,1,0 ); // ZYX 顺序,即 yaw pitch roll
    float yaw = euler_angles[0];
    float pitch = euler_angles[1];
    float roll = euler_angles[2];
    printf("输出指尖朝向= ( %f , %f , %f)\n", yaw , pitch , roll);

    // 将关节角度值输出到机器人实体
    ros::NodeHandle n;
    ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);

    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(4);
    ctrl_msg.position.resize(4);
    ctrl_msg.velocity.resize(4);
    ctrl_msg.name[0] = "joint1";
    ctrl_msg.name[1] = "joint2";
    ctrl_msg.name[2] = "joint3";
    ctrl_msg.name[3] = "joint4";
    for(int i=0;i<4;i++)
    {
        ctrl_msg.position[i] = joint_values[i];
        ctrl_msg.velocity[i] = 10;
    }

    ros::Rate r(10);
    while(ros::ok())
    {
        mani_ctrl_pub.publish(ctrl_msg); 
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}