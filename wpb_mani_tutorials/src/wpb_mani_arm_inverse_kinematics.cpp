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
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_mani_arm_inverse_kinematics");
    ROS_WARN("wpb_mani_arm_inverse_kinematics start!");
    ros::NodeHandle nh;

    // 从系统参数“robot_description”中获取机器人运动学模型
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    // 获取运动规划组对象，“wpb_arm”为规划组名称
    const robot_state::JointModelGroup *joint_model_group =kinematic_model->getJointModelGroup("wpb_arm");

    // 通过赋值指定一个目标姿态作为输入
    geometry_msgs::Pose end_effector_pose;
    end_effector_pose.position.x = 0.4;
    end_effector_pose.position.y = 0.0;
    end_effector_pose.position.z = 0.3;
    
    // 目标姿态朝向的欧拉角表示,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    // 将欧拉角旋转量转换成四元数表达并赋值到目标姿态当中
    tf::StampedTransform transform;
    transform.setRotation(quat);
    end_effector_pose.orientation.x = transform.getRotation().getX();
    end_effector_pose.orientation.y = transform.getRotation().getY();
    end_effector_pose.orientation.z = transform.getRotation().getZ();
    end_effector_pose.orientation.w = transform.getRotation().getW();
    
    // 用手爪末端的关节名称及目标姿态作为输入，求运动学反解
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_pose,"end_effector_link", 10, 1.0);
    if (found_ik)
    {
        // 运动学反解有解，将结果显示出来
        const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
        std::vector<double> joint_values;
        joint_values.resize(6);
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            // 显示的时候将单位从弧度换算到度，这样便于观察比对
            printf("反解得到关节值 %s = %f  \n", joint_names[i].c_str(), joint_values[i]);
        }

        // 将反解出的关节角度值输出到机器人实体
        ros::NodeHandle n;
        ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_mani/joint_ctrl", 30);

        sensor_msgs::JointState ctrl_msg;
        ctrl_msg.name.resize(4);
        ctrl_msg.position.resize(4);
        ctrl_msg.velocity.resize(4);
        for(int i=0;i<4;i++)
        {
            ctrl_msg.name[i] = joint_names[i];
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
    }
    else
    {
        // 运动学反解无解
        ROS_ERROR("[ERROR] Did not find IK solution");
    }

    return 0;
}