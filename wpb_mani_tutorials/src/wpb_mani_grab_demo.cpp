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
#include <geometry_msgs/Pose.h>
#include <wpb_mani_behaviors/Coord.h>

static ros::Publisher grab_box_pub;
static geometry_msgs::Pose grab_box_msg;

#define STEP_READY                     0
#define STEP_BOX_DETECT       1
#define STEP_GRAB_BOX            2
#define STEP_DONE                       3
int step = STEP_READY;

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

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "done")
    {
        ROS_INFO("[GrabResultCB] grab_box done!");
        step = STEP_DONE;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_grab_demo");

    ros::NodeHandle n;
    ros::Publisher behaviors_pub = n.advertise<std_msgs::String>("/wpb_mani/behaviors", 10);
    grab_box_pub = n.advertise<geometry_msgs::Pose>("/wpb_mani/grab_box", 10);
    ros::Subscriber box_result_sub = n.subscribe("/wpb_mani/boxes_3d", 10 , BoxCoordCB);
    ros::Subscriber res_sub = n.subscribe("/wpb_mani/grab_result", 10, GrabResultCB);

    sleep(1);

    std_msgs::String behavior_msg;
    behavior_msg.data = "box_detect start";
    behaviors_pub.publish(behavior_msg);
    step = STEP_BOX_DETECT;

    ros::spin();
    return 0;
}
