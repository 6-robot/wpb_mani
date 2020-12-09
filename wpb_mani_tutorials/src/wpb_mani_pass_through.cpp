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
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <dynamic_reconfigure/server.h>
#include "wpb_mani_tutorials/pc_pass_throughConfig.h"

// 默认检测范围
static float filter_x_max = 1.5;
static float filter_x_min = 0.3;
static float filter_y_max = 0.75;
static float filter_y_min = -0.75;
static float filter_z_max = 0.5;
static float filter_z_min = 0.1;

static tf::TransformListener *tf_listener; 
static ros::Publisher segmented_pub;

void PointcloudCB(const sensor_msgs::PointCloud2 &input)
{
    // 将点云数值从相机坐标系转换到机器人坐标系
    bool result = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(1.0)); 
    if(result == false)
    {
        return;
    }
    sensor_msgs::PointCloud2 pc_footprint;
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //将点云数据从ROS格式转换到PCL格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
   
    // 截取z轴方向
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (filter_x_min, filter_x_max);
    pass.filter (cloud_src);
    // 截取y轴方向
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (filter_y_min, filter_y_max);
    pass.filter (cloud_src);
    // 截取z轴方向
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (filter_z_min, filter_z_max);
    pass.filter (cloud_src);

    segmented_pub.publish(cloud_src);
 } 

void PassThroughCfg(wpb_mani_tutorials::pc_pass_throughConfig &config)
{
    filter_x_max = config.x_max;
    filter_x_min =  config.x_min;
    filter_y_max =  config.y_max;
    filter_y_min = config.y_min;
    filter_z_max = config.z_max;
    filter_z_min =  config.z_min;
    ROS_WARN("Reconfigure Request  x ( %.2f  %.2f)",filter_x_max,filter_x_min);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_mani_pass_through");
    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/points2", 10 , PointcloudCB);
    segmented_pub = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_pc",10);
    dynamic_reconfigure::Server<wpb_mani_tutorials::pc_pass_throughConfig> server;
    dynamic_reconfigure::Server<wpb_mani_tutorials::pc_pass_throughConfig>::CallbackType func;
  
    func = boost::bind(&PassThroughCfg, _1); //绑定回调函数
    server.setCallback(func); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况

    ros::spin();

    delete tf_listener; 

    return 0;
}