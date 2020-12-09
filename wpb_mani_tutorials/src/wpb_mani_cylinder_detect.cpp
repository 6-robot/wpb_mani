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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

static tf::TransformListener *tf_listener; 

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

    // 将点云数据从ROS格式转换到PCL格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
   
    // 对设定范围内的点云进行截取
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // 截取x轴方向，前方0.5米到2.0米内的点云
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.5, 2.0);
    pass.filter (cloud_src);
    // 截取z轴方向，高度0.2米到0.5米内的点云
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 0.5);
    pass.filter (cloud_src);

   // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;    //法线估计对象
	ne.setInputCloud(cloud_src.makeShared());       //将截取后的点云作为输入
	ne.setSearchMethod(tree);                                          //设置内部算法实现时所用的搜索对象，tree为指向kdtree或者octree对应的指针
	ne.setKSearch(50);                                                          //设置K近邻搜索时所用的K参数
	ne.compute(*cloud_normals);                                  //计算法线向量特征值

    // 定义模型分类器
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentation;
    segmentation.setInputCloud(cloud_src.makeShared());          //将截取后的点云作为输入
    segmentation.setInputNormals(cloud_normals);                        //输入前面计算法线向量特征值
    segmentation.setOptimizeCoefficients(true);        					       //设置对估计的模型系数需要进行优化
	segmentation.setModelType(pcl::SACMODEL_CYLINDER);      //设置分割模型为圆柱型
	segmentation.setMethodType(pcl::SAC_RANSAC);                      //设置采用RANSAC作为算法的参数估计方法
	segmentation.setNormalDistanceWeight(0.1);                              //设置表面法线权重系数
	segmentation.setMaxIterations(10000);                                           //设置迭代的最大次数10000
	segmentation.setDistanceThreshold(0.15);                                    //设置内点到模型的距离允许最大值
	segmentation.setRadiusLimits(0, 0.1);                                               //设置估计的圆柱模型的半径范围

    // 使用模型分类器进行检测
    pcl::PointIndices::Ptr cylinderIndices(new pcl::PointIndices);
    segmentation.segment (*cylinderIndices, *coefficients);

    // 统计圆柱点集的中心坐标
    int point_num =  cylinderIndices->indices.size();
    float points_x_sum = 0;
    float points_y_sum = 0;
    float points_z_sum = 0;
    for(int i=0;i<point_num;i++)
    {
        int point_index = cylinderIndices->indices[i];
        points_x_sum += cloud_src.points[point_index].x;
        points_y_sum += cloud_src.points[point_index].y;
        points_z_sum += cloud_src.points[point_index].z;
    }
    float cylinder_x = points_x_sum/point_num;
    float cylinder_y = points_y_sum/point_num;
    float cylinder_z = points_z_sum/point_num;
    ROS_INFO("cylinder pos = ( %.2f , %.2f , %.2f)" , cylinder_x,cylinder_y,cylinder_z);
 } 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_mani_cylinder_detect");
    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/points2", 10 , PointcloudCB);

    ros::spin();

    delete tf_listener; 

    return 0;

}