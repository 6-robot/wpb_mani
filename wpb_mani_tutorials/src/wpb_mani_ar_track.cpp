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
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_ar_track");

    ros::NodeHandle n;
    
    ros::Rate r(1);
    
    tf::TransformListener tf_listener; 
    tf::StampedTransform ar_transform;

    sleep(1);
    while(ros::ok())
    {
        ros::Time time = ros::Time(0);
        try
        {
            tf_listener.waitForTransform("/base_footprint","/ar_marker_1",  time, ros::Duration(20.0) );
            tf_listener.lookupTransform("/base_footprint", "/ar_marker_1", time, ar_transform);
        }  
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            return 0;
        }
        float ar_x = ar_transform.getOrigin().x();
        float ar_y = ar_transform.getOrigin().y();
        float ar_z = ar_transform.getOrigin().z();
        printf("1号标记的坐标:x= %.2f y=%.2f z=%.2f\n", ar_x, ar_y, ar_z);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}