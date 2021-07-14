
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <wpb_mani_behaviors/Coord.h>

static int grab_count = 2;				//抓取次数

static ros::Publisher grab_box_pub;
static geometry_msgs::Pose grab_box_msg;
static ros::Publisher plane_height_pub;
static int nCooldown = 0;

#define STEP_READY                     0
#define STEP_BOX_DETECT       1
#define STEP_GRAB_BOX            2
#define STEP_COOLDOWN          3
#define STEP_DONE                       4
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
		if(box_num >grab_box_index )
		{
			grab_box_msg.position.x = msg->x[grab_box_index];
			grab_box_msg.position.y = msg->y[grab_box_index];
			grab_box_msg.position.z = msg->z[grab_box_index];
			grab_box_pub.publish(grab_box_msg);
			step = STEP_GRAB_BOX;
		}
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "done")
    {
        if(step == STEP_GRAB_BOX)
        {
            ROS_INFO("[GrabResultCB] grab_box done!");
            grab_count --;
            if(grab_count  > 0)
            {
                nCooldown = 20;     // 延时两秒等技能冷却
                step = STEP_COOLDOWN;
            }
            else
            {
                step = STEP_DONE;
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_mani_grab_again");

    ros::NodeHandle n;
    plane_height_pub = n.advertise<std_msgs::Float64>("/wpb_mani/plane_height", 10);
    grab_box_pub = n.advertise<geometry_msgs::Pose>("/wpb_mani/grab_box", 10);
    ros::Subscriber box_result_sub = n.subscribe("/wpb_mani/boxes_3d", 10 , BoxCoordCB);
    ros::Subscriber res_sub = n.subscribe("/wpb_mani/grab_result", 10, GrabResultCB);

    sleep(1);
    
    std_msgs::Float64 plane_height_msg;
    plane_height_msg.data = 0.22;        //载物台高度(单位:米)
    plane_height_pub.publish(plane_height_msg);
    step = STEP_BOX_DETECT;

    ros::Rate r(10);
    while(ros::ok())
    {
        if(step == STEP_COOLDOWN)
        {
            nCooldown --;
            if(nCooldown <= 0)
            {
                // 再次激活 “检测 - 抓取”流程
                std_msgs::Float64 plane_height_msg;
                plane_height_msg.data = 0.22;        //载物台高度(单位:米)
                plane_height_pub.publish(plane_height_msg);
                step = STEP_BOX_DETECT;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
