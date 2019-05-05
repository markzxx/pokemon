#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 1000);
    ros::Rate loop_rate(10);
 
    int count = 1;
	int count1=0;
    while (ros::ok())
    {
        std_msgs::Int32 msg;
	msg.data=count;
        ROS_INFO("%d", msg.data);
 
        /**
         * 向 Topic: chatter 发送消息, 发送频率为10Hz（1秒发10次）；消息池最大容量1000。
         */
        chatter_pub.publish(msg);
 	++count1;
	if(count1==50)
		count=2;
	if(count1==100)
		count=0;
        loop_rate.sleep();
        
    }
    return 0;
}

