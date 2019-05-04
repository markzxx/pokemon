#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"
ros::Publisher cmdVelPub;
/*
0: forward
1: backward



*/
bool stop=false;
int threshold=10;
void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());//使机器人停止运动
  ROS_INFO("move_turtle_goforward ended!");
  ros::shutdown();
}

void run(const std_msgs::Int32 msg){

	geometry_msgs::Twist speed; // 控制信号载体 Twist message
	
	if(msg.data==0){
		 speed.linear.x = 0;
		 speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
	}
	else if(msg.data==1){
		 speed.linear.x = 0.1;
		 speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
	}
	else if(msg.data==2){

		speed.linear.x = -0.1;
		speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
	}
	else if(msg.data==3){
	speed.linear.x = 0.1;
		 speed.angular.z = 0.1; // 设置角速度为0rad/s，正为左转，负为右转
	}
	else if(msg.data==4){
		speed.linear.x = 0.1;
		speed.angular.z = -0.1; // 设置角速度为0rad/s，正为左转，负为右转
	}
    cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "move_turtle_goforward");//初始化ROS,它允许ROS通过命令行进行名称重映射
  ros::NodeHandle node;//为这个进程的节点创建一个句柄
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);//在/mobile_base/commands/velocity topic上发布一个geometry_msgs/Twist的消息

 	ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, run);

  //ros::Rate loopRate(500);//ros::Rate对象可以允许你指定自循环的频率
  //signal(SIGINT, shutdown);
  ROS_INFO("move_turtle_goforward cpp start...");
	ros::Rate loopRate(10);//ros::Rate对象可以允许你指定自循环的频率
  //geometry_msgs::Twist speed; // 控制信号载体 Twist message
  int time=0;
	
  while (ros::ok())
  {
		ros::spin();

 		loopRate.sleep();//休眠直到一个频率周期的时间
/*
		speed.linear.x = 0.1; // 设置线速度为0.1m/s，正为前进，负为后退
		speed.angular.z = 0; // 设置角速度为0rad/s，正为左转，负为右转
		cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
		loopRate.sleep();//休眠直到一个频率周期的时间
		time++;
*/
	//if(time>500)
		//break;
  }
  return 0;
}

