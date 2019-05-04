#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>

using namespace cv;
using namespace std;


class Mover
{
  ros::NodeHandle nh_;
  ros::Subscriber search_sub_;

public:
  Mover(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    search_sub_ = it_.subscribe("/pokemon_go/searcher", 1, &Mover::moving, this);
  }

  ~Mover()
  {
  }

  void moving(std_msgs::Float32MultiArray &rect)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an target square on the video stream
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    cv::rectangle(cv_ptr->image, cv::Point(width/3, height/6), cv::Point(2*width/3, 5*height/6), CV_RGB(255,0,0));
	
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
	
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  float dis(Point p1, Point p2){
	  return sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2));
  }
  
  bool cross(Rect  r1, Rect  r2){
	return (r1 & r2).area() > 0;
  }
  
  void mergeRec(int id, vector<int> &merge, vector<Rect> &rectangles){
	for(int i=0;i<merge.size();i++){
		rectangles[id] = rectangles[id] | rectangles[merge[i]];
	}
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pokemon_searching");
  Mover ic;
  ros::spin();
  return 0;
}
