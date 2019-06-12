#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <apriltags/AprilTagDetections.h>

using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Pokemon Search";
static const std::string GREY_WINDOW = "灰度";
static const std::string CANNY_WINDOW = "canny";
Mat img;
int fileNum = 1;
int zeroCount = 0;
bool flag = false;
bool listen_tag = true;
map<int, geometry_msgs::Pose> tagId;

class Searcher
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher image_pub_;
	ros::Subscriber save_sub_;
	ros::Publisher tag_pub_;
	ros::Subscriber tag_sub_;


public:
	Searcher()
			: it_(nh_) {
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Searcher::imageCb, this);
		save_sub_ = nh_.subscribe("/pokemon_go/save", 1, &Searcher::saveImg, this);
		image_pub_ = nh_.advertise<std_msgs::Int32>("/pokemon_go/searcher", 1);
        tag_pub_ = nh_.advertise<geometry_msgs::Pose>("/apriltag_pose", 1);
		tag_sub_ = nh_.subscribe("/apriltags/detections", 1, &Searcher::collect_tag, this);

		cv::namedWindow(OPENCV_WINDOW);
//		cv::namedWindow(GREY_WINDOW);
//	cv::namedWindow(CANNY_WINDOW);
	}

	~Searcher() {
		cv::destroyWindow(OPENCV_WINDOW);
//		cv::destroyWindow(GREY_WINDOW);
	}

	void collect_tag(const apriltags::AprilTagDetections &apriltags) {
		for (auto atg : apriltags.detections) {
            tagId[atg.id] = atg.pose;
		}
	}

	void saveImg(std_msgs::Bool save) {
        for (auto it : tagId) {
            ROS_ERROR("id:%d x:%f y:%f", it.first, it.second.position.x, it.second.position.y);
        }
//        stringstream stream;
//        stream << "/home/ubuntu/1001/pokemon" << fileNum << ".jpg";
//        imwrite(stream.str(), img);
//        cout << "pokemon" << fileNum << " had Saved." << endl;
//        fileNum++;
//        listen_tag = true;
	}

	void imageCb(const sensor_msgs::ImageConstPtr &msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		// 传递出去的信息，外框和内框共16个点

		std_msgs::Float32MultiArray rect;

		// Draw an target square on the video stream
		int height = cv_ptr->image.rows;
		int width = cv_ptr->image.cols;
		cv::rectangle(cv_ptr->image, cv::Point(width / 3, height / 6), cv::Point(2 * width / 3, 5 * height / 6),
					  CV_RGB(255, 0, 0));
		// 记录外面红框的4个顶点x y坐标
		rect.data.push_back(width / 3);
		rect.data.push_back(height / 6);
		rect.data.push_back(2 * width / 3);
		rect.data.push_back(height / 6);
		rect.data.push_back(2 * width / 3);
		rect.data.push_back(5 * height / 6);
		rect.data.push_back(width / 3);
		rect.data.push_back(5 * height / 6);

		//detect the white pokemon, 记录白框的四个顶点
		Rect r = detect(cv_ptr, rect, width, height);
		vector<int> dists;
		dists.push_back(r.tl().x - width / 3);//左
		dists.push_back(r.tl().y - height / 6);//上
		dists.push_back(2 * width / 3 - r.br().x);//右
		dists.push_back(5 * height / 6 - r.br().y);//下

//    cout<< dists[1] <<endl;


//	for (int i = 0; i < 3; i++) {
//		if (dists[i] < minDis.data) minDis.data = dists[i];
//	}

//	printf("p1x:%f p1y%f p3x:%f p3y:%f\n", rect.data[0], rect.data[1], rect.data[8],rect.data[9]);
		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
	}

	Rect detect(cv_bridge::CvImagePtr &cv_ptr, std_msgs::Float32MultiArray &rect, int weight, int height) {
		int iLowH = 0, iHighH = 180, iLowS = 0, iHighS = 40, iLowV = 200, iHighV = 255;
		img = cv_ptr->image;
		Mat imgHSV;
		vector <Mat> hsvSplit;
		cvtColor(img, imgHSV, COLOR_BGR2HSV);
		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);
		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),
				imgThresholded); //Threshold the image
		//开操作 (去除一些噪点)
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

		//闭操作 (连接一些连通域)
		morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);
		GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0, 0);
//		imshow(GREY_WINDOW, imgThresholded);
		Mat cannyImage;
		Canny(imgThresholded, cannyImage, 128, 255, 3);

		vector <vector<Point>> contours;
		vector <Vec4i> hierarchy;
		findContours(cannyImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		//绘制轮廓
		for (int i = 0; i < (int) contours.size(); i++) {
			drawContours(cannyImage, contours, i, Scalar(255), 1, 8);
		}
//	imshow(CANNY_WINDOW, cannyImage);                //用矩形圈出轮廓并返回位置坐标
		Point2f vertex[4];
		float max_s = 0;
		int max_idx = 0;
		vector <Rect> rectangles;
		vector<int> idx;
		for (int i = 0; i < contours.size(); i++) {
			vector <Point> points = contours[i];
			RotatedRect box = minAreaRect(Mat(points));
			//	Point2f tem_vertex[4];
			//	box.points(tem_vertex);
			rectangles.push_back(box.boundingRect());
			idx.push_back(i);
			// rectangle(img, rectangles[i].tl(), rectangles[i].br(), CV_RGB(255,0,255));
		}

		vector<int> idxNOT;
		for (int i = 0; i < idx.size(); i++) {
			Rect r = rectangles[idx[i]];
			if (r.br().y < height / 3) idxNOT.push_back(idx[i]);
			if (r.tl().y > 3 * height / 5) idxNOT.push_back(idx[i]);
		}

		vector<int> diff;
		std::set_difference(idx.begin(), idx.end(), idxNOT.begin(), idxNOT.end(),
							std::inserter(diff, diff.begin()));
		idx = diff;
		for (int i = 0; i < diff.size(); i++) {
			rectangle(img, rectangles[diff[i]].tl(), rectangles[diff[i]].br(), CV_RGB(255, 0, 255));
		}


		MERGE:
		for (int i = 0; i < idx.size(); i++) {
			int id1 = idx[i];
			vector<int> merge;
			for (int j = i + 1; j < idx.size(); j++) {
				int id2 = idx[j];
				if (id1 != id2 && cross(rectangles[id1], rectangles[id2])) {
					merge.push_back(id2);
				}
			}

			if (!merge.empty()) {
				for (int i = 0; i < merge.size(); i++) {
					idx.erase(find(idx.begin(), idx.end(), merge[i]));
				}
				mergeRec(id1, merge, rectangles);
				goto MERGE;
			}
		}


		for (int i = 0; i < idx.size(); i++) {
			Rect r = rectangles[idx[i]];
			float s = r.area();
			if (s > max_s) {
				max_s = s;
				max_idx = idx[i];
			}
			rectangle(img, r.tl(), r.br(), CV_RGB(0, 0, 255));
		}

		rectangle(img, rectangles[max_idx].tl(), rectangles[max_idx].br(), CV_RGB(255, 255, 0));

		return rectangles[max_idx];
//	namedWindow("绘制的最小矩形面积",WINDOW_NORMAL);

//	imshow("绘制的最小矩形面积",img);
	}

	float dis(Point p1, Point p2) {
		return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
	}

	bool cross(Rect r1, Rect r2) {
		int error = 30;
		return (r1 + Size(0, error) & r2).area() > 0 || (r1 & r2 + Size(0, error)).area() > 0;
	}

	void mergeRec(int id, vector<int> &merge, vector <Rect> &rectangles) {
		for (int i = 0; i < merge.size(); i++) {
			rectangles[id] = rectangles[id] | rectangles[merge[i]];
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pokemon_searching");
	Searcher ic;
	ros::spin();
	return 0;
}

