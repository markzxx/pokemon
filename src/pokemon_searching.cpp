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
#include <visualization_msgs/MarkerArray.h>

using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Pokemon Search";
static const std::string GREY_WINDOW = "灰度";
static const std::string CANNY_WINDOW = "canny";
bool good = false;
Mat img;
int fileNum = 1;
int zeroCount = 0;
bool flag = false;
bool listen_tag = true;
vector<bool> tagTake(100);
map<int, geometry_msgs::Pose> tagMap;
map<int, double> tagNumric;
unsigned last_markers_count_ = 0;

class Searcher
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher image_pub_;
	ros::Subscriber save_sub_;
	ros::Publisher tag_pub_;
    ros::Subscriber camera_sub_;
	ros::Subscriber tag_sub_;
    ros::Publisher marker_array_publisher_;


public:
	Searcher()
			: it_(nh_) {
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Searcher::imageCb, this);
		save_sub_ = nh_.subscribe("/pokemon_go/save", 1, &Searcher::saveImg, this);
		image_pub_ = nh_.advertise<std_msgs::Int32>("/pokemon_go/searcher", 1);
        tag_pub_ = nh_.advertise<geometry_msgs::Pose>("/apriltag_pose", 1);
		tag_sub_ = nh_.subscribe("/apriltags/detections", 1, &Searcher::collect_tag, this);
//        camera_sub_ = nh_.subscribe("/apriltag_save", 1, &Searcher::screenShot, this);
        marker_array_publisher_ =
                nh_.advertise<visualization_msgs::MarkerArray>("tags", 5);
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
            tagMap[atg.id] = atg.pose;
            double tmp = atg.pose.orientation.x + atg.pose.orientation.y + atg.pose.orientation.z + atg.pose.orientation.w;
            // printf("id:%d x:%f y:%f z:%f w:%f sum:%f\n", atg.id, atg.pose.orientation.x, atg.pose.orientation.y,
            //        atg.pose.orientation.z, atg.pose.orientation.w, tmp);

            if (abs(tagNumric[atg.id] - tmp) > 0.1 && good && tagTake[atg.id] == false) {
                tagTake[atg.id] = true;
                tagNumric[atg.id] = tmp;
                autoSave();
            }
		}
        visualizeFrontiers();
    }

    void visualizeFrontiers() {
        ROS_DEBUG("visualising %lu tags", tagMap.size());
        visualization_msgs::MarkerArray markers_msg;
        std::vector <visualization_msgs::Marker> &markers = markers_msg.markers;
        visualization_msgs::Marker m;

//        m.header.frame_id = markers_msg.header.frame_id;
        m.header.stamp = ros::Time::now();
        m.ns = "tags";
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        m.color.r = 0;
        m.color.g = 0;
        m.color.b = 255;
        m.color.a = 255;
        // lives forever
        m.lifetime = ros::Duration(0);
        m.frame_locked = true;

        m.action = visualization_msgs::Marker::ADD;
        size_t id = 0;
        for (auto &tag_iter : tagMap) {
            m.type = visualization_msgs::Marker::POINTS;
            m.id = int(id);
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;

            markers.push_back(m);
            ++id;
            m.type = visualization_msgs::Marker::SPHERE;
            m.id = int(id);
            m.pose.position = tag_iter.second.position;
        }
        size_t current_markers_count = markers.size();

        // delete previous markers, which are now unused
        m.action = visualization_msgs::Marker::DELETE;
        for (; id < last_markers_count_; ++id) {
            m.id = int(id);
            markers.push_back(m);
        }

        last_markers_count_ = current_markers_count;
        marker_array_publisher_.publish(markers_msg);
    }

	void saveImg(std_msgs::Bool save) {

        auto it = *(tagMap.begin());
        ROS_ERROR("id:%d x:%f y:%f", it.first, it.second.position.x, it.second.position.y);
        tag_pub_.publish(it.second);


    }

    void autoSave() {
        stringstream stream;
        stream << "/home/jeremy/pokemon" << fileNum << ".jpg";
        imwrite(stream.str(), img);
        cout << "pokemon" << fileNum << " had Saved." << endl;
        fileNum++;
    }

	void screenShot(std_msgs::Bool a) {
        autoSave();
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

		//detect the white pokemon, 记录白框的四个顶点
        Rect r = detect(cv_ptr, width, height);
        // 78 185 252 240
        // 640 480 -> 320 240
        // cout << "x: " <<abs((r.br().x+r.tl().x)/2-320) << endl;
        // cout << "y: " <<abs((r.tl().y+r.br().y)/2-240) << endl;
        // cout << "width: " <<r.br().x-r.tl().x << endl;
        float rate = float((r.br().x-r.tl().x))/float((r.br().y-r.tl().y));
        // cout << "y_center: " <<(r.tl().y+r.br().y)/2 << endl;
        // printf("width:%d height%f x_center:%f y_center:%f\n", r.br().x-r.tl().x, r.br().y-r.tl().y, (r.br().x+r.tl().x)/2,(r.tl().y+r.br().y)/2);
		// printf("width:%d height%d\n", width, height);

//		if (rate<0.8) cout << rate<< endl;

		if (abs((r.br().x+r.tl().x)/2-320)<150 
			&& abs((r.tl().y+r.br().y))/2-240<150
			&& r.br().x-r.tl().x > 70
			&& rate < 0.8)
		{	good = true;
//			cout << "good:"<<rate<< endl;
		}
			
		else good = false;

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);
	}

    Rect detect(cv_bridge::CvImagePtr &cv_ptr, int weight, int height) {
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
        int error = 20;
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

