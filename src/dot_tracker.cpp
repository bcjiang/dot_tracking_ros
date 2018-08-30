#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>
#include "track_helper.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dot_tracker");

	ros::NodeHandle nh;

	ros::Publisher pose_pub = nh.advertise<std_msgs::String>("dot_marker_pose", 100);

	std_msgs::String msg;
    msg.data = "hello world: dot_marker_pose" ;

	string video_filename = "/home/ziri/Downloads/XDO5O.mp4";	//hybrid_test_video / circular_test_video
    VideoCapture vid_cap (video_filename);

	if(!vid_cap.isOpened())
	{
		ROS_INFO("%s", "Cannot open video"); 
	}

	TrackHelper track_helper("/home/ziri/catkin_ws/src/dot_tracking_ros/config/Settings.xml");

	cv::namedWindow("marker tracking");
	while (ros::ok())
	{
		Mat img, img_track;
		if (!vid_cap.read(img))
			break;

		//resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);	for re-scale video
		track_helper.process(img, img_track);

		imshow("marker tracking", img_track);
		char key = waitKey(5);
		if (key == 27)
			break;

		pose_pub.publish(msg);
	}

	vid_cap.release();


    return 0;
}