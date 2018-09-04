#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "opencv2/opencv.hpp"
#include "track_helper.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dot_tracker");

    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("dot_marker_pose", 100);

    string video_filename = "/home/ziri/Downloads/XDO5O.mp4";   //hybrid_test_video / circular_test_video
    VideoCapture vid_cap (video_filename);
    // VideoCapture vid_cap(1);

    if(!vid_cap.isOpened())
    {
        ROS_INFO("%s", "Cannot open video"); 
    }

    TrackHelper track_helper("/home/ziri/catkin_ws/src/dot_tracking_ros/config/Settings.xml");

    cv::namedWindow("marker tracking");
    while (ros::ok())
    {
        Mat img, img_track, PoseMat;
        if (!vid_cap.read(img))
            break;

        //resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR); for re-scale video
        track_helper.process(img, img_track, PoseMat);
        // cout << "PoseMat = "<< endl << " "  << PoseMat << endl << endl;

        imshow("marker tracking", img_track);
        char key = waitKey(5);
        if (key == 27)
            break;

        // Convert to PoseStamped msg
        ros::Time curr_stamp(ros::Time::now());
        tf::Matrix3x3 tf_rot(PoseMat.at<double>(0,0), PoseMat.at<double>(0,1), PoseMat.at<double>(0,2),
                         PoseMat.at<double>(1,0), PoseMat.at<double>(1,1), PoseMat.at<double>(1,2),
                         PoseMat.at<double>(2,0), PoseMat.at<double>(2,1), PoseMat.at<double>(2,2));
        tf::Vector3 tf_pos(PoseMat.at<double>(0,3), PoseMat.at<double>(1,3), PoseMat.at<double>(2,3));
        tf::Transform transform = tf::Transform(tf_rot, tf_pos);

        geometry_msgs::PoseStamped poseMsg;
        tf::poseTFToMsg(transform, poseMsg.pose);
        poseMsg.header.frame_id = "dot_marker_frame";
        poseMsg.header.stamp = curr_stamp;
        pose_pub.publish(poseMsg);
    }

    // // Test code on video 1 input
    // Mat edges;
    // namedWindow("edges",1);
    // for(;;)
    // {
    //     Mat frame;
    //     vid_cap >> frame; // get a new frame from camera
    //     cvtColor(frame, edges, CV_BGR2GRAY);
    //     GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
    //     Canny(edges, edges, 0, 30, 3);
    //     imshow("edges", edges);
    //     if(waitKey(30) >= 0) break;
    // }

    vid_cap.release();


    return 0;
}