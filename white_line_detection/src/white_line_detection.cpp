#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ohm_igvc_msgs/TurnAngles.h>
#include <string>

#define NUM_MASKS 10

void on_low_H_thresh_trackbar(int, void *) {}
void on_high_H_thresh_trackbar(int, void *) {}
void on_low_S_thresh_trackbar(int, void *) {}
void on_high_S_thresh_trackbar(int, void *) {}
void on_low_V_thresh_trackbar(int, void *) {}
void on_high_V_thresh_trackbar(int, void *) {}

double getTurnAngles(cv::Point p1, cv::Point p2)
{
    double dist = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    double adjacent = p1.y - p2.y;
    double rads = acos(adjacent / dist);
    double theta = (rads * 180 / 3.14159);

    return theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_turn_angle");
    ros::NodeHandle n;
    ros::Publisher camera_angles = n.advertise<ohm_igvc_msgs::TurnAngles>("camera_turn_angle", 1);

	std::string mask_path = ros::package::getPath("white_line_detection") + std::string("/../data/masks/");

    // ROS Params
    std::string cam_device;
    n.param("device", cam_device, std::string("/dev/video0"));

    std::string camera_frame;
    n.param("camera_frame", camera_frame, std::string("camera"));

    double mask_threshold;
    n.param("intersection_threshold", mask_threshold, 0.5);

#pragma region start
    cv::Mat mask[NUM_MASKS];
    cv::Mat anding;
    cv::Mat input;
    cv::Mat hsv;
    double mask_turn_angles[NUM_MASKS];
    double mask_size[NUM_MASKS];

    cv::VideoCapture ohm_webcam(cam_device, cv::CAP_V4L); // video1 is external cam

    cv::Point pointArray[NUM_MASKS];
    cv::Point base_point = cv::Point2f(616.5, 553.5);
    pointArray[0] = cv::Point2f(91.5, 349.5);
    pointArray[1] = cv::Point2f(27.0, 193.5);
    pointArray[2] = cv::Point2f(22.5, 7.5);
    pointArray[3] = cv::Point2f(349.5, 7.5);
    pointArray[4] = cv::Point2f(604.5, 6.0);
    pointArray[5] = cv::Point2f(775.5, 9.0);
    pointArray[6] = cv::Point2f(960.0, 9.0);
    pointArray[7] = cv::Point2f(1266.0, 36.0);
    pointArray[8] = cv::Point2f(1263.0, 186.0);
    pointArray[9] = cv::Point2f(1245.0, 346.5);

    for (size_t i = 0; i < NUM_MASKS; i++)
    {
        mask_size[i] = cv::countNonZero(mask[i]);

        if (i < NUM_MASKS / 2)
        {
            mask_turn_angles[i] = -getTurnAngles(base_point, pointArray[i]);
        }
        else
        {
            mask_turn_angles[i] = getTurnAngles(base_point, pointArray[i]);
        }
    }

    mask[0] = cv::imread((mask_path + std::string("0.png").c_str()), 0); // left most turn
    mask[1] = cv::imread((mask_path + std::string("1.png").c_str()), 0);
    mask[2] = cv::imread((mask_path + std::string("2.png").c_str()), 0);
    mask[3] = cv::imread((mask_path + std::string("3.png").c_str()), 0);
    mask[4] = cv::imread((mask_path + std::string("4.png").c_str()), 0);
    mask[5] = cv::imread((mask_path + std::string("5.png").c_str()), 0);
    mask[6] = cv::imread((mask_path + std::string("6.png").c_str()), 0);
    mask[7] = cv::imread((mask_path + std::string("7.png").c_str()), 0);
    mask[8] = cv::imread((mask_path + std::string("8.png").c_str()), 0);
    mask[9] = cv::imread((mask_path + std::string("9.png").c_str()), 0); // right most turn

    cv::namedWindow("MASK", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ORIGINAL", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("TRACKBARS", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("WARPED", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("OVERLAP", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("HSV", cv::WINDOW_AUTOSIZE);

    ohm_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    ohm_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    int low_H = 0, low_S = 0, low_V = 200;        // low limit for HSV slider
    int high_H = 180, high_S = 255, high_V = 255; // upper limit for HSV slider

    cv::createTrackbar("Low Hue", "TRACKBARS", &low_H, 180, on_low_H_thresh_trackbar);
    cv::createTrackbar("High Hue", "TRACKBARS", &high_H, 180, on_high_H_thresh_trackbar);
    cv::createTrackbar("Low Sat", "TRACKBARS", &low_S, 255, on_low_S_thresh_trackbar);
    cv::createTrackbar("High Sat", "TRACKBARS", &high_S, 255, on_high_S_thresh_trackbar);
    cv::createTrackbar("Low Val", "TRACKBARS", &low_V, 255, on_low_V_thresh_trackbar);
    cv::createTrackbar("High Val", "TRACKBARS", &high_V, 255, on_high_V_thresh_trackbar);

    cv::Point Q1 = cv::Point2f(507, 236); //top left pixel coordinate
    cv::Point Q2 = cv::Point2f(755, 237); //top right
    cv::Point Q3 = cv::Point2f(802, 389); //bottom right
    cv::Point Q4 = cv::Point2f(458, 394); //bottom left

    double ratio = 1.3333; // width / height of the actual panel on the ground
    double cardH = sqrt((Q3.x - Q2.x) * (Q3.x - Q2.x) + (Q3.y - Q2.y) * (Q3.y - Q2.y));
    double cardW = ratio * cardH;

    cv::Rect R(Q1.x, Q1.y, cardW, cardH);

    cv::Point R1 = cv::Point2f(R.x, R.y);
    cv::Point R2 = cv::Point2f(R.x + R.width, R.y);
    cv::Point R3 = cv::Point2f(cv::Point2f(R.x + R.width, R.y + R.height));
    cv::Point R4 = cv::Point2f(cv::Point2f(R.x, R.y + R.height));

    std::vector<cv::Point2f> quad_pts;
    std::vector<cv::Point2f> squre_pts;

    quad_pts.push_back(Q1);
    quad_pts.push_back(Q2);
    quad_pts.push_back(Q3);
    quad_pts.push_back(Q4);

    squre_pts.push_back(R1);
    squre_pts.push_back(R2);
    squre_pts.push_back(R3);
    squre_pts.push_back(R4);
    cv::Mat transmtx = cv::getPerspectiveTransform(quad_pts, squre_pts);

    cv::Mat transformed = cv::Mat::zeros(ohm_webcam.get(CV_CAP_PROP_FRAME_HEIGHT), ohm_webcam.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3);

    double intersections;

#pragma endregion
    while (ros::ok())
    {
        ohm_igvc_msgs::TurnAngles msg; // will be member of ohm_igvc_msgs namespace
        ohm_webcam >> input;
        if (input.empty())
            break;

        msg.header.frame_id = camera_frame;
        msg.header.stamp = ros::Time::now();
        cv::warpPerspective(input, transformed, transmtx, transformed.size());

        cv::cvtColor(transformed, hsv, CV_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V, 0), cv::Scalar(high_H, high_S, high_V, 0), hsv);

        cv::erode(hsv, hsv, cv::Mat(), cv::Point(-1, -1), 3);
        cv::dilate(hsv, hsv, cv::Mat(), cv::Point(-1, -1), 3);
        imshow("HSV", hsv);

        msg.turn_angles.clear();

        for (size_t i = 0; i < NUM_MASKS; i++)
        {

            intersections = 0;
            cv::imshow("MASK", mask[i]);
            cv::bitwise_and(hsv, mask[i], anding);
            intersections = (cv::countNonZero(anding) / mask_size[i]) * 100;

            cv::imshow("OVERLAP", anding);
            if (intersections < mask_threshold)
            {
                msg.turn_angles.push_back(mask_turn_angles[i]);
            }
        }

        camera_angles.publish(msg); // publishing code goes here
    }
}
