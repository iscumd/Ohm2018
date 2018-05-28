#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
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

int main()
{
    ros::init(argc, argv, "camera_turn_angle");
    ros::NodeHandle n;
    ros::Publisher camera_angles = n.advertise<ohm_igvc_msgs::TurnAngles>("camera_turn_angle", 1);

    // ROS Params
    std::string cam_device;
    n.param("device", cam_device, "/dev/video0");

    std::string camera_frame;
    n.param("camera_frame", camera_frame, "camera");

    int mask_threshold;
    n.param("intersection_threshold", mask_threshold, 0.5);

    bool debug;
    n.param("debug value", debug, false);

#pragma region start
    cv::Mat mask[NUM_MASKS], cropped_mask[NUM_MASKS];
    cv::Mat anding, input, hsv, binary_image, cropped_region;
    double mask_turn_angles[NUM_MASKS];
    double mask_size[NUM_MASKS];

    cv::Rect ROI = cv::Rect(230, 0, 820, 468);

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

    mask[0] = cv::imread("0.png", 0); // left most turn
    mask[1] = cv::imread("1.png", 0);
    mask[2] = cv::imread("2.png", 0);
    mask[3] = cv::imread("3.png", 0);
    mask[4] = cv::imread("4.png", 0);
    mask[5] = cv::imread("5.png", 0);
    mask[6] = cv::imread("6.png", 0);
    mask[7] = cv::imread("7.png", 0);
    mask[8] = cv::imread("8.png", 0);
    mask[9] = cv::imread("9.png", 0); // right most turn

    double mask_size[NUM_MASKS];
    for (size_t i = 0; i < NUM_MASKS; i++)
    {
        cropped_mask[i] = mask[i](ROI);
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

    cv::namedWindow("HSV", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("MASK", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("BINARY", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("WARPED", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("OVERLAP", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ORIGINAL", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("TRACKBARS", cv::WINDOW_AUTOSIZE);

    ohm_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    ohm_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    int low_H = 0, low_S = 0, low_V = 200;        // low limit for HSV slider
    int high_H = 180, high_S = 255, high_V = 255; // upper limit for HSV slider

    cv::createTrackbar("Low Hue", "TRACKBARS", &low_H, 180, on_low_H_thresh_trackbar);
    cv::createTrackbar("Low Sat", "TRACKBARS", &low_S, 255, on_low_S_thresh_trackbar);
    cv::createTrackbar("Low Val", "TRACKBARS", &low_V, 255, on_low_V_thresh_trackbar);
    cv::createTrackbar("High Hue", "TRACKBARS", &high_H, 180, on_high_H_thresh_trackbar);
    cv::createTrackbar("High Sat", "TRACKBARS", &high_S, 255, on_high_S_thresh_trackbar);
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

    std::vector<cv::Point2f> quad_pts{Q1, Q2, Q3, Q4};
    std::vector<cv::Point2f> squre_pts{R1, R2, R3, R4};

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
        cropped_region = transformed(ROI);
        cv::cvtColor(cropped_region, hsv, CV_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V, 0), cv::Scalar(high_H, high_S, high_V, 0), binary_image);

        cv::GaussianBlur(binary_image, binary_image, cv::Size(5, 5), 4.5);
        cv::erode(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), 3);
        cv::dilate(binary_image, binary_image, cv::Mat(), cv::Point(-1, -1), 3);

        msg.turn_angles.clear();

        for (size_t i = 0; i < NUM_MASKS; i++)
        {
            intersections = 0;

            cv::bitwise_and(binary_image, cropped_mask[i], anding);
            intersections = (cv::countNonZero(anding) / mask_size[i]) * 100;

            if (intersections < mask_threshhold)
            {
                msg.turn_angles.push_back(mask_turn_angles[i]);
            }

            if (debug)
            {
                //cv::waitKey(1);
                cv::imshow("ORIGINAL", input);
                cv::imshow("WARPED", cropped_region);
                cv::imshow("BINARY", binary_image);
                cv::imshow("HSV", hsv);
                cv::imshow("MASK", cropped_mask[i]);
                cv::imshow("OVERLAP", anding);
                std::cout << '[' << i << "]: " << intersections << '%' << std::endl;
            }
        }

        camera_angles.publish(msg); // publishing code goes here
    }
}
