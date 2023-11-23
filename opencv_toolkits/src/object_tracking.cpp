#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>

class Node{
    private:
        ros::NodeHandle nh;
        cv::Ptr<cv::Tracker> tracker_ptr;

        cv::Ptr<cv::Tracker> init_tracker(std::string const method_){
            enum method_code{
                CSRT,
                BOOST,
                GOTURN,
                KCF,
                MIL,
                MOSSE,
                VIT,
                TLD,
                MF,
            };

            int code = 999;
            if (method_ == "csrt") code = method_code::CSRT;
            if (method_ == "boost") code = method_code::BOOST;
            if (method_ == "goturn") code = method_code::GOTURN;
            if (method_ == "kcf") code = method_code::KCF;
            if (method_ == "mf") code = method_code::MF;
            if (method_ == "mil") code = method_code::MIL;
            if (method_ == "vit") code = method_code::VIT;
            if (method_ == "mosse") code = method_code::MOSSE;
            if (method_ == "tld") code = method_code::TLD;

            switch(code){
                case method_code::CSRT:
                    return cv::TrackerCSRT::create();
                case method_code::BOOST:
                    return cv::TrackerBoosting::create();
                case method_code::GOTURN:
                    return cv::TrackerGOTURN::create();
                case method_code::KCF:
                    return cv::TrackerKCF::create();
                case method_code::MF:
                    return cv::TrackerMedianFlow::create();
                case method_code::MIL:
                    return cv::TrackerMIL::create();
                case method_code::MOSSE:
                    return cv::TrackerMOSSE::create();
                case method_code::TLD:
                    return cv::TrackerTLD::create();
                default:
                    ROS_ERROR("invalid tracking method, shutdown node");
                    ros::shutdown();
            }
        }

    public:
        Node():
            nh("~")
        {
            cv::VideoCapture cap;

            std::string tracking_method;
            if(nh.getParam("tracking_method", tracking_method)){
                tracker_ptr = init_tracker(tracking_method);
            }
            else{
                ROS_ERROR("need to specify one method of tracking, shutdown now");
                ros::shutdown();
            }
            
            std::string video_source;
            if(nh.getParam("video_source", video_source)){
                cap.open(video_source);
                if(!cap.isOpened()){
                    ROS_ERROR("Unable to open video source, shutdown node");
                    ros::shutdown();
                }
            }
            else{
                ROS_ERROR("need to specify one video for tracking, shutdown now");
                ros::shutdown();
            }
            
            cv::namedWindow("show");
            cv::Mat frame;
            ROS_INFO("press S to select roi, press ESC to shutdown");
            while(cap.read(frame) && ros::ok()){
                if(frame.empty()){
                    ROS_ERROR("blank frame grabbed, shutdown node");
                    cap.release();
                    cv::destroyAllWindows();
                    ros::shutdown();
                }
                
                cv::resize(frame, frame, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
                char key = cv::waitKey(50); // 100 ms
                if(key == 27){
                    // Esc
                    ROS_WARN("shutdown node");
                    cap.release();
                    cv::destroyAllWindows();
                    ros::shutdown();
                }
                else if(key == 83){
                    // S
                    cv::Rect roi = cv::selectROI("show", frame);
                    if(roi.width == 0 || roi.height == 0){
                        continue;
                    }
                    else{
                        if(!tracker_ptr->init(frame, roi)){
                            ROS_ERROR("Error when init tracker roi, shutdown now");
                            cap.release();
                            cv::destroyAllWindows();
                            ros::shutdown();
                        }
                    }
                }

                cv::Rect2d bb;
                cv::Mat img = frame;
                if(tracker_ptr->update(frame, bb)){
                    cv::rectangle(img, bb, cv::Scalar(255, 0, 0), 2, 1);
                }
                cv::imshow("show", img);
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "object_tracting");
    Node node;
    ros::spin();
    return 0;
}