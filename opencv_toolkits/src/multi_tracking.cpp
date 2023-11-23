#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <vision_msgs/BoundingBox2D.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>

class Node{
    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::BoundingBox2DArray> ApproxSyncPolicy;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, vision_msgs::BoundingBox2DArray> ExactSyncPolicy;
        
        ros::NodeHandle nh;
        ros::Timer timer;
        ros::Publisher image_pub;
        cv::MultiTracker multi_tracker;
        std::string tracking_method;
        int sync_queuesize = 10;
        bool exact_time = true;
        cv::Mat img;
        std::vector<vision_msgs::BoundingBox2D> bboxs;

        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<vision_msgs::BoundingBox2DArray> bbox_sub;

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

        cv::Rect msg_to_rect(const vision_msgs::BoundingBox2D &bbox_){
            float cx = bbox_.center.x;
            float cy = bbox_.center.y;
            float left_up_x = cx - bbox_.size_x;
            float left_up_y = cy - bbox_.size_y;
            return cv::Rect(left_up_x, left_up_y, bbox_.size_x, bbox_.size_y);
        }

        void sync_callback(const sensor_msgs::ImageConstPtr &image_msg, const vision_msgs::BoundingBox2DArray::ConstPtr &bbox_msg){
            // deal with image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC1);
            img = cv_ptr->image;
            for(auto &bbox : bbox_msg->boxes){
                bboxs.emplace_back(bbox);
            }
            cv::Mat frame = img;

            if(!multi_tracker.empty()){
                if(multi_tracker.update(img)){
                
                }
                else{
                    ROS_ERROR("multi_tracker update return false, shutdown now");
                    ros::shutdown();
                }
                // draw the tracked object
                for(unsigned i=0; i<multi_tracker.getObjects().size(); i++)
                    cv::rectangle(frame, multi_tracker.getObjects()[i], cv::Scalar(255, 0, 0), 2, 1);
            }
            
            cv_bridge::CvImage out_msg;
            out_msg.header = image_msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image = frame;
            image_pub.publish(out_msg.toImageMsg());
        }

        void timer_callback(const ros::TimerEvent& event){
            // a timer to init bbox, renew multi_tracker
            multi_tracker.clear();
            std::vector<cv::Ptr<cv::Tracker>> algorithms;
            std::vector<cv::Rect2d> objects;
            for (size_t i = 0; i < bboxs.size(); i++)
            {
                algorithms.push_back(init_tracker(tracking_method));
                objects.push_back(msg_to_rect(bboxs[i]));
            }
            if(!multi_tracker.add(algorithms, img, objects)){
                ROS_ERROR("multi_tracker add return false, shutdown now");
                ros::shutdown();
            }
        }

    public:
        Node():
            nh("~")
        {
            if(nh.getParam("tracking_method", tracking_method)){
                ROS_INFO("get param: tracking method");
            }
            else{
                ROS_ERROR("need to specify one method of tracking, shutdown now");
                ros::shutdown();
            }
            nh.param<bool>("exact_time", exact_time, true);
            nh.param<int>("sync_queuesize", sync_queuesize, 10);
            int duration_to_update = 5; // five second
            nh.param<int>("update_duration", duration_to_update, 5);

            timer = nh.createTimer(ros::Duration(duration_to_update), &Node::timer_callback, this);
            image_sub.subscribe(nh, "/colorimage_topic", 1);
            bbox_sub.subscribe(nh, "/boundingbox_topic", 1);
            if(exact_time){
                static message_filters::Synchronizer<ExactSyncPolicy> sync(ExactSyncPolicy(sync_queuesize), image_sub, bbox_sub);
                sync.registerCallback(boost::bind(&Node::sync_callback, this, _1, _2));
            }
            else{
                static message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(sync_queuesize), image_sub, bbox_sub);
                sync.registerCallback(boost::bind(&Node::sync_callback, this,_1, _2));
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "multi_tracting");
    Node node;
    ros::spin();
    return 0;
}