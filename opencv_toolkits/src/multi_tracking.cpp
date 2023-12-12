#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/PoseArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

class Node{
    private:
        struct CameraInfo{
            cv::Mat intrisicMat = cv::Mat::ones(3, 3, cv::DataType<float>::type);
            cv::Mat distCoeffs = cv::Mat::ones(5, 1, cv::DataType<float>::type);
            int width;
            int height;
        };

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, vision_msgs::BoundingBox2DArray> ApproxSyncPolicy;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, vision_msgs::BoundingBox2DArray> ExactSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseArray> ApproxSyncPolicy_speed;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseArray> ExactSyncPolicy_speed;

        ros::NodeHandle nh;
        tf::TransformListener _tfListener;
        image_transport::ImageTransport it;
        ros::Timer timer;
        image_transport::Publisher image_pub;
        ros::Publisher speed_result_pub;
        ros::Publisher pose_array_pub;
        cv::Ptr<cv::MultiTracker> multi_tracker = cv::MultiTracker::create();
        std::string tracking_method;
        int sync_queuesize = 10;
        bool exact_time = true;
        cv_bridge::CvImagePtr imgptr;
        std::vector<vision_msgs::BoundingBox2D> bboxs;
        std::vector<std::pair<int, cv::Point>> central; // id, central pixel
        std::vector<cv::Point3d> points_last;
        int count = 0;
        bool cal_speed = false;
        CameraInfo cam_info;
        std::string reference_frame;

        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<vision_msgs::BoundingBox2DArray> bbox_sub;
        message_filters::Subscriber<geometry_msgs::PoseArray> pose_array_sub;

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
                    return NULL;
            }
        }

        cv::Rect msg_to_rect(const vision_msgs::BoundingBox2D &bbox_){
            float cx = bbox_.center.x;
            float cy = bbox_.center.y;
            float left_up_x = cx - bbox_.size_x;
            float left_up_y = cy - bbox_.size_y;
            return cv::Rect(bbox_.center.x, bbox_.center.y, bbox_.size_x, bbox_.size_y);
        }

        void sync_callback(const sensor_msgs::ImageConstPtr &image_msg, const vision_msgs::BoundingBox2DArray::ConstPtr &bbox_msg){
            count += 1;
            // deal with image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC3);
            imgptr = cv_ptr;
            bboxs.clear();
            for(auto &bbox : bbox_msg->boxes){
                bboxs.emplace_back(bbox);
            }
            cv::Mat frame = imgptr->image;

            if(!multi_tracker->empty()){
                if(multi_tracker->update(imgptr->image)){
                
                }
                else{
                    ROS_WARN("multi_tracker update return false");
                }
                // getObjects: Returns a reference to a storage for the tracked objects, each object corresponds to one tracker algorithm.
                // draw the tracked object
                for(unsigned int i=0; i < multi_tracker->getObjects().size(); i++){
                    cv::Rect2d bb = multi_tracker->getObjects()[i];
                    if(!(bb.width == 0 || bb.height == 0)){
                        cv::rectangle(frame, bb, cv::Scalar(255, 0, 0), 2, 1);
                        cv::putText(frame, std::to_string(i), cv::Point(bb.x, bb.y), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,255,0), 2, true);
                        central.emplace_back(std::make_pair(i, cv::Point(bb.x+bb.width/2, bb.y+bb.height/2)));
                    }
                }
            }

            geometry_msgs::PoseArray array_msg;
            array_msg.header = image_msg->header;
            array_msg.header.seq = count;
            for(auto &p: central){
                geometry_msgs::Pose pose_msg;
                pose_msg.position.x = p.second.x;
                pose_msg.position.y = p.second.y;
                array_msg.poses.emplace_back(pose_msg);
                pose_msg.orientation.x = p.first;
            }
            pose_array_pub.publish(array_msg);

            cv_bridge::CvImage out_msg;
            out_msg.header = image_msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = frame;
            image_pub.publish(out_msg.toImageMsg());
        }

        void speed_sync_callback(const sensor_msgs::ImageConstPtr &image_msg, const geometry_msgs::PoseArrayConstPtr &pose_array_msg){
            std::string camera_frame = image_msg->header.frame_id;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC3);
            cv::Mat image = cv_ptr->image;
            cv_bridge::CvImage out_msg;
            out_msg.header = image_msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            // use last frame to this frame and draw on this frame
            if(pose_array_msg->header.seq == 1){
                points_last.clear();
                for(auto &pose : pose_array_msg->poses){
                    cv::Point3d p_now(pose.position.x, pose.position.y, pose.position.z);
                    points_last.emplace_back(p_now);
                }
                out_msg.image = image;
                speed_result_pub.publish(out_msg.toImageMsg());
            }
            else{
                for(int i = 0; i < pose_array_msg->poses.size(); i++){
                    tf::StampedTransform transform;
                    cv::Point3d p_now((pose_array_msg->poses)[i].position.x, (pose_array_msg->poses)[i].position.y, (pose_array_msg->poses)[i].position.z);
                    if(getTransform(reference_frame, camera_frame, transform)){
                        std::vector<cv::Point2f> imagePoints;
                        std::vector<cv::Point3d> objectPoints;
                        cv::Mat rVec = cv::Mat::zeros(3, 1, cv::DataType<float>::type);
                        cv::Mat tVec = cv::Mat::zeros(3, 1, cv::DataType<float>::type);
                        objectPoints.emplace_back(p_now);
                        objectPoints.emplace_back(points_last[i]);
                        transform_to_rtvec(transform, rVec, tVec);
                        cv::projectPoints(objectPoints, rVec, tVec, cam_info.intrisicMat, cam_info.distCoeffs, imagePoints);
                        cv::arrowedLine(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), 2);
                    }
                    points_last.emplace_back(p_now);
                }
                out_msg.image = image;
                speed_result_pub.publish(out_msg.toImageMsg());
            }
        }

        void timer_callback(const ros::TimerEvent& event){
            // a timer to init bbox, renew multi_tracker
            multi_tracker = cv::MultiTracker::create();
            std::vector<cv::Ptr<cv::Tracker>> algorithms;
            std::vector<cv::Rect2d> objects;
            for (size_t i = 0; i < bboxs.size(); i++)
            {
                algorithms.push_back(init_tracker(tracking_method));
                objects.push_back(msg_to_rect(bboxs[i]));
            }
            if(imgptr == nullptr)
                return;
            if(!multi_tracker->add(algorithms, imgptr->image, objects)){
                ROS_ERROR("multi_tracker add return false, shutdown now");
                ros::shutdown();
            }
            count = 0;
        }

        bool get_once_cameraInfo(ros::NodeHandle &nh, CameraInfo &cam_info){
            ros::Duration five_seconds_timeout(5, 0);
            sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camerainfo_topic", nh, five_seconds_timeout);
            if(msg == NULL){
                return false;
            }
            else{
                cam_info.intrisicMat.at<float>(0, 0) = msg->K[0]; //fx
                cam_info.intrisicMat.at<float>(1, 0) = 0;
                cam_info.intrisicMat.at<float>(2, 0) = 0;
                cam_info.intrisicMat.at<float>(0, 1) = 0;
                cam_info.intrisicMat.at<float>(1, 1) = msg->K[4]; //fy
                cam_info.intrisicMat.at<float>(2, 1) = 0;
                cam_info.intrisicMat.at<float>(0, 2) = msg->K[2]; //cx
                cam_info.intrisicMat.at<float>(1, 1) = msg->K[5]; //cy
                cam_info.intrisicMat.at<float>(2, 1) = 1;
                cam_info.width = msg->width;
                cam_info.height = msg->height;
                cam_info.distCoeffs.at<float>(0) = msg->D[0];
                cam_info.distCoeffs.at<float>(1) = msg->D[1];
                cam_info.distCoeffs.at<float>(2) = msg->D[2];
                cam_info.distCoeffs.at<float>(3) = msg->D[3];
                cam_info.distCoeffs.at<float>(4) = msg->D[4];
                return true;
            }
        }

        void transform_to_rtvec(tf::Transform& transform, cv::Mat& rvec, cv::Mat& tvec){
            tvec.at<float>(0) = transform.getOrigin().getX();
            tvec.at<float>(1) = transform.getOrigin().getY();
            tvec.at<float>(2) = transform.getOrigin().getZ();
            float theta_2 = acos(transform.getRotation().getW());
            float x = transform.getRotation().getX()/sin(theta_2);
            float y = transform.getRotation().getY()/sin(theta_2);
            float z = transform.getRotation().getZ()/sin(theta_2);
            rvec.at<float>(0) = x / theta_2 / 2;
            rvec.at<float>(1) = y / theta_2 / 2;
            rvec.at<float>(2) = z / theta_2 / 2;
        }

        cv::Mat convert_vector_to_rmat(cv::Point3d &p1, cv::Point3d &p2){
            cv::Vec3f vec(p2.x-p1.x, p2.y-p1.y, p2.z-p1.z);
            cv::normalize(vec, vec);
            cv::Vec3f x_axis(1, 0, 0);
            cv::Mat R = cv::Mat::zeros(3, 3, cv::DataType<float>::type);
            if(vec == -x_axis){
                R.at<float>(0, 0) = -1;
                R.at<float>(1, 1) = -1;
                R.at<float>(2, 2) = -1;
            }
            else{
                cv::Vec3f v_vec = x_axis.cross(vec);
                float cosine = x_axis.dot(vec);
                cv::Mat v_mat = cv::Mat::zeros(3, 3, cv::DataType<float>::type);
                v_mat.at<float>(0, 0) = 0;
                v_mat.at<float>(1, 0) = -v_vec[2];
                v_mat.at<float>(2, 0) = v_vec[1];
                v_mat.at<float>(0, 1) = v_vec[2];
                v_mat.at<float>(1, 1) = 0;
                v_mat.at<float>(2, 1) = -v_vec[0];
                v_mat.at<float>(0, 2) = -v_vec[1];
                v_mat.at<float>(1, 1) = v_vec[0];
                v_mat.at<float>(2, 1) = 0;
                cv::Mat I = cv::Mat::eye(3, 3, cv::DataType<float>::type);
                cv::Mat v_mat_2;
                pow(v_mat, 2, v_mat_2);
                R = I + v_mat + v_mat_2 * (1 / (1 + cosine));
            }
            return R;
        }

        bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform){
            std::string errMsg;
            if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg)){
                ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
                return false;
            }
            else{
                try
                {
                    _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
                }
                catch (const tf::TransformException& e)
                {
                    ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame << ": " << e.what());
                    return false;
                }

            }
            return true;
        }
    public:
        Node():
            nh("~"), it(nh)
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
            nh.param<bool>("calculate_speed", cal_speed, false);

            timer = nh.createTimer(ros::Duration(duration_to_update), &Node::timer_callback, this);
            image_sub.subscribe(nh, "/colorimage_topic", 1);
            bbox_sub.subscribe(nh, "/boundingbox_topic", 1);
            pose_array_sub.subscribe(nh, "/posearray_topic", 1);
            if(exact_time){
                static message_filters::Synchronizer<ExactSyncPolicy> sync(ExactSyncPolicy(sync_queuesize), image_sub, bbox_sub);
                sync.registerCallback(boost::bind(&Node::sync_callback, this, _1, _2));
                if(cal_speed){
                    static message_filters::Synchronizer<ExactSyncPolicy_speed> speed_sync(ExactSyncPolicy_speed(sync_queuesize), image_sub, pose_array_sub);
                    speed_sync.registerCallback(boost::bind(&Node::speed_sync_callback, this, _1, _2));
                }
            }
            else{
                static message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(sync_queuesize), image_sub, bbox_sub);
                sync.registerCallback(boost::bind(&Node::sync_callback, this,_1, _2));
                if(cal_speed){
                    static message_filters::Synchronizer<ApproxSyncPolicy_speed> speed_sync(ApproxSyncPolicy_speed(sync_queuesize), image_sub, pose_array_sub);
                    speed_sync.registerCallback(boost::bind(&Node::speed_sync_callback, this, _1, _2));
                }
            }
            image_pub = it.advertise("tracking_result", 1);
            pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("2d_points", 1);
            speed_result_pub = nh.advertise<sensor_msgs::Image>("speed_result", 1);

            if(!get_once_cameraInfo(nh, cam_info)){
                ROS_ERROR("multi_tracking: did not subscribe to camera info");
                ros::shutdown();
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "multi_tracting");
    Node node;
    ros::spin();
    return 0;
}