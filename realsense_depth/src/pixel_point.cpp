#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tuple>

class CamInfo{
	public:
		int width;
		int height;
		float ppx;
		float ppy;
		float fx;
		float fy;
		rs2_distortion model;
		float coeffs[5];
};

class Node{
    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PointStamped> ApproxSyncPolicy;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PointStamped> ExactSyncPolicy;
        
        std::string reference_frame = " ";
        bool exact_time = true;
        int sync_queuesize = 10;
        
        CamInfo rs_intrin_;
        ros::NodeHandle nh;
        ros::Publisher point_pub;

        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub;

        void rs2_deproject_pixel_to_point(std::tuple<float, float, float> &point, const CamInfo &intrin, const float pixel[2],const float &depth){
            assert(intrin.model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
            assert(intrin.model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
            //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

            float x = (pixel[0] - intrin.ppx) / intrin.fx;
            float y = (pixel[1] - intrin.ppy) / intrin.fy;
            if(intrin.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
            {
                float r2  = x*x + y*y;
                float f = 1 + intrin.coeffs[0]*r2 + intrin.coeffs[1]*r2*r2 + intrin.coeffs[4]*r2*r2*r2;
                float ux = x*f + 2*intrin.coeffs[2]*x*y + intrin.coeffs[3]*(r2 + 2*x*x);
                float uy = y*f + 2*intrin.coeffs[3]*x*y + intrin.coeffs[2]*(r2 + 2*y*y);
                x = ux;
                y = uy;
            }
            std::get<0>(point) = depth * x;
            std::get<1>(point) = depth * y;
            std::get<2>(point) = depth;
        }

        bool get_once_cameraInfo(ros::NodeHandle &nh, CamInfo &rs_intrin){
            ros::Duration five_seconds_timeout(5, 0);
            sensor_msgs::CameraInfo::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camerainfo_topic", nh, five_seconds_timeout);
            if(msg == NULL){
                return false;
            }
            else{
                rs_intrin.fx = msg->K[0]; //fx
                rs_intrin.fy = msg->K[4]; //fy
                rs_intrin.ppx = msg->K[2]; //cx
                rs_intrin.ppy = msg->K[5]; //cy
                rs_intrin.width = msg->width;
                rs_intrin.height = msg->height;
                rs_intrin.coeffs[0] = msg->D[0];
                rs_intrin.coeffs[1] = msg->D[1];
                rs_intrin.coeffs[2] = msg->D[2];
                rs_intrin.coeffs[3] = msg->D[3];
                rs_intrin.coeffs[4] = msg->D[4];
                rs_intrin.model = RS2_DISTORTION_BROWN_CONRADY;
                return true;
            }
        }

        void sync_callback(const sensor_msgs::ImageConstPtr &image_msg, const geometry_msgs::PointStamped::ConstPtr &pixel_msg){
            if(!point_pub.getNumSubscribers())
                return;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat depth_image = cv_ptr->image;
            float pixel_to_find[2];
            std::tuple<float, float, float> point_3d;
            pixel_to_find[0] = pixel_msg->point.x;
            pixel_to_find[1] = pixel_msg->point.y;
            float depth = depth_image.at<float>(pixel_to_find[1], pixel_to_find[0]);
            rs2_deproject_pixel_to_point(point_3d, rs_intrin_, pixel_to_find, depth);
            // ROS_INFO("x: %f, y: %f, z: %f", std::get<0>(point_3d), std::get<1>(point_3d), std::get<2>(point_3d));
            geometry_msgs::PointStamped ps;
            //change to milimeter
            ps.point.x = std::get<0>(point_3d) / 1000;
            ps.point.y = std::get<1>(point_3d) / 1000;
            ps.point.z = std::get<2>(point_3d) / 1000;
            // init header
            ps.header = image_msg->header;
            ps.header.frame_id = pixel_msg->header.frame_id;

            geometry_msgs::PointStamped point_stamped_out;
            // do transform
            if(reference_frame != ps.header.frame_id){
                try{
                    static tf::TransformListener listener(ros::Duration(10));
                    listener.transformPoint(reference_frame, ps, point_stamped_out);
                }
                catch (const tf::TransformException& e){
                    ROS_ERROR_STREAM("Error in transform of " << ps.header.frame_id << " in " << reference_frame);
                    ros::shutdown();
                }
            }
            else{
                point_stamped_out = ps;
            }
            
            float now = (ros::Time::now()).toSec();
            float last = ps.header.stamp.toSec();
            // ROS_INFO("latency(sec): %f", last - now);
            point_pub.publish(point_stamped_out);
        }

    public:
        Node():
            nh("~")
        {
            nh.param<std::string>("reference_frame", reference_frame, "");
            nh.param<bool>("exact_time", exact_time, true);
            nh.param<int>("sync_queuesize", sync_queuesize, 10);

            if(!get_once_cameraInfo(nh, rs_intrin_)){
                ROS_ERROR("pixel_point: did not subscribe to camera info");
                ros::shutdown();
            }
            else{
                image_sub.subscribe(nh, "/depthimage_topic", 1);
                point_sub.subscribe(nh, "/pixel_topic", 1);
                // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
                if(exact_time){
                    static message_filters::Synchronizer<ExactSyncPolicy> sync(ExactSyncPolicy(sync_queuesize), image_sub, point_sub);
                    sync.registerCallback(boost::bind(&Node::sync_callback, this, _1, _2));
                }
                else{
                    static message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(sync_queuesize), image_sub, point_sub);
                    sync.registerCallback(boost::bind(&Node::sync_callback, this,_1, _2));
                }
            }
            point_pub = nh.advertise<geometry_msgs::PointStamped>("/point",10);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pixel_point");
    Node node;
    ros::spin();
    return 0;
}