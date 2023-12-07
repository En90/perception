#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class Node{
    private:
        ros::NodeHandle nh;
        ros::Publisher pose_pub;
        tf::TransformListener _tfListener;
        std::string reference_frame;
        typedef message_filters::sync_policies::ExactTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> ExactSyncPolicy;
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub1;
        message_filters::Subscriber<geometry_msgs::PointStamped> point_sub2;

        void sync_callback(const geometry_msgs::PointStamped::ConstPtr &point1_msg, const geometry_msgs::PointStamped::ConstPtr &point2_msg){
            float theta;
            if(point2_msg->point.x > point1_msg->point.x){
                theta = 3.14159/2 - atan((point1_msg->point.y - point2_msg->point.y)/(point2_msg->point.x - point1_msg->point.x));
            }
            else if(point2_msg->point.x < point1_msg->point.x){
                theta = -1*(3.14159/2 - atan((point1_msg->point.y - point2_msg->point.y)/(point1_msg->point.x - point2_msg->point.x)));
            }
            
            geometry_msgs::PoseStamped p;
            p.header = point1_msg->header;
            p.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
            p.pose.position.x = point1_msg->point.x;
            p.pose.position.y = point1_msg->point.y;
            p.pose.position.z = point1_msg->point.z;

            geometry_msgs::PoseStamped pose_stamped_out;
            if (reference_frame != p.header.frame_id){
                try{
                    static tf::TransformListener listener(ros::Duration(10));
                    listener.transformPose(reference_frame, p, pose_stamped_out);
                }
                catch (const tf::TransformException& e){
                    ROS_ERROR_STREAM("Error in transform of " << p.header.frame_id << " in " << reference_frame);
                    ros::shutdown();
                }
            }
            else{
                pose_stamped_out = p;
            }
            pose_pub.publish(pose_stamped_out);
        }

    public:
        Node():
            nh("~")
        {
            point_sub1.subscribe(nh, "/point1", 1);
            point_sub2.subscribe(nh, "/point2", 1);
            static message_filters::Synchronizer<ExactSyncPolicy> sync(ExactSyncPolicy(10), point_sub1, point_sub2);
            sync.registerCallback(boost::bind(&Node::sync_callback, this, _1, _2));
            nh.param<std::string>("reference_frame", reference_frame, "");
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/destination_pose",10);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "z_main");
    Node node;
    ros::spin();
    return 0;
}