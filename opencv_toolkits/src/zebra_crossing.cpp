#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <vector>

class Node{
    private:
        ros::NodeHandle nh;
        ros::Subscriber image_sub;
        ros::Publisher image_pub;
        ros::Publisher image_pub2;
        ros::Publisher point_pub1;
        ros::Publisher point_pub2;
        float coutour_area_thresh;
        float white_region_thresh;
        
        struct comp{
            template<typename T>
            bool operator()(const T &l, const T &r) const
            {
                if (l.first.x == r.first.x) {
                    return l.second.x > r.second.x;
                }
                return l.first.x < r.first.x;
            }
        };
        static bool mycomp(std::pair<cv::Point, std_msgs::Header> r, std::pair<cv::Point, std_msgs::Header> l){
            if (l.first.x == r.first.x) {
                return l.first.x > r.first.x;
            }
            return l.first.x < r.first.x;
        };
        std::vector<std::pair<cv::Point, std_msgs::Header>> point1_mem;
        std::vector<std::pair<cv::Point, std_msgs::Header>> point2_mem;

        enum Thresh_Method{
            ADAPT = 1,
            OTSU = 2,
            CANNY = 3,
        };
        Thresh_Method thresh_method;
        enum Filter_Method{
            MORPH = 1,
            ERODE = 2,
            MEDIAN = 3,
            NON = 4,
        };
        Filter_Method filter_method;

        void image_callback(const sensor_msgs::ImageConstPtr &msg){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                ros::Time curr_stamp = msg->header.stamp;
                std::string image_frame = msg->header.frame_id;
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                cv::Mat original_image = cv_ptr->image;
                int height_ = original_image.size().height;
                int width_ = original_image.size().width;
                int img_size = height_* width_;
                cv::Mat inImage;
                cv::cvtColor(original_image, inImage, cv::COLOR_RGB2GRAY);
                cv::Mat roi = set_region(inImage);
                preprocess(roi);

                // use contour
                std::vector<cv::Point> middle_points;
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::Point offset(0, inImage.size().height/2+20);
                cv::findContours(
                    roi,
                    contours,
                    hierarchy,
                    cv::RETR_EXTERNAL, //RETR_EXTERNAL / RETR_LIST / RETR_CCOMP / RETR_TREE / RETR_FLOODFILL
                    cv::CHAIN_APPROX_SIMPLE,
                    offset);
                cv::drawContours(original_image, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_8);
                for(auto &contour: contours){
                    if(cv::contourArea(contour) > (img_size*coutour_area_thresh)){
                        cv::Moments m = cv::moments(contour);
                        int cX = int(m.m10 / m.m00);
                        int cY = int(m.m01 / m.m00);
                        cv::circle(original_image, cv::Point(cX, cY), 3, cv::Scalar(255,255,0), 2);
                        middle_points.emplace_back(cv::Point(cX, cY));
                    }
                }

                // // use hough transform
                // std::vector<cv::Vec4i> lines;
                // int degree_of_tolerance = 1;
                // int pixel_of_tolerance = 1;
                // cv::HoughLinesP(roi, lines, pixel_of_tolerance, degree_of_tolerance*CV_PI/180, 250, 100, 50);
                // std::vector<cv::Point> left_points;
                // std::vector<cv::Point> right_points;
                // std::vector<cv::Point> middle_points;
                // std::cout << lines.size() << std::endl;
                // for( size_t i = 0; i < lines.size(); i++ ){
                //     cv::Vec4i l = lines[i];
                //     cv::line(original_image, cv::Point(l[0], l[1]+inImage.size().height/2+20), cv::Point(l[2], l[3]+inImage.size().height/2+20), cv::Scalar(0,0,255), 3);
                //     cv::circle(original_image, cv::Point(l[0], l[1]+inImage.size().height/2+20), 3, cv::Scalar(0,255,0), 2);
                //     left_points.emplace_back(cv::Point(l[0], l[1]+inImage.size().height/2+20));
                //     cv::circle(original_image, cv::Point(l[2], l[3]+inImage.size().height/2+20), 3, cv::Scalar(255,0,0), 2);
                //     right_points.emplace_back(cv::Point(l[2], l[3]+inImage.size().height/2+20));
                //     cv::Point m((l[0]+l[2])/2, (l[1]+l[3])/2+inImage.size().height/2+20);
                //     middle_points.emplace_back(m);
                // }
                // cv::Vec4f left_line;
                // cv::Vec4f right_line;
                // cv::Vec4f middle_line;
                // // if(left_points.size() > 1)
                // //     cv::fitLine(left_points, left_line, cv::DIST_FAIR, 0, 1e-2, 1e-2);
                // //     cv::Point l_point1, l_point2;
                // //     pointslope_to_pointpoint(left_line, l_point1, l_point2, inImage);                
                // //     cv::line(original_image, l_point1, l_point2, cv::Scalar(0,255,255), 3);
                // // if(right_points.size() > 1)
                // //     cv::fitLine(right_points, right_line, cv::DIST_FAIR, 0, 1e-2, 1e-2);
                // //     cv::Point r_point1, r_point2;
                // //     pointslope_to_pointpoint(right_line, r_point1, r_point2, inImage);                
                // //     cv::line(original_image, r_point1, r_point2, cv::Scalar(0,255,255), 3);
                // // if(middle_points.size() > 1)
                // //     cv::fitLine(middle_points, middle_line, cv::DIST_L2, 0, 1e-2, 1e-2);
                // //     cv::Point m_point1, m_point2;
                // //     pointslope_to_pointpoint(middle_line, m_point1, m_point2, inImage);                
                // //     cv::line(original_image, m_point1, m_point2, cv::Scalar(255,0,255), 3);

                cv::Mat middle_mat = cv::Mat::zeros(original_image.size(), CV_8UC1);
                for(auto &p: middle_points){
                    middle_mat.at<uchar>(p.y,p.x) = 255;
                }
                std::vector<cv::Vec2f> middle_lines;
                cv::HoughLines(middle_mat, middle_lines, 1, CV_PI/180, 1, 0, 0);
                if(!middle_lines.empty()){
                    cv::Vec2f middle_line = middle_lines[1];
                    float rho = middle_line[0], theta = middle_line[1];
                    cv::Point pt1, pt2;
                    double a = cos(theta), b = sin(theta);
                    double x0 = a*rho, y0 = b*rho;
                    pt1.x = cvRound(x0 + 1000*(-b));
                    pt1.y = cvRound(y0 + 1000*(a));
                    pt2.x = cvRound(x0 - 1000*(-b));
                    pt2.y = cvRound(y0 - 1000*(a));
                    cv::line(original_image, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
                    
                    cv::Point line1_p = pointpoint_solve_point(pt1, pt2, -1, height_-10);
                    cv::Point line2_p = pointpoint_solve_point(pt1, pt2, -1, height_-60);
                    //std::cout << line1_p.x << " " << line1_p.y << std::endl;
                    cv::circle(original_image, cv::Point(line1_p.x, line1_p.y), 3, cv::Scalar(255,0,0), 2);
                    cv::circle(original_image, cv::Point(line2_p.x, line2_p.y), 3, cv::Scalar(255,0,0), 2);
                    bool line1_p_condition = (line1_p.x >= 10 && line1_p.x <= width_-10);
                    bool line2_p_condition = (line2_p.x >= 10 && line2_p.x <= width_-10);
                    if(line1_p_condition && line2_p_condition){
                        std_msgs::Header h;
                        h.stamp = curr_stamp;
                        h.frame_id = image_frame;
                        point1_mem.emplace_back(std::make_pair(line1_p, h));
                        point2_mem.emplace_back(std::make_pair(line2_p, h));
                    }
                }

                cv_bridge::CvImage out_msg;
                out_msg.header.stamp = curr_stamp;
                out_msg.encoding = sensor_msgs::image_encodings::RGB8;
                out_msg.image = original_image;
                image_pub.publish(out_msg.toImageMsg());

                cv_bridge::CvImage out_msg2;
                out_msg2.header.stamp = curr_stamp;
                out_msg2.encoding = sensor_msgs::image_encodings::MONO8;
                out_msg2.image = roi;
                image_pub2.publish(out_msg2.toImageMsg());

                if(point1_mem.size() == 5){
                    std::nth_element(point1_mem.begin(), point1_mem.begin()+2, point1_mem.end(), mycomp);
                    std::nth_element(point2_mem.begin(), point2_mem.begin()+2, point2_mem.end(), mycomp);
                    geometry_msgs::PointStamped point_msg1;
                    geometry_msgs::PointStamped point_msg2;
                    point_msg1.header = point1_mem[2].second;
                    point_msg2.header = point2_mem[2].second;
                    geometry_msgs::Point p1_;
                    geometry_msgs::Point p2_;
                    p1_.x = point1_mem[2].first.x;
                    p2_.x = point2_mem[2].first.x;
                    p1_.y = point1_mem[2].first.y;
                    p2_.y = point2_mem[2].first.y;
                    point_msg1.point = p1_;
                    point_msg2.point = p2_;
                    point_pub1.publish(point_msg1);
                    point_pub1.publish(point_msg2);
                    point1_mem.clear();
                    point2_mem.clear();
                }
                
            }
            catch (cv_bridge::Exception &e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

        void pointslope_to_pointpoint(cv::Vec4f &src, cv::Point &p1, cv::Point &p2, cv::Mat &inImage){
            cv::Point point0;
            point0.x = src[2];
            point0.y = src[3];
            double k = src[1] / src[0];
            p1.x = 0;
            p1.y = round(k*(0-point0.x)+point0.y);
            p2.x = inImage.size().width;
            p2.y = round(k*(inImage.size().width-point0.x)+point0.y);
        }

        void preprocess(cv::Mat &inImage){
            cv::GaussianBlur(inImage, inImage, cv::Size(25,25), 0, 0, cv::BORDER_CONSTANT);
            switch(thresh_method){
                case Thresh_Method::ADAPT:
                {
                    cv::equalizeHist(inImage, inImage);
                    cv::adaptiveThreshold(inImage, inImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3, 2);
                    break;
                }
                case Thresh_Method::OTSU:
                {
                    cv::threshold(inImage, inImage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
                    break;
                }
                case Thresh_Method::CANNY:
                {
                    //cv::Canny(inImage, inImage, 10, 30, 3);
                    cv::Canny(inImage, inImage, 100, 300, 5);
                    break;
                }
                default:
                {
                    ROS_ERROR("wrong thresh method");
                    break;
                }
            }
            switch(filter_method){
                case Filter_Method::MORPH:
                {
                    cv::Mat kernel = cv::getStructuringElement(0, cv::Size(3, 3));
                    cv::morphologyEx(inImage, inImage, 2, kernel);
                    break;
                }
                case Filter_Method::ERODE:
                {
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                    cv::erode(inImage, inImage, kernel, cv::Point(-1, -1), 1);
                    cv::dilate(inImage, inImage, kernel);
                    break;
                }
                case Filter_Method::MEDIAN:
                {
                    //cv::medianBlur(inImage, inImage, 3);
                    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                    cv::dilate(inImage, inImage, kernel);
                    cv::Mat label_mat;
                    cv::Mat stats;
                    cv::Mat centroids;
                    int n = cv::connectedComponentsWithStats(
                        inImage,
                        label_mat,
                        stats,
                        centroids
                    );
                    // std::cout << n << std::endl;
                    // std::vector<int> colors(n, 0);
                    // for(int i = 1; i < n; i++){
                    //     if(stats.at<int>(i, cv::CC_STAT_AREA) < 20)
                    //         colors[i] = 1;
                    // }
                    // cv::Mat img_filtered = cv::Mat::zeros(inImage.size(), CV_8UC1);
                    // for(int y = 0; y < img_filtered.rows; y++){
                    //     for(int x = 0; x < img_filtered.cols; x++){
                    //         int label = label_mat.at<int>(y,x);
                    //         CV_Assert(0 <= label && label <=n);
                    //         img_filtered.at<int>(y, x) = 
                    //     }
                    // }
                    // std::cout << inImage.size() << std::endl;
                    // std::cout << label_mat.size() << std::endl;
                    for(int y = 0; y < inImage.rows; y++){
                        for(int x = 0; x < inImage.cols; x++){
                            int label = label_mat.at<int>(y,x);
                            if(stats.at<int>(label, cv::CC_STAT_AREA) < (inImage.total()*white_region_thresh))
                                inImage.at<uchar>(y, x) = 0;
                            // CV_Assert(0 <= label && label <=n);
                            // if(colors[label] == 1)
                            //     inImage.at<int>(y, x) = 0;
                        }
                    }
                    cv::dilate(inImage, inImage, kernel);
                    // cv::erode(inImage, inImage, kernel, cv::Point(-1, -1), 1);
                    // cv::Mat canny_image;
                    // cv::Canny(inImage, inImage, 100, 200);
                    break;
                }
                case Filter_Method::NON:
                {
                    break;
                }
                default:
                {
                    ROS_ERROR("wrong filter method");
                    break;
                }
            }
            cv::Rect border(cv::Point(0, 0), inImage.size());
            cv::rectangle(inImage, border, cv::Scalar(0, 0, 0), 3);
        }

        cv::Mat set_region(cv::Mat &inImage){
            cv::Rect region(cv::Point(0, inImage.size().height/2+20), cv::Point(inImage.size().width, inImage.size().height));
            cv::Mat roi = inImage(region);
            return roi;
        }

        cv::Point pointpoint_solve_point(cv::Point &line_pt1, cv::Point &line_pt2, int x = -1, int y = -1){
            if(y != -1){
                x = line_pt2.x - round((line_pt2.y - y)*(line_pt2.x - line_pt1.x)/(line_pt2.y - line_pt1.y));
            }else if(x != -1){
                y = line_pt2.y - round((line_pt2.x - x)*(line_pt2.x - line_pt1.x)/(line_pt2.y - line_pt1.y));
            }
            return cv::Point(x, y);
        }

    public:
        Node():
            nh("~")
        {
            image_sub = nh.subscribe("/image", 1, &Node::image_callback, this);
            image_pub = nh.advertise<sensor_msgs::Image>("/result", 1);
            image_pub2 = nh.advertise<sensor_msgs::Image>("/thresh", 1);
            point_pub1 = nh.advertise<geometry_msgs::PointStamped>("/point1", 1);
            point_pub2 = nh.advertise<geometry_msgs::PointStamped>("/point2", 1);
            
            std::string tm;
            if(nh.getParam("thresh_method", tm)){
                if(tm == "adapt"){
                    thresh_method = Thresh_Method::ADAPT;
                    ROS_INFO("set thresh method to: %s", tm.c_str());
                }
                else if(tm == "otsu"){
                    thresh_method = Thresh_Method::OTSU;
                    ROS_INFO("set thresh method to: %s", tm.c_str());
                }
                else if(tm == "canny"){
                    thresh_method = Thresh_Method::CANNY;
                    ROS_INFO("set thresh method to: %s", tm.c_str());
                } 
            }
            else{
                ROS_WARN("need specify one threding method");
                thresh_method = Thresh_Method::ADAPT;
            }

            std::string fm;
            if(nh.getParam("filter_method", fm)){
                if(fm == "morph"){
                    filter_method = Filter_Method::MORPH;
                    ROS_INFO("set filter method to: %s", fm.c_str());
                }
                else if(fm == "erode"){
                    filter_method = Filter_Method::ERODE;
                    ROS_INFO("set filter method to: %s", fm.c_str());
                }
                else if(fm == "non"){
                    filter_method = Filter_Method::NON;
                    ROS_INFO("set filter method to: %s", fm.c_str());
                }
                else if(fm == "median"){
                    filter_method = Filter_Method::MEDIAN;
                    ROS_INFO("set filter method to: %s", fm.c_str());
                }
            }
            else{
                filter_method = Filter_Method::NON;
            }

            if(nh.getParam("coutour_area_thresh", coutour_area_thresh)){
                ROS_INFO("set ratio of coutour_area_thresh: %f", coutour_area_thresh);
            }
            else{
                ROS_WARN("use default ratio of coutour_area_thresh: 0.005");
                white_region_thresh = 0.005;
            }

            if(nh.getParam("white_region_thresh", white_region_thresh)){
                ROS_INFO("set ratio of white_region_thresh: %f", white_region_thresh);
            }
            else{
                ROS_WARN("use default ratio of white_region_thresh: 0.0015");
                white_region_thresh = 0.0015;
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "zebra_crossing");
    Node node;
    ros::spin();
    return 0;
}