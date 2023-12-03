class Node{
    private:
        ros::NodeHandle nh;
        ros::subscriber point_sub1;
        ros::subscriber point_sub2;
    public:
        
};

int main(int argc, char** argv){
    ros::init(argc, argv, "z_main");
    Node node;
    ros::spin();
    return 0;
}