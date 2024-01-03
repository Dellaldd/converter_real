#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <sensor_msgs/Imu.h>
using namespace cv;
using namespace std;
class ImageConver{
    public:
        std_msgs::Header header;
        void imuCallback(const sensor_msgs::ImuConstPtr& msg){
            header = msg->header;
            // cout << "in callback:" << header.stamp.toSec() << endl;
        }
        
    
        void initial(){
            imu_sub = n.subscribe<sensor_msgs::Imu>("/imu0", 50, &ImageConver::imuCallback,this);
        }

        ImageConver(const ros::NodeHandle& nh):n(nh){

        }
        // ~ImageConver();

    private:
        ros::NodeHandle n;
        ros::Subscriber imu_sub;
};

// void imuCallback(const std_msgs::String::ConstPtr& msg){
//             std::cout << "get imu" << std::endl;
//         }

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cam_node");
    ros::NodeHandle nh("~");
    ImageConver imageConver(nh);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher cam0_pub = it.advertise("test/cam0_image", 1);
    image_transport::Publisher cam1_pub = it.advertise("test/cam1_image", 1);
    
    Mat cam0(480, 640, CV_8UC3), cam1(480, 640, CV_8UC3);
    ros::Rate loop_rate(10);
    VideoCapture videoCapture(0);
    std_msgs::Header header;
    
    imageConver.initial();
    while (ros::ok()){
        videoCapture >>cam0;
        cam1 = cam0;
        cout << "in main: " << imageConver.header.stamp.toSec() << endl;
        header = imageConver.header;
        cv_bridge::CvImage image0(header,"bgr8", cam0);
        cv_bridge::CvImage image1(header, "bgr8", cam1);
        
        cam0_pub.publish(image0.toImageMsg());
        cam1_pub.publish(image1.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }
}