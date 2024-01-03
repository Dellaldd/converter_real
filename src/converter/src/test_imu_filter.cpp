#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>
using namespace Eigen;
using namespace std;

// low-pass
class LowPassFilter {
public:
    LowPassFilter(double sample_rate, double cutoff_frequency) {
        double dt = 1.0 / sample_rate;
        double RC = 1.0 / (cutoff_frequency * 2.0 * M_PI);
        alpha_ = dt / (dt + RC);
        prev_output_ = 0.0;
    }

    LowPassFilter() {
        double sample_rate = 140;
        double cutoff_frequency = 50;
        double dt = 1.0 / sample_rate;
        double RC = 1.0 / (cutoff_frequency * 2.0 * M_PI);
        alpha_ = dt / (dt + RC);
        prev_output_ = 0.0;
    }
 
    // update output
    double update_acc(double input) {
        alpha_ = 0.01;
        double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
        prev_output_ = output;
        return output;
    }

    double update_angular(double input, int coordinate) {
        if (coordinate == 2){
            alpha_ = 0.01;
        }else{
            alpha_ = 0.02;
        }
        
        double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
        prev_output_ = output;
        return output;
    }
 
private:
    double alpha_;
    double prev_output_;
};

class ImuConver{
    public:
        std_msgs::Header header;
        void magCallback(const sensor_msgs::MagneticFieldConstPtr& msg){
            scale = msg->magnetic_field.x;
        }

        void imu_rawCallback(const sensor_msgs::ImuConstPtr& msg){

            imu_filter.header = msg->header;
            imu_filter.angular_velocity.x = filter_angular_1.update_angular(msg->angular_velocity.x,0);
            imu_filter.angular_velocity.y = filter_angular_2.update_angular(msg->angular_velocity.y,1);
            imu_filter.angular_velocity.z = filter_angular_3.update_angular(msg->angular_velocity.z,2);//rad/s

            imu_filter.orientation.x = msg->orientation.x;
            imu_filter.orientation.y = msg->orientation.y;
            imu_filter.orientation.z = msg->orientation.z;
            imu_filter.orientation.w = msg->orientation.w;
            // cout << imu_filter.orientation << endl;

            imu_filter.linear_acceleration.x = filter_acc_1.update_acc(msg->linear_acceleration.x);
            imu_filter.linear_acceleration.y = filter_acc_2.update_acc(msg->linear_acceleration.y);
            imu_filter.linear_acceleration.z = filter_acc_3.update_acc(msg->linear_acceleration.z); //g
            // cout << "have processed one!" << endl;
        }

        void imu_Callback(const sensor_msgs::ImuConstPtr& msg){
            vec[0] = msg->angular_velocity.x;
            vec[1] = msg->angular_velocity.y;
            vec[2] = msg->angular_velocity.z;
            q = Eigen::AngleAxisf(vec[0], ::Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(vec[1], ::Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(vec[2], ::Eigen::Vector3f::UnitX());
            
            // imu_full.orientation.x = q.x();
            // imu_full.orientation.y = q.y();
            // imu_full.orientation.z = q.z();
            // imu_full.orientation.w = q.w();

            Matrix3f rx = q.toRotationMatrix();
            Eigen::Vector3f ea = rx.eulerAngles(2,1,0);      
        }

        void openvins_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
            Eigen::Quaternionf q;
            q.x() = msg->pose.pose.orientation.x;
            q.y() = msg->pose.pose.orientation.y;
            q.z() = msg->pose.pose.orientation.z;
            q.w() = msg->pose.pose.orientation.w;
            Matrix3f rx = q.toRotationMatrix();
            Eigen::Vector3f ea = rx.eulerAngles(2,1,0); 
            pos.position.x = msg->pose.pose.position.x;
            pos.position.y = msg->pose.pose.position.y;
            pos.position.z = msg->pose.pose.position.z;
            pos.velocity.x;
            pos.yaw = ea[2];
            pos.type_mask = // mavros_msgs::PositionTarget::IGNORE_PX |
                                    // mavros_msgs::PositionTarget::IGNORE_PY |
                                    // mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    // mavros_msgs::PositionTarget::IGNORE_YAW |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            pos.coordinate_frame = 1;
            pos.header = msg->header;  
        }
        
    
        void initial(){
            imu_raw_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/full", 140, &ImuConver::imu_rawCallback,this);
        }

        ImuConver(const ros::NodeHandle& nh):n(nh){

        }

    sensor_msgs::Imu imu_filter;
    mavros_msgs::PositionTarget pos;

    private:
        ros::NodeHandle n;
        ros::Subscriber imu_mag_sub, imu_raw_sub, imu_sub, openvins_sub;
        float scale = 1;
        Vector3f imu_raw_gyro, imu_raw_acc;
        Quaternionf q;
        Vector3f vec;
        LowPassFilter filter_acc_1, filter_acc_2, filter_acc_3, filter_angular_1, filter_angular_2, filter_angular_3;

};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");
    ImuConver imuConver(nh);
    ros::Publisher imu_full_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/filter",1);

    ros::Rate loop_rate(140);
    
    imuConver.initial();
    while (ros::ok()){
        imu_full_pub.publish(imuConver.imu_filter);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}