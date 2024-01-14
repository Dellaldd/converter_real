#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>

using namespace Eigen;
using namespace std;

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
        alpha_ = 1;
        double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
        prev_output_ = output;
        return output;
    }

    double update_angular(double input) {
        alpha_ =  1;
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
            imu_raw_acc[0] = msg->linear_acceleration.x/scale*1.953125;
            imu_raw_acc[1] = msg->linear_acceleration.y/scale*1.953125;
            imu_raw_acc[2] = msg->linear_acceleration.z/scale*1.953125;

            imu_raw_gyro[0] = msg->angular_velocity.x * 17.4532925;
            imu_raw_gyro[1] = msg->angular_velocity.y * 17.4532925;
            imu_raw_gyro[2] = msg->angular_velocity.z * 17.4532925;

            imu_full.header = msg->header;

            imu_full.angular_velocity.x = imu_raw_gyro[0];
            imu_full.angular_velocity.y = imu_raw_gyro[1];
            imu_full.angular_velocity.z = imu_raw_gyro[2];//rad/s

            imu_full.linear_acceleration.x = imu_raw_acc[0]*9.80665;
            imu_full.linear_acceleration.y = imu_raw_acc[1]*9.80665;
            imu_full.linear_acceleration.z = imu_raw_acc[2]*9.80665; //g

            // imu_full.angular_velocity.x = filter_angular_1.update_angular(imu_raw_gyro[0]);
            // imu_full.angular_velocity.y = filter_angular_2.update_angular(imu_raw_gyro[1]);
            // imu_full.angular_velocity.z = filter_angular_3.update_angular(imu_raw_gyro[2]);//rad/s

            // imu_full.linear_acceleration.x = filter_acc_1.update_acc(imu_raw_acc[0]*9.80665);
            // imu_full.linear_acceleration.y = filter_acc_2.update_acc(imu_raw_acc[1]*9.80665);
            // imu_full.linear_acceleration.z = filter_acc_3.update_acc(imu_raw_acc[2]*9.80665); //g

        }

        void imu_Callback(const sensor_msgs::ImuConstPtr& msg){
            vec[0] = msg->angular_velocity.x;
            vec[1] = msg->angular_velocity.y;
            vec[2] = msg->angular_velocity.z;
            q = Eigen::AngleAxisf(vec[0], ::Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(vec[1], ::Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(vec[2], ::Eigen::Vector3f::UnitX());
            
            imu_full.orientation.x = q.x();
            imu_full.orientation.y = q.y();
            imu_full.orientation.z = q.z();
            imu_full.orientation.w = q.w();

            Matrix3f rx = q.toRotationMatrix();
            Eigen::Vector3f ea = rx.eulerAngles(2,1,0);      
        }

        

        void optitrack_Callback(const geometry_msgs::PoseStampedConstPtr& msg){
            tf::Quaternion rq;
            tf::quaternionMsgToTF(msg->pose.orientation, rq);
            Vector3d ea;
            tf::Matrix3x3(rq).getRPY(ea[0], ea[1], ea[2]);
            
            pos.position.x = msg->pose.position.x;
            pos.position.y = msg->pose.position.y;
            pos.position.z = msg->pose.position.z;
            pos.yaw_rate = ea[2];
            
            pos.type_mask = 0b011111111000;
            pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos.header = msg->header;  
            
        }

        void imu_state_Callback(const sensor_msgs::ImuConstPtr& msg){
            // if(pos.yaw_rate != 0)
            cout << "yaw_rate:" << pos.yaw_rate << " current imu state: " << msg->linear_acceleration  << endl;     
        }
        
        void initial(){
            imu_mag_sub = n.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 140, &ImuConver::magCallback,this);
            imu_raw_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 140, &ImuConver::imu_rawCallback,this);
            imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 140, &ImuConver::imu_Callback,this);
            optitrack_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/jiahao1/pose", 15, &ImuConver::optitrack_Callback,this);
            imu_state_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/full", 140, &ImuConver::imu_state_Callback,this);
        }

        ImuConver(const ros::NodeHandle& nh):n(nh){

        }

    sensor_msgs::Imu imu_full;
    mavros_msgs::PositionTarget pos;

    private:
        ros::NodeHandle n;
        ros::Subscriber imu_mag_sub, imu_raw_sub, imu_sub, openvins_sub, imu_state_sub, optitrack_sub;
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
    ros::Publisher imu_full_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/full",1);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1);
    
    ros::Rate loop_rate(140);
    imuConver.initial();
    cout << "start convert!" << endl;
    while (ros::ok()){
        
        if(imuConver.imu_full.linear_acceleration.x!=0){
            imu_full_pub.publish(imuConver.imu_full);
        }
    	setpoint_pub.publish(imuConver.pos);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}