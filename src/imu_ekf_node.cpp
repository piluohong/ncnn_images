
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

using namespace Eigen;

struct IMUdata
{
    /* data */
    Eigen::Vector3d data_acc;
    Eigen::Vector3d data_gyro;
    double timestamp;
};

// 静态
class IMUInitializer 
{
public:
    IMUInitializer() {
        GRAVITY_ << 0, 0, 9.80;
        imu_sub = nh.subscribe("/livox/imu", 1000, &IMUInitializer::IMU_CBK,this);
        
    }

    void IMU_CBK(const sensor_msgs::Imu::ConstPtr &imu_in){
        
        if(imu_count == 0){
            imu_first = imu_in->header.stamp;
            ROS_INFO("First frame\n");
        }
        // if((imu_in->header.stamp - imu_first).toSec() <= 10){
            // if(imu_count == 0)
                
            
            if(normalized){
                data.data_acc << imu_in->linear_acceleration.x * 9.8, imu_in->linear_acceleration.y*9.8,imu_in->linear_acceleration.z*9.8;
                data.data_gyro << imu_in->angular_velocity.x, imu_in->angular_velocity.y,imu_in->angular_velocity.z;
                data.timestamp = imu_in->header.stamp.toSec();
            }else{
                data.data_acc << imu_in->linear_acceleration.x, imu_in->linear_acceleration.y,imu_in->linear_acceleration.z;
                data.data_gyro << imu_in->angular_velocity.x, imu_in->angular_velocity.y,imu_in->angular_velocity.z;
                data.timestamp = imu_in->header.stamp.toSec();
            }

            data_vec.push_back(data);
            imu_count++;
            // std::cout << "imu msg number: " << imu_count << std::endl;
            
        //  return;
    // }

         
        if(IMU_Initialize()){
            printf("Initialize success\n");
            ros::shutdown();
        }else{
            printf("Initialize failure\n");
            // ros::shutdown();
        }

    }
    // 5s 静态初始化
    bool IMU_Initialize(){

        if (data_vec.empty()) return false;

        if (data_vec.back().timestamp - data_vec.begin()->timestamp < window_length)
            return false;

        Eigen::Vector3d accel_avg(0, 0, 0);
        Eigen::Vector3d gyro_avg(0, 0, 0);

        std::vector<IMUdata> imu_cache;
        for (size_t i = data_vec.size() - 1; i >= 0; i--) {
            accel_avg += data_vec[i].data_acc;
            gyro_avg += data_vec[i].data_gyro;
            imu_cache.push_back(data_vec[i]);
            if (data_vec.back().timestamp - data_vec[i].timestamp >= window_length)
            break;
        }
        accel_avg /= (int)imu_cache.size();
        gyro_avg /= (int)imu_cache.size();

        double accel_var = 0;
        for (size_t i = 0; i < imu_cache.size(); i++) {
            accel_var +=
                (imu_cache[i].data_acc - accel_avg).dot(imu_cache[i].data_acc - accel_avg);
        }
        accel_var = std::sqrt(accel_var / ((int)imu_cache.size() - 1));

        if (accel_var >= imu_excite_threshold_) {
            std::cout << "[IMUInitializer] Dont Move !\n";
            return false;
        }

        Eigen::Vector3d z_axis = accel_avg / accel_avg.norm();
        // std::cout << MAGENTA << "[accel_avg] " << accel_avg.transpose() << RESET << std::endl;
        // std::cout << MAGENTA << "[z_axis] " << z_axis.transpose() << RESET << std::endl;
        Eigen::Vector3d e_1(1, 0, 0);
        Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
        x_axis = x_axis / x_axis.norm();

        Eigen::Matrix<double, 3, 1> y_axis =
            z_axis.cross(x_axis);

        Eigen::Matrix<double, 3, 3> Rot;  //R_GtoI0
        Rot.block<3, 1>(0, 0) = x_axis;
        Rot.block<3, 1>(0, 1) = y_axis;
        Rot.block<3, 1>(0, 2) = z_axis;
        Eigen::Vector3d g_inI0 = Rot * GRAVITY_; // world -> imu

        Eigen::Matrix3d R_I0toG = Rot.inverse(); // imu -> world
        
        // Set our state variables
        auto time0 = data_vec.at(data_vec.size()-1).timestamp;
        auto q_GtoI0 = R_I0toG;
        auto b_w0 = gyro_avg;
        auto v_I0inG = Eigen::Matrix<double,3,1>::Zero();
        auto b_a0 = accel_avg - g_inI0;
        auto p_I0inG = Eigen::Matrix<double,3,1>::Zero();
        std::cout << "Result: \n" << q_GtoI0 << std::endl;
        std::cout << "\n "<< b_w0 << std::endl;
        std::cout << "\n "<< v_I0inG << std::endl;
        std::cout << "\n "<< b_a0 << std::endl;
        std:: cout << "\n "<< p_I0inG << std::endl;
        
        
        // Done!!!
        return true; 

    }

    inline Eigen::Matrix<double,4,1> rot_2_quat(Eigen::Matrix<double,3,3> &Ro){
        Eigen::Quaterniond quat(Ro);

        Eigen::Matrix<double,4,1> quat_;
        quat_(0,0) = quat.x();
        quat_(1,0) = quat.y();
        quat_(2,0) = quat.z();
        quat_(3,0) = quat.w();

        return quat_;

    }

    inline Eigen::Matrix<double,3,3> quat_2_Rot(Eigen::Matrix<double,4,1> &quat){
        Eigen::Matrix<double,3,3> rot;

        Eigen::Quaterniond q;
        q.x() = quat(0,0);
        q.y() = quat(1,0);
        q.z() = quat(2,0);
        q.w() = quat(3,0);


        rot = q.toRotationMatrix();
        return rot;
    }
        

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    IMUdata data;
    std::vector<IMUdata> data_vec;
    
    bool normalized = true;
    bool wait_for_jerk = false;
    ros::Time imu_first;

    uint32_t imu_count = 0;
    double window_length = 5;
    double imu_excite_threshold_ = 0.5;
    Eigen::Vector3d GRAVITY_;
     
 
};

class IMUodometry {
public:
    IMUodometry(){

    }

private:

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_ekf_node");
    printf("********* imu_ekf_node start *******\n");

    IMUInitializer imu_initial_node;

    ros::spin();
    return 0;
}


// if (imu_data.empty())
        //     return false;
        
        // double newesttime = imu_data.at(imu_data.size()-1).timestamp;


        // // 这里分为两个窗口，一个是从静态跳到动态，一个是就静态
        // // std::vector<IMUdata> window_newest, window_secondnew;
        // // for(IMUdata data : imu_data) {
        // //     if(data.timestamp > newesttime-1*window_length && data.timestamp <= newesttime-0*window_length) {
        // //     window_newest.push_back(data);
        // //     }
        // //     if(data.timestamp > newesttime-2*window_length && data.timestamp <= newesttime-1*window_length) {
        // //     window_secondnew.push_back(data);
        // //     }
        // // }

        // if (window_newest.empty() || window_secondnew.empty()){
        //     return false;
        // }

        // // Calculate the sample variance for the newest one
        // // a_avg平均加速度
        // Eigen::Matrix<double,3,1> a_avg = Eigen::Matrix<double,3,1>::Zero();
        // for(IMUdata data : window_newest) {
        // a_avg += data.data_acc;
        // }
        // a_avg /= (int)window_newest.size();
        // // a_var方差，通过方差可以判断运动的激励够不够
        // double a_var = 0;
        // for(IMUdata data : window_newest) {
        // a_var += (data.data_acc - a_avg).dot(data.data_acc-a_avg);
        // }
        // a_var = std::sqrt(a_var/((int)window_newest.size()-1));

        // if(a_var < imu_excite_threshold_) {
        //     // printf(YELLOW "InertialInitializer::initialize_with_imu(): no IMU excitation, below threshold %.4f < %.4f\n" RESET,a_var,_imu_excite_threshold);
        //     return false;
        // }


        // // 静止窗口中的平均加速度和平均角速度，然后计算方差，判断激励够不够
        // Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
        // Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
        // for(size_t i=0; i<window_secondnew.size(); i++) {
        // linsum += window_secondnew.at(i).data_acc;
        // angsum += window_secondnew.at(i).data_gyro;
        // }
        
        // // Calculate the mean of the linear acceleration and angular velocity
        // Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
        // Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
        // linavg = linsum/window_secondnew.size();
        // angavg = angsum/window_secondnew.size();
        
        
        // double a_var2 = 0;
        // for(IMUdata data : window_secondnew) {
        // a_var2 += (data.data_acc-linavg).dot(data.data_acc-linavg);
        // }
        // a_var2 = std::sqrt(a_var2/((int)window_secondnew.size()-1));
        
        // // If it is above the threshold and we are not waiting for a jerk
        // // Then we are not stationary (i.e. moving) so we should wait till we are
        // //静止的窗口运动激励要小于阈值才能继续初始化
        // if((a_var > imu_excite_threshold_ || a_var2 > imu_excite_threshold_)) {
        //     // printf(YELLOW "InertialInitializer::initialize_with_imu(): to much IMU excitation, above threshold %.4f,%.4f > %.4f\n" RESET,a_var,a_var2,_imu_excite_threshold);
        //     return false;
        // }
        
        // // Get z axis, which alines with -g (z_in_G=0,0,1)
        // // 静止时加速度计测量值为g
        // // 确定z轴方向后，我们利用施密特正交化构建单位坐标系。施密特正交化过程为
        // // v1 = x1 
        // // v2 = x2-(x2*v1)/(v1*v1)*v1
        // // 还剩下一个轴可以利用两个轴的叉乘来确定
        // Eigen::Vector3d z_axis = linavg/linavg.norm();
        
        // // Create an x_axis
        // // x轴
        // Eigen::Vector3d e_1(1,0,0);
        
        // // Make x_axis perpendicular to z
        // //根据施密特正交化可得
        // Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
        // x_axis= x_axis/x_axis.norm();
        
        // // Get z from the cross product of these two
        // // 通过叉乘确定y轴
        // Eigen::Matrix<double,3,1> y_axis = z_axis.cross(x_axis);
        
        // // From these axes get rotation
        // //确定初始的旋转
        // Eigen::Matrix<double,3,3> Ro;
        // Ro.block(0,0,3,1) = x_axis;
        // Ro.block(0,1,3,1) = y_axis;
        // Ro.block(0,2,3,1) = z_axis;
        
        // // Create our state variables
        // Eigen::Matrix<double,4,1> q_GtoI = rot_2_quat(Ro);
        
        // // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
        // //静止的时候，角速度应该就是bias了
        // Eigen::Matrix<double,3,1> bg = angavg;
        // //则加速度bias如下，因为静止加速度就是重力和bias了
        // Eigen::Matrix<double,3,1> ba = linavg - quat_2_Rot(q_GtoI)* GRAVITY_;