#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "net.h"
#include "mat.h"
#include "opencv2/opencv.hpp"
#include "chrono"
#include "tracking.h"


#define IMG_H 480
#define IMG_W 640

class ncnn_image
{
    public:
    ros::NodeHandle nh;
    cv::Mat cv_image;
    
    ros::Publisher ncnn_grayimage_feats_pub;
    ros::Publisher image_feats_pub;
    ros::Publisher ncnn_descimage_pub;
    ros::Subscriber cam_sub;

    ncnn::Net net;
    corner_tracking tracker;
     

    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1.0/255.0, 1.0/255.0, 1.0/255.0};
    const float mean_vals_inv[3] = {0, 0, 0};
    const float norm_vals_inv[3] = {255.f, 255.f, 255.f};

    ncnn_image() { 
        
         net.opt.num_threads=8;
         net.load_param("/home/hhh/project_hhh/temp/slam/lvio/src/ncnn_images/models/model.param");
         net.load_model("/home/hhh/project_hhh/temp/slam/lvio/src/ncnn_images/models/model.bin");
         cam_sub = nh.subscribe<sensor_msgs::CompressedImage>("/camera/image_color/compressed",1000, &ncnn_image::ImageCBK,this);
         ncnn_grayimage_feats_pub = nh.advertise<sensor_msgs::Image>("/ncnn_grayimage_feats",100);
         image_feats_pub = nh.advertise<sensor_msgs::Image>("/image_feats",100);
         ncnn_descimage_pub = nh.advertise<sensor_msgs::Image>("/ncnn_descimage_feats",100);
    }

    void ncnn_solve(cv::Mat& mat_in, cv::Mat mat_out);
    void ImageCBK (const sensor_msgs::CompressedImageConstPtr &msg);
   
};

void ncnn_image::ImageCBK(const sensor_msgs::CompressedImageConstPtr &msg)
{
        auto msg_in = msg;
        // ros->cv
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::RGB8);
        cv_image = cv_ptr->image;

        cv::Mat image_;
        ncnn_solve(cv_image, image_);

       
 }

 void ncnn_image::ncnn_solve(cv::Mat& mat_in, cv::Mat mat_out)
 {
    cv::Mat score(IMG_H, IMG_W, CV_8UC1);
    cv::Mat desc(IMG_H, IMG_W, CV_8UC3);
    ncnn::Mat in;
    ncnn::Mat out1, out2;

    
        ncnn::Extractor ex = net.create_extractor();
        ex.set_light_mode(true);
        ex.set_num_threads(4);

        if (mat_in.empty())
            return;

        cv::resize(mat_in, mat_in, cv::Size(IMG_W, IMG_H));

        //////////////////////////  opencv image to ncnn mat  //////////////////////////
        // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        in = ncnn::Mat::from_pixels(mat_in.data, ncnn::Mat::PIXEL_BGR, mat_in.cols, mat_in.rows);
        in.substract_mean_normalize(mean_vals, norm_vals);

        //////////////////////////  ncnn forward  //////////////////////////

        // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

        ex.input("input", in);
        ex.extract("score", out1);
        ex.extract("descriptor", out2);

        //////////////////////////  ncnn mat to opencv image  //////////////////////////

        // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        out1.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
        out2.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

      //memcpy((uchar*)score.data, out1.data, sizeof(float) * out1.w * out1.h);
        out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
        out2.to_pixels(desc.data, ncnn::Mat::PIXEL_BGR);

        // std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();

        cv::Mat new_score  = score.clone();
        cv::Mat new_desc  = desc.clone();
        auto fea_pts_vec =  tracker.extractFeature(score);

        for (auto& p : fea_pts_vec)
        {
            cv::circle(mat_in, p, 2, cv::Scalar(0, 255, 0), 2);
            cv::circle(new_score, p, 2, cv::Scalar(255, 255, 255), 1);
            
        }
        // tracker.update(score, new_desc);
        // tracker.show(mat_in,mat_out); // show feature points
        // tracker.showsize();   
        // 特征点位置

         // toRos
        cv_bridge::CvImage bridge_1;
        bridge_1.image = mat_in;
        bridge_1.encoding = "rgb8";
        sensor_msgs::Image::Ptr imageShowPointer_1 = bridge_1.toImageMsg();
        imageShowPointer_1->header.stamp = ros::Time::now();
        image_feats_pub.publish(imageShowPointer_1);

        cv_bridge::CvImage bridge_2;
        bridge_2.image = new_score;
        bridge_2.encoding = "mono8";
        sensor_msgs::Image::Ptr imageShowPointer_2 = bridge_2.toImageMsg();
        imageShowPointer_2->header.stamp = ros::Time::now();
        ncnn_grayimage_feats_pub.publish(imageShowPointer_2);

        cv_bridge::CvImage bridge_3;
        bridge_3.image = new_desc;
        bridge_3.encoding = "rgb8";
        sensor_msgs::Image::Ptr imageShowPointer_3 = bridge_3.toImageMsg();
        imageShowPointer_3->header.stamp = ros::Time::now();
        ncnn_descimage_pub.publish(imageShowPointer_3);
 }

 int main(int argc, char**argv)
 {
    std::setlocale(LC_ALL, "");
    ros::init (argc, argv, "ncnn_image_node");
    printf("********* ncnn_image_node start *******\n");

    ncnn_image image_;

    ros::spin();

    return 0;
 }

 //////////////////////////  show times  //////////////////////////
        // std::chrono::duration<double> time_used_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        // std::chrono::duration<double> time_used_2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3-t2);
        // std::chrono::duration<double> time_used_3 = std::chrono::duration_cast<std::chrono::duration<double>>(t4-t3);

        // std::cout<<"time_used 1 : "<<time_used_1.count()*1000<<"ms"<<std::endl;
        // std::cout<<"time_used 2 : "<<time_used_2.count()*1000<<"ms"<<std::endl;
        // std::cout<<"time_used 3 : "<<time_used_3.count()*1000<<"ms"<<std::endl;