#ifndef VETSG_UTILS
#define VETSG_UTILS
#include<opencv2/opencv.hpp>
#include<iostream>
#include<cmath>

//#define THRESHOLD_GRAY 20
extern int THRESHOLD_GRAY;

//void pre_process(cv::Mat& image,cv::Mat& pre_image,cv::Mat& dframe,const int frame_count);


void equalizehist_transformation(cv::Mat& image);

void median_blur(cv::Mat& image);

void bilateral_blur(cv::Mat& image);

void draw_sg_moving(cv::Mat &image,int x,int y);
void draw_sg_stop(cv::Mat &image,int x,int y);

void get_dframe_moving_count(cv::Mat dframe,int& count);

void compute_avg_gray(cv::Mat image,unsigned char& avg_of_gray);

long compute_horizontal_sum_of_variance(cv::Mat &image_gray);

#endif // VETSG_UTILS

