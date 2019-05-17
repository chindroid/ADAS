#ifndef VETTHISCARMOTIONSTATEJUDGING_FRAME_DIFF
#define VETTHISCARMOTIONSTATEJUDGING_FRAME_DIFF
#include<opencv2/opencv.hpp>
#include<iostream>

#include "vetsg_utils.h"

extern int THRESHOLD_GRAY;
extern float THRESHOLD_RATIO;

class VetThisCarMotionStateJudging_Frame_Diff{
public:
    VetThisCarMotionStateJudging_Frame_Diff(int cols_,int rows_);
    void getThisCarMotionState(cv::Mat &dframe,bool &state);
    void compute_horizontal_sum_variance_total(cv::Mat &image_gray,
        float &l_top_total_variance,float &l_bottom_total_variance,float &r_top_total_variance,float &r_bottom_total_variance);

    cv::Rect left_top_roi;
    cv::Rect left_bottom_roi;
    cv::Rect right_top_roi;
    cv::Rect right_bottom_roi;

private:

    int l_top_pixel_count;
    int l_bottom_pixel_count;
    int r_top_pixel_count;
    int r_bottom_pixel_count;

    int this_car_motion_state;
    bool last_this_car_motion_state;

//    void compute_variance_of_roi(cv::Mat img);

};
#endif // VETTHISCARMOTIONSTATEJUDGING_FRAME_DIFF

