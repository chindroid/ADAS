#ifndef VETFRONTCARSTATEJUDGING
#define VETFRONTCARSTATEJUDGING
#include<opencv2/opencv.hpp>
#include<iostream>
#include "vetsg_utils.h"


//前车运动状态检测参数
extern int THRESHOLD_GRAY;
extern int DEQUE_MAXSIZE;
extern float THRESHOLD_RARIO_MITIPLE;

class VetFontCarMotionStateJudging_Frame_Diff{
public:
    VetFontCarMotionStateJudging_Frame_Diff();
    void getFrontCarMotionState(cv::Mat &dframe_total,bool &front_car_state);
    void clear_aver_recent_ratio();

    cv::Rect center_roi;
private:
    std::deque<float> recent_front_car_diff_ratio;
    float aver_recent_ratio;
//    cv::Mat pre_frame;

    void make_trapezoid(cv::Mat& center_frame);
    void update_aver_recent_ratio();

};
#endif // VETFRONTCARSTATEJUDGING

