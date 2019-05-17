#ifndef VETSTOPORGO_H
#define VETSTOPORGO_H
#include "vetthiscarmotionstatejudging_frame_diff.h"
#include "vetfrontcarmotionstatejudging_frame_diff.h"
#include "vetsg_utils.h"
#include <utility>

using namespace std;
using namespace cv;

//SG模块数据结构    本车运动状态、前车运动状态、是否发出起步提醒、
struct STU_SG_DATA
{
    bool this_car_state;
    bool front_car_state;
    bool go;
    float fps;
};
class VetStopOrGo
{
public:
    VetStopOrGo();
    int remindDriver(Mat& frame,STU_SG_DATA& sg_data);
    int start(Mat& frame,STU_SG_DATA& sg_data);
    void pre_process(cv::Mat& image,cv::Mat& pre_image,cv::Mat& dframe);

private:

private:
    cv::Mat pre_frame;
    bool front_car_stop_;
    timeb time;
    timeb last_remind_;
    int count_;
    VetThisCarMotionStateJudging_Frame_Diff this_car_;
    VetFontCarMotionStateJudging_Frame_Diff front_car_;
    bool night_precess;
};

#endif // VETSTOPORGO_H
