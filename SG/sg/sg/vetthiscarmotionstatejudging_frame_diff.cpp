#include"vetthiscarmotionstatejudging_frame_diff.h"

using namespace std;
using namespace cv;

VetThisCarMotionStateJudging_Frame_Diff::VetThisCarMotionStateJudging_Frame_Diff(int cols_,int rows_ ){
    int width=cols_/4;
    int height=rows_/3;

    //初始化四块检测区域的ROI划分
    left_top_roi=Rect(0,0,width,height);
    left_bottom_roi=Rect(0,height,width,height*2);
    right_top_roi=Rect(cols_-width,0,width,height);
    right_bottom_roi=Rect(cols_-width,height,width,height*2);

    //四块检测区域的总像素个数
    l_top_pixel_count=left_top_roi.area();
    r_top_pixel_count=right_top_roi.area();
    l_bottom_pixel_count=left_bottom_roi.area();
    r_bottom_pixel_count=right_bottom_roi.area();

    this_car_motion_state=1;
    last_this_car_motion_state=false;
}

extern float THRESHOLD_RATIO_LT;
extern float THRESHOLD_RATIO_LB;
extern float THRESHOLD_RATIO_RT;
extern float THRESHOLD_RATIO_RB;

void VetThisCarMotionStateJudging_Frame_Diff::getThisCarMotionState(Mat &dframe,bool &state){

    //四块检测区域的ROI划分
    Mat left_top_dframe=dframe(left_top_roi);
    Mat left_bottom_dframe=dframe(left_bottom_roi);
    Mat right_top_dframe=dframe(right_top_roi);
    Mat right_bottom_dframe=dframe(right_bottom_roi);

    int l_top_diff_point_count=0;
    int l_bottom_diff_point_count=0;
    int r_top_diff_point_count=0;
    int r_bottom_diff_point_count=0;

    //计算差分像素的个数
    get_dframe_moving_count(left_top_dframe,l_top_diff_point_count);
    get_dframe_moving_count(left_bottom_dframe,l_bottom_diff_point_count);
    get_dframe_moving_count(right_top_dframe,r_top_diff_point_count);
    get_dframe_moving_count(right_bottom_dframe,r_bottom_diff_point_count);

    bool l_top_is_moved;
    bool r_top_is_moved;
    bool l_bottom_is_moved;
    bool r_bottom_is_moved;

    //差分像素的占比和预定的阈值做比较
    float lt_dpoint_ratio=(float)l_top_diff_point_count/(float)l_top_pixel_count;
    float rt_dpoint_ratio=(float)r_top_diff_point_count/(float)r_top_pixel_count;
    float lb_dpoint_ratio=(float)l_bottom_diff_point_count/(float)l_bottom_pixel_count;
    float rb_dpoint_ratio=(float)r_bottom_diff_point_count/(float)r_bottom_pixel_count;

    cout<<lt_dpoint_ratio<<" "<<lb_dpoint_ratio<<" "<<rt_dpoint_ratio<<" "<<rb_dpoint_ratio<<endl;

    if(lt_dpoint_ratio>THRESHOLD_RATIO_LT)
        l_top_is_moved=true;
    else
        l_top_is_moved=false;
    if(rt_dpoint_ratio>THRESHOLD_RATIO_RT)
        r_top_is_moved=true;
    else
        r_top_is_moved=false;
    if(lb_dpoint_ratio>THRESHOLD_RATIO_LB)
        l_bottom_is_moved=true;
    else
        l_bottom_is_moved=false;
    if(rb_dpoint_ratio>THRESHOLD_RATIO_RB)
        r_bottom_is_moved=true;
    else
        r_bottom_is_moved=false;

    //状态机消除闪现
    if((l_top_is_moved||r_top_is_moved)&&(l_bottom_is_moved&&r_bottom_is_moved)){
        if(this_car_motion_state<3)
            this_car_motion_state++;
        else
            this_car_motion_state=3;
    }
    else{
        if(this_car_motion_state>0)
            this_car_motion_state--;
        else
            this_car_motion_state=0;
    }

    if(this_car_motion_state==3)
        state=true;
    if(this_car_motion_state==0)
        state=false;
    if(this_car_motion_state<3&&this_car_motion_state>0)
        state=last_this_car_motion_state;

    last_this_car_motion_state=state;
}

void VetThisCarMotionStateJudging_Frame_Diff::compute_horizontal_sum_variance_total(Mat &image_gray,
    float &l_top_total_variance,float &l_bottom_total_variance,float &r_top_total_variance,float &r_bottom_total_variance){
    Mat left_top_gray=image_gray(left_top_roi);
    Mat left_bottom_gray=image_gray(left_bottom_roi);
    Mat right_top_gray=image_gray(right_top_roi);
    Mat right_bottom_gray=image_gray(right_bottom_roi);

    l_top_total_variance=(float)compute_horizontal_sum_of_variance(left_top_gray)/(float)(left_top_gray.cols*left_top_gray.rows);
    l_bottom_total_variance=(float)compute_horizontal_sum_of_variance(left_bottom_gray)/(float)(left_bottom_gray.cols*left_bottom_gray.rows);
    r_top_total_variance=(float)compute_horizontal_sum_of_variance(right_top_gray)/(float)(right_top_gray.cols*right_top_gray.rows);
    r_bottom_total_variance=(float)compute_horizontal_sum_of_variance(right_bottom_gray)/(float)(right_bottom_gray.cols*right_bottom_gray.rows);
}

