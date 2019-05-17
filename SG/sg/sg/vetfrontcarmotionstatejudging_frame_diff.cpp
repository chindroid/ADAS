#include "vetfrontcarmotionstatejudging_frame_diff.h"

using namespace std;
using namespace cv;

VetFontCarMotionStateJudging_Frame_Diff::VetFontCarMotionStateJudging_Frame_Diff(){
    aver_recent_ratio=0;
    extern Rect FRONT_CAR_CENTER_ROI;
    center_roi=FRONT_CAR_CENTER_ROI;
}

void VetFontCarMotionStateJudging_Frame_Diff::getFrontCarMotionState(cv::Mat &dframe_total, bool &front_car_ismoved){

    Mat dframe=(dframe_total(center_roi)).clone();
    //make_trapezoid(dframe);

    int c_point_diff_count=0;

    get_dframe_moving_count(dframe,c_point_diff_count);

//    cout<<recent_front_car_diff_ratio.size()<<endl;
    imshow("2",dframe);
    int c_pixel_count=dframe.cols*dframe.rows;
    float curr_ratio=(float)c_point_diff_count/(float)c_pixel_count;

//    cout<<"curr_ratio "<<curr_ratio<<endl;
//    cout<<"aver_ratio "<<aver_recent_ratio<<endl;

    //当前前车运动状态的阈值
    if(curr_ratio>THRESHOLD_RARIO_MITIPLE*aver_recent_ratio&&recent_front_car_diff_ratio.size()==DEQUE_MAXSIZE){
        front_car_ismoved=true;
    }
    else
        front_car_ismoved=false;
//    cout<<"front car state:"<<front_car_ismoved<<endl;
    if(!front_car_ismoved){
        if(recent_front_car_diff_ratio.size()<DEQUE_MAXSIZE){
            recent_front_car_diff_ratio.push_front(curr_ratio);
            update_aver_recent_ratio();
        }

        else{
            recent_front_car_diff_ratio.pop_back();
            recent_front_car_diff_ratio.push_front(curr_ratio);
            update_aver_recent_ratio();
        }
    }
}


void VetFontCarMotionStateJudging_Frame_Diff::make_trapezoid(Mat &center_frame){
    int width=center_frame.cols;
    int height=center_frame.rows;
    //cout<<width<<" "<<height<<endl;
    for(int i=0;i<height;i++){
        for(int j=(height-i)+width/3;j>=0;j--)
            center_frame.at<unsigned char>(i,j)=0;
        for(int k=3*width-(height-i+width/3);k<3*width;k++)
            center_frame.at<unsigned char>(i,k)=0;
    }
    for(int i=0;i<height/5;i++)
        for(int j=0;j<width*3;j++)
            center_frame.at<unsigned char>(i,j)=0;
}

void VetFontCarMotionStateJudging_Frame_Diff::update_aver_recent_ratio(){
    deque<float>::iterator iter;
    float sum=0;
    for(iter=recent_front_car_diff_ratio.begin();iter!=recent_front_car_diff_ratio.end();iter++)
        sum+=*iter;
    aver_recent_ratio=sum/recent_front_car_diff_ratio.size();
}

void VetFontCarMotionStateJudging_Frame_Diff::clear_aver_recent_ratio(){
    recent_front_car_diff_ratio.clear();
}



