#include "vetstoporgo.h"

extern int N_FRAME;
extern int frame_width;
extern int frame_height;

VetStopOrGo::VetStopOrGo()
    :this_car_(frame_width,frame_height)
{
    front_car_stop_ = true;   //刚开始假设前车是静止的
    ftime(&last_remind_);
    count_ = 0;
    night_precess=false;
}

extern bool ENABLE_SHOW_FPS;

extern float THRESHOLD_RATIO_LT;
extern float THRESHOLD_RATIO_LB;
extern float THRESHOLD_RATIO_RT;
extern float THRESHOLD_RATIO_RB;

//图像预处理函数
void VetStopOrGo::pre_process(Mat &image, Mat &pre_image, Mat &dframe){

    //灰度化
    cv::cvtColor(image,image,CV_RGB2GRAY);
    cv::cvtColor(pre_image,pre_image,CV_RGB2GRAY);

    //动态阈值调整
    if(count_ %(10*N_FRAME)==0){
        float lt_hsv;float lb_hsv;float rt_hsv;float rb_hsv;
        this_car_.compute_horizontal_sum_variance_total(image,lt_hsv,lb_hsv,rt_hsv,rb_hsv);
        cout<<"l_t: "<<lt_hsv<<" l_b: "<<lb_hsv<<" rt_: "<<rt_hsv<<" rb_: "<<rb_hsv<<endl;
        THRESHOLD_RATIO_LB=lb_hsv/1500;
        THRESHOLD_RATIO_LT=lt_hsv/1500;
        THRESHOLD_RATIO_RB=rb_hsv/1500;
        THRESHOLD_RATIO_RT=rt_hsv/1500;
    }

    //是否进夜晚图像增强的判断
    if(count_%100==0){
        extern unsigned char THRESHOLD_GRAY_NIGHT_PROCESSING;
        unsigned char avg_gray;
        compute_avg_gray(image,avg_gray);
        if(avg_gray<THRESHOLD_GRAY_NIGHT_PROCESSING){
            night_precess=true;
        }
        else
            night_precess=false;
    }
    //直方图均衡化+中值滤波处理
    if(night_precess){
        equalizehist_transformation(image);
        equalizehist_transformation(pre_image);
        median_blur(image);
        median_blur(pre_image);
        cout<<"night_process: "<<endl;
    }

    imshow("3",image);
    //帧差图像获取
    cv::absdiff(image,pre_image,dframe);
}


//状态显示和提醒函数
int VetStopOrGo::remindDriver(Mat& image,STU_SG_DATA& sg_data)
{
//    rectangle(image,this_car_.left_bottom_roi,Scalar(0,0,255));
//    rectangle(image,front_car_.center_roi,Scalar(0,255,0));
    if(sg_data.this_car_state == true)
    {
        draw_sg_moving(image,60,60);
    }

    if(sg_data.this_car_state == false)
    {
        if(sg_data.front_car_state == true)
        {
            draw_sg_moving(image,660,360);
        }
        else
        {
            draw_sg_stop(image,600,300);
        }
    }

    if(ENABLE_SHOW_FPS)
    {
        char path[1024];
        sprintf(path,"sg_fps: %f",sg_data.fps);
        //putText(image,path,Point(1000,200),FONT_HERSHEY_TRIPLEX, 0.5 ,Scalar(0,0,0));
    }

}

int VetStopOrGo::start(Mat& frame,STU_SG_DATA& sg_data)
{
    Mat temp_frame=frame.clone();
    if(count_ % N_FRAME == 0)
    {
        if(pre_frame.empty()){
            pre_frame=frame.clone();
        }
        Mat dframe;
        pre_process(temp_frame,pre_frame,dframe);//图像预处理

        //本车运动状态检测模块输入帧差图像
        sg_data.this_car_state = true;
        this_car_.getThisCarMotionState(dframe,sg_data.this_car_state);
        //若本车静止，前车运动状态检测模块输入帧差图像
        if(sg_data.this_car_state == false)
        {
            front_car_.getFrontCarMotionState(dframe,sg_data.front_car_state);
        }
        //本车一旦开始运动，清空前车帧差状态队列
        else
        {
            front_car_.clear_aver_recent_ratio();
        }
        pre_frame=frame.clone();
    }
    //获取当前帧的SG数据，显示与提醒
    remindDriver(frame,sg_data);
    //帧数计数的循环
    if(count_!=65535)
        count_++;
    else
        count_=0;

    return 0;
}
