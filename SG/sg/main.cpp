#include <iostream>
#include "vetstoporgo.h"
#include "vetsg_utils.h"

using namespace std;

bool ENABLE_SHOW_FPS=true;

cv::Rect FRONT_CAR_CENTER_ROI=Rect(420,400,400,300);//前车启停状态检测的ROI设定

int THRESHOLD_GRAY=20;//灰度二值化处理阈值

float THRESHOLD_RATIO_LT=0.01;//本车运动差分像素的占比阈值
float THRESHOLD_RATIO_RT=0.01;
float THRESHOLD_RATIO_LB=0.01;
float THRESHOLD_RATIO_RB=0.01;

int DEQUE_MAXSIZE=20;//前车近期差分像素占比队列的MAXSIZE
float THRESHOLD_RARIO_MITIPLE=5;//判断前车ROI差分像素占比的倍数阈值

int N_FRAME=4;//每隔N_FRAME帧抽取一帧作为SG功能模块的输入

unsigned char THRESHOLD_GRAY_NIGHT_PROCESSING=60;//开启夜晚图像增强模式的灰度阈值

int frame_width=1280;
int frame_height=720;

int main()
{
    VetStopOrGo sg;
    VideoCapture vc("HPIM0088.mov");
    Mat image;
    STU_SG_DATA sg_data;
    int count=0;
    //视频写入
    VideoWriter writer;
    writer.open("sg_show_video1.avi",CV_FOURCC('M','P','4','2'),30,Size(1280,720));


    //循环读入图像
    while(vc.read(image))
    {
        //启动SG
        sg.start(image,sg_data);
        imshow("1",image);

        char ret=waitKey(30);
//        //写入视频
//        if(ret=='e')
//            writer<<image;
//        else if(ret=='s')
//        {
//            writer.release();
//            return 0;
//        }
    }
    cout << "finished!" << endl;
    return 0;
}
