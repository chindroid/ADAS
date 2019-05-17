#ifndef LINESEGMENT
#define LINESEGMENT

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

//#define X_MAX 1280
//#define X_MIN 0
//#define Y_MAX 520
//#define Y_MIN 0

#define MIN_B 150
#define MIN_G 120
#define MIN_R 120

#define D_value_lineside_pix 20  //边界像素RGB通道的差值

class Linesegment{
public:
    cv::Point2f startp;//线段出发点
    cv::Point2f endp;//线段结束点
    cv::Point2f centerp;//线段的中心点
    float angle=0.5;//初始化线段的角度为90°
    float slope=99999999;//初始化斜率为无限大
    //k、b为当前线段所在直线的方程参数，其中k为斜率
    float k;//点斜式方程的k系数
    float b;//点斜式方程的b系数
    std::pair<cv::Vec3b,cv::Vec3b> side_color;//线段两侧的颜色信息
    std::pair<bool,bool> line_result;//first:是否为车道线、seconde：是否为车道线的左边界线

    cv::Mat* p_image;//线段所在的image
    //image的像素边界
        int X_MAX;//X坐标的最大边界
        int X_MIN;//X坐标的最小边界
        int Y_MAX;//Y坐标的最大边界
        int Y_MIN;//Y坐标的最小边界

    bool clustered=false;//标记该线段是否已经被聚类
public:
    Linesegment(cv::Point2f,cv::Point2f,cv::Mat &);
    void get_angle_and_slope();//预处理线段的数据,计算线段角度等,以x轴为0°角
    void get_line_average_pix();//计算直线两侧(左边和右边)的平均像素值，结果保存在类成员side_color中
    void judge_linesegment();//计算线段两端平均像素的差值是否大于一个阈值,每个通道分开计算,并且判断是左侧的车道线还是右侧的车道线，结果保存在line_result中

};

#endif // LINESEGMENT

