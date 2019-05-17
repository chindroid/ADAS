#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include "linesegment.h"
#include "global.h"

using namespace cv;
using namespace std;

//生成二值化图像
void generate_binary_image(Mat& img,float threshold);

//计算两条线段之间的距离：
float compute_lines_distance(Linesegment l1,Linesegment l2);

//计算两条线段的夹角
float angle_between_lines(Linesegment l1,Linesegment l2);

//计算点到点的距离
float compute_pt2pt_distance(Point2f p1,Point2f p2);

//将线段src的所有信息拷贝给线段dst
void copy_line(Linesegment dst,Linesegment src);

//重叠线段、邻近相似线段的合并
void combine_lines(vector<Linesegment>& linesegments,Mat& image);

//过滤一个block中的线段
void lines_filter(vector<Linesegment>& linesegments);

//计算两点连线的angle
float pt2pt_angle(Point2f,Point2f);

//计算ROI区域的平均亮度
float compute_roi_aver_light_threshold(Mat img);

#endif // UTILS_H

