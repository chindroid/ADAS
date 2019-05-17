#ifndef GLOBAL_H
#define GLOBAL_H

#include <cmath>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

extern string video_path;
extern string pixel_path;
extern string calibrationSettingFile_path;
extern float ROI_HEIGHT;
extern float ROI_WIDTH;

extern float roi_x0;
extern float roi_y0;
extern float roi_width;
extern float roi_height;

struct StructPixelAndReal
{
    Point2d _pixel_coordinate;//像素坐标
    Point2d _real_coordinate;//世界坐标
};

//程序参数设定初始化


#endif // GLOBAL_H

