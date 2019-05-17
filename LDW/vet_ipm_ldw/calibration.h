#ifndef Calibration_H
#define Calibration_H
#include "global.h"

void initialize();

class Calibration
{
public:
    static Calibration *instance();//设计成单例模式类，因为后面有三处类里用到了这个类
    static void destoryInstance();
    bool initComprehensiveParameters();  //获取8个标定参数
    void pixel2Vehicle(StructPixelAndReal&);   //像素坐标到实际坐标变换
    void vehicle2Pixel(StructPixelAndReal&);   //实际坐标到像素坐标变换
    bool getAllPixelPoint(vector<Point2d>&); //获取所有点的像素坐标：标定圆的四个点及车前端左侧、右侧两个像素点坐标
    bool getAllRealPoint(vector<Point2d>&);//获取所有点的世界坐标：标定圆的四个点及车前端左侧、右侧两个世界坐标
    bool createPixelFile();//生成一个xml文件，里面包含对应的六个像素坐标，用来给后面的代码读取，在图像上计算距离，用来预警。

private:
    Calibration();

private:
    static Calibration *vet_ptr_calibration;
    double comprehensive_parameters[8]; //用来存放综合参数
};



#endif // Calibration

