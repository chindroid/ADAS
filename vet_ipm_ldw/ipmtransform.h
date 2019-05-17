#ifndef IPMTRANSFORM_H
#define IPMTRANSFORM_H

#include "global.h"
#include "calibration.h"

class IpmTransform{
public:
    IpmTransform(Calibration* cali);
    ~IpmTransform();

public:
    Mat warpMatrix_src2ipm;//保存透视变换的变换矩阵,原图到俯视图
    Mat warpMatrix_ipm2src;//保存透视变换的变换矩阵,俯视图到原图
    Calibration* _calibration;
    bool _calibration_file_normal;//判断是否读取标定文件成功
    //计算逆透视变换的四个目标点
    void setROIParameters(Mat& imageData,Point2d& P1,Point2d& P2,Point2d& P3,Point2d& P4);
    void rectImage(Mat&);   //ROI截取
    bool executeipm(Mat&,Mat&,Point2d,Point2d,Point2d,Point2d);
    void pts_coordinate_transform(vector<vector<Point> > src,vector<vector<Point> > &dst,Mat );
};

#endif // IPMTRANSFORM_H

