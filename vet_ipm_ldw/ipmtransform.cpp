#include "ipmtransform.h"

IpmTransform::IpmTransform(Calibration* cali){
    _calibration=cali;
    _calibration_file_normal=_calibration->initComprehensiveParameters();
}

IpmTransform::~IpmTransform(){
    _calibration->destoryInstance();
}

string video_path;
string pixel_path;
string calibrationSettingFile_path;
float ROI_HEIGHT;
float ROI_WIDTH;

float roi_x0;
float roi_y0;
float roi_width;
float roi_height;

void IpmTransform::setROIParameters(Mat& imageData,Point2d& PO1,Point2d& PO2,Point2d& PO3,Point2d& PO4)
{

    int width = imageData.cols;
    int height = imageData.rows;


    vector<Point2d> pixel;
    if(true==_calibration->getAllPixelPoint(pixel))//如果有pixel.txt文件
    {
        //存储根据标定结果计算后得到的关于ROI区域的三个值，默认包含图像底端
        int roi_y = 0;
        int roi_left_x = 0;
        int roi_right_x = 0;
        //计算两条直线在像素坐标中的交点坐标（即，消失点），得出可截取的y值
        //l1:slope1=(y1-y2)/(x1-x2)=(y-y1)/(x-x1) => y=slope1*(x-x1)+y1
        Point2d p0 = pixel.at(0);

        Point2d p1 = pixel.at(1);

        Point2d p2= pixel.at(2);

        Point2d p3 = pixel.at(3);

        Point2d p4 = pixel.at(4);
        Point2d p5 = pixel.at(5);

        double car_forward = ROI_HEIGHT;//以毫米为单位，车前多少距离
        Point2d inter_vanish; //左右两侧标定圆心在图像上连线的交点
        double candidate_roi_y = 0;//根据car_forward处位置在图像上的对应点来得到一个候选的roi的y值位置
        StructPixelAndReal forward_left_real,forward_right_real;
        forward_left_real._pixel_coordinate = p4;
        forward_right_real._pixel_coordinate = p5;
        _calibration->pixel2Vehicle(forward_left_real);
        _calibration->pixel2Vehicle(forward_right_real);
        forward_left_real._real_coordinate.y = forward_left_real._real_coordinate.y+car_forward;//计算车头左侧前ROI_HEIGHT米的位置的世界坐标
        forward_right_real._real_coordinate.y = forward_right_real._real_coordinate.y+car_forward;//计算车头右侧前ROI_HEIGHT米的位置的世界坐标
        _calibration->vehicle2Pixel(forward_left_real);
        _calibration->vehicle2Pixel(forward_right_real);
        //candidate_roi_y取车头左右两侧ROI_HEIGHT处在图像中y值最大的
        if(forward_left_real._pixel_coordinate.y<=forward_right_real._pixel_coordinate.y)
        {
            candidate_roi_y = forward_right_real._pixel_coordinate.y;
        }
        else
        {
            candidate_roi_y = forward_left_real._pixel_coordinate.y;
        }
        //求消失点，并根据消失点的y值来确定roi_y，再加上candidate_roi_y的值，来确认最终的roi_y
        if(0!=(p0.x-p1.x)&&0!=(p2.x-p3.x))
        {
            double slope1 = (p0.y-p1.y)/(p0.x-p1.x);
            double slope2 = (p2.y-p3.y)/(p2.x-p3.x);
            if(slope1!=slope2)
            {
                inter_vanish.x = (p2.y-p0.y+slope1*p0.x-slope2*p2.x)/(slope1-slope2);
                inter_vanish.y = slope1*(inter_vanish.x-p0.x)+p0.y;
                if(0<=inter_vanish.y&&inter_vanish.y<height)
                {
                    roi_y = inter_vanish.y;
                }
                else
                {
                    roi_y = 0;
                }
                //再将roi_y与candidate_roi_y比较，取值大的那位
                if(roi_y<candidate_roi_y)
                {
                    roi_y = candidate_roi_y;
                }
            }
            else
            {
                 _calibration_file_normal = false;
            }
        }
        else
        {
             _calibration_file_normal = false;
        }
        //以N宽为限度（按照国标，一般道路宽度为3.0~3.75m）
        double width_y = ROI_WIDTH;//以毫米为单位，车左右两侧多少距离
        StructPixelAndReal left_pixel1,right_pixel1,left_pixel2,right_pixel2;
        double inter_left_x,inter_right_x;
        left_pixel1._real_coordinate = Point2d(-width_y/2.0,0);
        right_pixel1._real_coordinate = Point2d(width_y/2.0,0);
        left_pixel2._real_coordinate = Point2d(-width_y/2.0,width_y);
        right_pixel2._real_coordinate = Point2d(width_y/2.0,width_y);
        _calibration->vehicle2Pixel(left_pixel1);
        _calibration->vehicle2Pixel(right_pixel1);
        _calibration->vehicle2Pixel(left_pixel2);
        _calibration->vehicle2Pixel(right_pixel2);
        Point2d a1 = left_pixel1._pixel_coordinate;
        Point2d a2 = left_pixel2._pixel_coordinate;
        Point2d b1 = right_pixel1._pixel_coordinate;
        Point2d b2 = right_pixel2._pixel_coordinate;
        //求左右两侧标定圆心在图像上的连线分别与图像底部的交点，以此来确定roi_left_x和roi_right_x
        //l1:slope1=(y1-y2)/(x1-x2)=(y-y1)/(x-x1) => y=slope1*(x-x1)+y1，求两条直线，在y=height时，x的值
        if(0!=(a1.x-a2.x)&&0!=(b1.x-b2.x))
        {
            double slope1 = (a1.y-a2.y)/(a1.x-a2.x);
            double slope2 = (b1.y-b2.y)/(b1.x-b2.x);
            if(0!=slope1&&0!=slope2)
            {
                inter_left_x = (height-1-a1.y)/slope1+a1.x;
                inter_right_x = (height-1-b1.y)/slope2+b1.x;

                if(0<=inter_left_x&&inter_left_x<width)
                {
                    roi_left_x = inter_left_x;
                }
                else
                {
                    roi_left_x = 0;
                }

                if(0<=inter_right_x&&inter_right_x<width)
                {
                    roi_right_x = inter_right_x;
                }
                else
                {
                    roi_right_x = width-1;
                }
            }
            else
            {
                _calibration_file_normal = false;
            }
        }
        else
        {
            _calibration_file_normal = false;
        }

        if(true==_calibration_file_normal)
        {

            //赋值
            roi_x0 = roi_left_x;
            roi_y0 = roi_y;
            roi_width = roi_right_x-roi_left_x+1;//因为_roi_right_x和_roi_left_x是边界的坐标值，计算他们之间所占宽度时，应+1
            roi_height = height-roi_y;

            //计算两条线与ROI顶部的交点的横坐标
            double px1,px2;
            px1 = (roi_y-p0.y)*(p1.x-p0.x)/(p1.y-p0.y)+p0.x;
            px2 = (roi_y-p2.y)*(p3.x-p2.x)/(p3.y-p2.y)+p2.x;

            //使用标定的四个点
            PO1=pixel.at(0);
            PO1.y = PO1.y-roi_y;

            PO2.x = px1;
            PO2.y = 0;

            PO3.x = px2;
            PO3.y = 0;

            PO4=pixel.at(3);
            PO4.y = PO4.y-roi_y;

            PO1.x=PO1.x-roi_x0;
            PO2.x=PO2.x-roi_x0;
            PO3.x=PO3.x-roi_x0;
            PO4.x=PO4.x-roi_x0;

            ////仅仅显示用
            Mat roi_point_dst = imageData.clone();
            //画出ROI的矩形框  红色框
            line(roi_point_dst,Point(roi_left_x,roi_y),Point(roi_right_x,roi_y),Scalar(0,0,255),2);
            line(roi_point_dst,Point(roi_right_x,roi_y),Point(roi_right_x,height),Scalar(0,0,255),2);
            line(roi_point_dst,Point(roi_right_x,height),Point(roi_left_x,height),Scalar(0,0,255),2);
            line(roi_point_dst,Point(roi_left_x,height),Point(roi_left_x,roi_y),Scalar(0,0,255),2);
            //左右两侧标定圆心的连线及其相交点  红线
            line(roi_point_dst,pixel.at(0),Point(inter_vanish.x,inter_vanish.y),Scalar(0,0,255),4);
            line(roi_point_dst,pixel.at(3),Point(inter_vanish.x,inter_vanish.y),Scalar(0,0,255),4);
            circle(roi_point_dst,Point(inter_vanish.x,inter_vanish.y),1,Scalar(0,255,255),15);
            //显示5m宽位置的四个点，连接底下两个点   绿线
            line(roi_point_dst,a1,b1,Scalar(0,255,0),2);
            circle(roi_point_dst,a1,1,Scalar(0,255,0),10);
            circle(roi_point_dst,b1,1,Scalar(0,255,0),10);
            circle(roi_point_dst,a2,1,Scalar(0,255,0),10);
            circle(roi_point_dst,b2,1,Scalar(0,255,0),10);
            //消失点与5m宽位置的两个点之间的连线  绿线
            line(roi_point_dst,a1,Point(inter_vanish.x,inter_vanish.y),Scalar(0,255,0),2);
            line(roi_point_dst,b1,Point(inter_vanish.x,inter_vanish.y),Scalar(0,255,0),2);
            //车头两侧点与消失点的连线   黄线
            line(roi_point_dst,pixel.at(4),Point(inter_vanish.x,inter_vanish.y),Scalar(0,255,255),2);
            line(roi_point_dst,pixel.at(5),Point(inter_vanish.x,inter_vanish.y),Scalar(0,255,255),2);
            circle(roi_point_dst,pixel.at(4),1,Scalar(0,255,0),10);
            circle(roi_point_dst,pixel.at(5),1,Scalar(0,255,0),10);
            //显示标定圆心点及车头两个点在图像中的像素坐标位置  //红线
            for(int i=0;i<pixel.size();i++)
            {
                circle(roi_point_dst,pixel.at(i),1,Scalar(0,0,255),10);
            }
            imshow("ORI_roi",roi_point_dst);
        }
    }
    else
    {
        cout<<"cannot find pixel file and cannnot get ROI!"<<endl;
    }
    rectImage(imageData);
}

void IpmTransform::rectImage(Mat& imageData)
{
    //根据roi_y，roi_left_x，roi_right_x这三个值来截取ROI区域
    Mat m_temp(imageData,Rect(roi_x0,roi_y0,roi_width,roi_height));//裁剪原始图片大小，确定ROI，Rect的四个参数：左上角x，y，宽度，高度
    //20180330 -show only
//    cout<<_roi_x0<<" "<<_roi_y0<<" "<<_roi_width<<" "<<_roi_height<<endl;

    imageData = m_temp.clone();
}


double compute_P2P_dist(StructPixelAndReal sp1,StructPixelAndReal sp2){
    return sqrt(pow(sp1._real_coordinate.x-sp2._real_coordinate.x,2)+pow(sp1._real_coordinate.y-sp2._real_coordinate.y,2));
}

bool IpmTransform::executeipm(Mat& img,Mat& dst,Point2d P1,Point2d P2,Point2d P3,Point2d P4)
{
    if (img.data)
    {
        Point2f corners[4];

        extern float roi_x0;
        extern float roi_y0;

        extern float ROI_HEIGHT;
        extern float ROI_WIDTH;

        corners[0] = P2;
        corners[1] = P3;
        corners[2] = P1;
        corners[3] = P4;
//        for(int i=0;i<4;i++)
//            circle(img,corners[i],3,Scalar(0,255,255),4);
//        cout<<"roi_x: "<<roi_x0<<" roi_y:"<<roi_y0<<endl;
//        cout<<P2.x<<" "<<P2.y<<endl<<P3.x<<" "<<P3.y<<endl<<P1.x<<" "<<P1.y<<endl<<P4.x<<" "<<P4.y<<endl;

//        StructPixelAndReal sp1;//P1的像素坐标到世界坐标
//        StructPixelAndReal sp2;//P2的像素坐标到世界坐标
//        StructPixelAndReal sp3;//P3的像素坐标到世界坐标
//        StructPixelAndReal sp4;//P4的像素坐标到世界坐标
//        sp1._pixel_coordinate=Point2d(P1.x+roi_x0,P1.y+roi_y0);
//        sp2._pixel_coordinate=Point2d(P2.x+roi_x0,P2.y+roi_y0);
//        sp3._pixel_coordinate=Point2d(P3.x+roi_x0,P3.y+roi_y0);
//        sp4._pixel_coordinate=Point2d(P4.x+roi_x0,P4.y+roi_y0);
//       //获取世界坐标
//        _calibration->pixel2Vehicle(sp1);
//        _calibration->pixel2Vehicle(sp2);
//        _calibration->pixel2Vehicle(sp3);
//        _calibration->pixel2Vehicle(sp4);

//        //计算P1、P4到P2、P3的平均世界距离
//        double vertical_dist=(compute_P2P_dist(sp1,sp2)+compute_P2P_dist(sp3,sp4))/2;
//        double horizontal_dist=(compute_P2P_dist(sp1,sp4)+compute_P2P_dist(sp2,sp3))/2;

        float IPM_WIDTH=500;
        float N=5;
        //保证俯视图的宽度大概为N个车头宽
        float sacale=(IPM_WIDTH/N)/ROI_WIDTH;
        float IPM_HEIGHT=ROI_HEIGHT*sacale;

        //俯视图初始化
        dst=Mat::zeros(IPM_HEIGHT+50,IPM_WIDTH,img.type());

        Point2f corners_trans[4];
        corners_trans[0] = Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),0);  //P2
        corners_trans[1] = Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),0);  //P3
        corners_trans[2] = Point2f(IPM_WIDTH/2-IPM_WIDTH/(2*N),IPM_HEIGHT);   //P1
        corners_trans[3] = Point2f(IPM_WIDTH/2+IPM_WIDTH/(2*N),IPM_HEIGHT);   //P4

        //计算原图到俯视图和俯视图到原图的变换矩阵
        warpMatrix_src2ipm = getPerspectiveTransform(corners, corners_trans);
        warpMatrix_ipm2src = getPerspectiveTransform(corners_trans,corners);
        warpPerspective(img, dst, warpMatrix_src2ipm, dst.size());
    }
    else
    {
        cout << "NO IMAGE!!!" << endl;
        return false;
    }

    return true;
}

//透视图和原图之间的点坐标的转换
void IpmTransform::pts_coordinate_transform(vector<vector<Point> > src, vector<vector<Point> > &dst, Mat warpMatrix){

    double a11=warpMatrix.at<double>(0,0);double a12=warpMatrix.at<double>(0,1);double a13=warpMatrix.at<double>(0,2);
    double a21=warpMatrix.at<double>(1,0);double a22=warpMatrix.at<double>(1,1);double a23=warpMatrix.at<double>(1,2);
    double a31=warpMatrix.at<double>(2,0);double a32=warpMatrix.at<double>(2,1);double a33=warpMatrix.at<double>(2,2);
//    cout<<a11<<" "<<a12<<" "<<a13<<" "<<a21<<" "<<a22<<" "<<a23<<" "<<a31<<" "<<a32<<" "<<a33<<endl;
    for(int i=0;i<src.size();i++){
        vector<Point> vec_pts;
        for(int j=0;j<src[i].size();j++){
//            cout<<"u v:"<<src[i][j]<<endl;
            double x=(a11*src[i][j].x+a21*src[i][j].y+a31)/(a13*src[i][j].x+a23*src[i][j].y+a33);
            double y=(a12*src[i][j].x+a22*src[i][j].y+a32)/(a13*src[i][j].x+a23*src[i][j].y+a33);
//            cout<<x<<" "<<y<<endl;
            Point p=Point(x,y);
            vec_pts.push_back(p);
        }
        dst.push_back(vec_pts);
    }
}
