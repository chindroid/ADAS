#include"vetsg_utils.h"

using namespace std;
using namespace cv;

//直方图均衡化
void equalizehist_transformation(Mat &image_gray){
    equalizeHist(image_gray,image_gray);
}

//中值滤波
void median_blur(Mat& image){
    medianBlur(image,image,9);
}

//双边滤波
void bilateral_blur(Mat& image){
    Mat image2;
    cv::bilateralFilter(image,image2, 25, 25*2, 25/2);
    image=image2.clone();
}

//计算平均灰度值
void compute_avg_gray(Mat image_gray, unsigned char &avg_of_gray){
    int rows=image_gray.rows;
    int cols=image_gray.cols;
    unsigned long long total_gray=0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            total_gray+=image_gray.at<unsigned char>(i,j);
        }
    }
    avg_of_gray=(unsigned char)((double)total_gray/(double)(rows*cols));
}

//显示运动UI图标
void draw_sg_moving(Mat &image,int x,int y){
    int width=15;
    int height=60;
    for(int i=0;i<width;i++)
        for(int j=0;j<height;j++){
            image.at<Vec3b>(y+j,x+i)[0]=0;
            image.at<Vec3b>(y+j,x+i)[1]=255;
            image.at<Vec3b>(y+j,x+i)[2]=0;
        }
    x+=35;
    for(int i=0;i<width;i++)
        for(int j=0;j<height;j++){
            image.at<Vec3b>(y+j,x+i)[0]=0;
            image.at<Vec3b>(y+j,x+i)[1]=255;
            image.at<Vec3b>(y+j,x+i)[2]=0;
        }
}

//显示静止UI图标
void draw_sg_stop(Mat &image,int x,int y){

    Point points[1][3];
    points[0][0] = Point( x+60, y+60 );
    points[0][1] = Point( x+60, y+120 );
    points[0][2] = Point( x+120, y+90 );

    const Point* pt[1] = { points[0] };
    int npt[1] = {3};
    polylines( image, pt, npt, 1, 1, Scalar(0,0,255)) ;
    fillPoly( image, pt, npt, 1, Scalar(0,0,255), 8);

}

//计算差分图像的差分像素
void get_dframe_moving_count(Mat dframe, int &count){
    for(int i=0;i<dframe.rows;i++)
        for(int j=0;j<dframe.cols;j++)
            if(dframe.at<unsigned char>(i,j)<=THRESHOLD_GRAY)
                dframe.at<unsigned char>(i,j)=0;
            else{
                count++;
                dframe.at<unsigned char>(i,j)=255;
            }
}

//计算ROI内水平方向像素差异性总和
long compute_horizontal_sum_of_variance(Mat &image_gray){
    int rows=image_gray.rows;
    int cols=image_gray.cols;
    long sum_of_variance=0;
    for(int i=0;i<rows;i++){
        unsigned char aver_pix;
        long total=0;
        for(int j=0;j<cols;j++){
            total+=(long)(image_gray.at<unsigned char>(i,j));
        }
        aver_pix=(unsigned char)((double)total/(double)cols);
        for(int j=0;j<cols;j++){
            sum_of_variance+=(long)abs(aver_pix-image_gray.at<unsigned char>(i,j));
        }
    }
    return sum_of_variance;
}
