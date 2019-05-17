/*
* main.cpp
*
* @author [作者]
* @version [版本号, YYYY-MM-DD]
* @see [相关类/方法]
* @since [产品/模块版本]
* @deprecated
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include "calibration.h"
#include "global.h"
#include "ipmtransform.h"
#include "utils.h"
#include "linesegment.h"
#include "cluster.h"

using namespace std;
using namespace cv;

int N_BLOCKS=20;//将鸟瞰图按照垂直方向划分为N_BLOCKS个区域块
int block_height;//每个区域块的像素高度
int block_width;//每个区域块的像素宽度

int main(){

    Mat frame;
    Mat frame_copy;
    Point2d P1,P2,P3,P4;//用于透视变换的四个点
    initialize();//模块初始化
    Calibration* calibration=Calibration::instance();//标定类实例化
    IpmTransform ipm(calibration);//逆透视变换类实例化
    Cluster clr;//聚类类实例化

    ifstream videolist;
    videolist.open((video_path+"list.txt").c_str());
    if(!videolist.is_open()){
        cout<<"no videolist"<<endl;
        return 0;
    }

    VideoCapture cap;
    string videoname;

    VideoWriter writer;
    writer.open("D:\\video\\LDW_VIDEO\\show_video1.avi",CV_FOURCC('M','P','4','2'),30,Size(1280,720));

    while(getline(videolist, videoname)){

        videoname=video_path+videoname;
        cap.open(videoname);

        cout<<"testing   "<<videoname<<endl;
        while(true)
        {
            cap >> frame;
            if(frame.empty()){
                break;
            }
            frame_copy = frame.clone();
            //计算ROI区域的平均亮度
            float light_threshold=compute_roi_aver_light_threshold(frame_copy);
//            cout<<"aver_threshold: "<<light_threshold<<endl;
            ipm.setROIParameters(frame,P1,P2,P3,P4);//计算逆透视变换的参数以及参照点
            imshow("ROI",frame);

            //执行逆透视变换
            Mat ipm_img;
            ipm.executeipm(frame,ipm_img,P1,P2,P3,P4);


            //将鸟瞰图按照垂直方向划分为N_BLOCKS个区域块
            Mat ipm_copy=ipm_img.clone();
            int ipm_img_rows=ipm_img.rows;
            block_height=(float)ipm_img_rows/N_BLOCKS;
            block_width=ipm_img.cols;
            vector<Mat> M_blocks;
            for(int i=0;i<N_BLOCKS;i++){
                Rect r=Rect(0,0+block_height*i,block_width,block_height);
                Mat m=ipm_copy(r);
//                rectangle(ipm_copy,r,Scalar(255,255,0),2);//显示区域块的划分
                M_blocks.push_back(m);
            }

            vector<vector<Linesegment> > vec_blocks_linesegments;

            //对每个区域块进行直线检测
            for(int i=0;i<N_BLOCKS;i++){

                Mat b_copy=M_blocks[i].clone();
                cvtColor(b_copy,b_copy,CV_RGB2GRAY);//灰度化
                generate_binary_image(b_copy,light_threshold);//二值化
                blur(b_copy,b_copy,Size(5,5));//均值滤波
                Canny(b_copy,b_copy,30,60);//Canny边缘检测

                vector<Vec4i> Lines;
                Lines.clear();
                HoughLinesP(b_copy, Lines, 1, CV_PI / 180,10,10,5);//霍夫直线检测

                vector<Linesegment> linesegments;//存放霍夫直线检测到的直线
                linesegments.clear();
                for(int j=0;j<Lines.size();j++){
                    Linesegment ls=Linesegment(Point2f(Lines[j][0],Lines[j][1]),Point2f(Lines[j][2],Lines[j][3]),M_blocks[i]);
                    if(ls.line_result.first&&ls.centerp.x>2&&ls.centerp.x<block_width-2)//不是车道线和ROI边缘的线段过滤掉
                        linesegments.push_back(ls);
                }

                //车道线过滤
                lines_filter(linesegments);
//                combine_lines(linesegments,M_blocks[i]);

                //将按区域块进行过滤之后的线段的坐标重新映射到整个鸟瞰图的坐标
                for(int j=0;j<linesegments.size();j++){
                    linesegments[j].startp.y+=i*block_height;
                    linesegments[j].centerp.y+=i*block_height;
                    linesegments[j].endp.y+=i*block_height;
                }
                vec_blocks_linesegments.push_back(linesegments);

            }
////显示平行成对线，车道线左侧线为绿色，车道线右侧线为红色
//            for(int i=0;i<vec_blocks_linesegments.size();i++){
//                for(int j=0;j<vec_blocks_linesegments[i].size();j++){
//                    if(vec_blocks_linesegments[i][j].line_result.second){
//                        line(ipm_copy,vec_blocks_linesegments[i][j].startp,vec_blocks_linesegments[i][j].endp,Scalar(0,0,255),2);
//                    }
//                    else{
//                        line(ipm_copy,vec_blocks_linesegments[i][j].startp,vec_blocks_linesegments[i][j].endp,Scalar(0,255,0),2);
//                    }
//                }
//            }

            vector<vector<Point> > points_to_polyfitting;//存放用来拟合成曲线的点
            clr.perform_linesegments_cluster(vec_blocks_linesegments,points_to_polyfitting,ipm_copy);//将线段聚类，结果保存成线段的中心点

            imshow("ipm_copy",ipm_copy);
//            imshow("ipm_img",ipm_img);

            warpPerspective(ipm_copy,frame_copy,ipm.warpMatrix_ipm2src,frame_copy.size());//将鸟瞰图转成原视角图
            vector<vector<Point> > src_points_to_polyfitting;
            ipm.pts_coordinate_transform(points_to_polyfitting,src_points_to_polyfitting,ipm.warpMatrix_ipm2src);

////            cout<<src_points_to_polyfitting[0].size()<<" "<<src_points_to_polyfitting[1].size()<<endl;
//            for(int i=0;i<src_points_to_polyfitting.size();i++){
//                for(int j=0;j<src_points_to_polyfitting[i].size();j++){
////                    cout<<"coordinate:"<<src_points_to_polyfitting[i][j].x<<" "<<src_points_to_polyfitting[i][j].y<<endl;
//                    circle(frame_copy,src_points_to_polyfitting[i][j],4,Scalar(123,123,123),3);
//                }
//                polylines(frame_copy,src_points_to_polyfitting[i], false, cv::Scalar(255, 255, 0), 6, 8, 0);
//            }

            imshow("frame_copy",frame_copy);


            //===================效果图展示代码===========================
            Mat src_copy=frame_copy.clone();
            resize(src_copy,src_copy,Size(640,360));
            int show_img_height=ipm_copy.rows;
            int show_img_width=640+ipm_copy.cols;
            Mat show_img=Mat::zeros(show_img_height+1,show_img_width+1,ipm_copy.type());
            for(int i=0;i<show_img_height;i++){
                for(int j=0;j<ipm_copy.cols;j++)
                    show_img.at<Vec3b>(i,j+show_img.cols-ipm_copy.cols)=ipm_copy.at<Vec3b>(i,j);
            }
            for(int i=0;i<360;i++){
                for(int j=0;j<640;j++)
                    show_img.at<Vec3b>(i+show_img_height/2-180,j)=src_copy.at<Vec3b>(i,j);
            }

            imshow("show",show_img);
            writer<<show_img;
            //===================效果图展示代码===========================


            //按c键当前帧写入视频、e键快进、s键停止并保存视频
            char ret=waitKey(30);
            if(ret=='c'){
                resize(show_img,show_img,Size(1280,720));
                writer<<show_img;
            }
            if(ret=='e'){
                for(int i=0;i<10;i++)cap>>frame;
            }

            if(ret=='s'){
                writer.release();
                return 0;
            }

            int key = waitKey(10);//1000/fps
            if( 32 == char(key))   // ‘空格’：停止
            {
                waitKey(0);
            }
            else if( 'f' == char(key))  // ‘f’:快进 steps 帧
            {
                int steps = 100;
                while(steps > 0)
                {
                    cap >> frame;
                    steps--;
                }
            }
        }
        cout<<"finished testing   "<<videoname<<endl;
    }

    return 0;
}



