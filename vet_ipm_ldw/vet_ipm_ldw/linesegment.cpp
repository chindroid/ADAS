#include "linesegment.h"

using namespace std;
using namespace cv;

extern int block_height;
extern int block_width;

Linesegment::Linesegment(cv::Point2f startp_, cv::Point2f endp_, cv::Mat &image_){
    startp.x=startp_.x;
    startp.y=startp_.y;
    endp.x=endp_.x;
    endp.y=endp_.y;
    p_image=&image_;
    X_MAX=block_width;
    X_MIN=0;
    Y_MAX=block_height;
    Y_MIN=0;
    get_angle_and_slope();
    get_line_average_pix();
    judge_linesegment();
}

//预处理线段的数据,计算线段角度,以x轴为0°角
void Linesegment::get_angle_and_slope(){
    centerp.x=(startp.x+endp.x)/2;centerp.y=(startp.y+endp.y)/2;
    if(startp.x==endp.x)
        return ;
    slope=-(startp.y-endp.y)/(float)(startp.x-endp.x);
    k=-slope;
    b=startp.y-startp.x*k;
    angle=-atan(k)/CV_PI;
    if(angle<0)angle+=1;

}

//计算直线两侧(左边和右边)的平均像素值
void Linesegment::get_line_average_pix(){
    int bios_x=-4;
    int bios_y=0;
//    if((angle>-0.5&&angle<-0.333)||(angle>0.333&&angle<0.5))
//        bios_x=-2,bios_y=0;
//    if(angle>-0.333&&angle<-0.166)
//        bios_x=-1,bios_y=1;
//    if(angle>-0.166&&angle<0)
//        bios_x=0,bios_y=2;
//    if(angle>0&&angle<0.166)
//        bios_x=0,bios_y=-2;
//    if(angle>0.166&&angle<0.333)
//        bios_x=-1,bios_y=-1;
////    if((l.angle>-0.5&&l.angle<-0.333)||(l.angle>0.333&&l.angle<0.5))
////        bios_x=-4,bios_y=0;
////    if(l.angle>-0.333&&l.angle<-0.166)
////        bios_x=-2,bios_y=2;
////    if(l.angle>-0.166&&l.angle<0)
////        bios_x=0,bios_y=4;
////    if(l.angle>0&&l.angle<0.166)
////        bios_x=0,bios_y=-4;
////    if(l.angle>0.166&&l.angle<0.333)
////        bios_x=-2,bios_y=-2;
    Vec3b left;
    Vec3b right;
    for(int i=0;i<3;i++){
        int left_sum=0;
        int right_sum=0;
        LineIterator lit(*p_image,startp,endp);
        int real_count=lit.count;
        for(int j=0;j<lit.count;j++,lit++){
            Point pt=Point(lit.pos());
            int rbx=pt.x-bios_x;
            int rby=pt.y-bios_y;
            int lbx=pt.x+bios_x;
            int lby=pt.y+bios_y;

//            circle(*p_image,Point(rbx,rby),1,Scalar(0,0,255),1);
//            circle(*p_image,Point(lbx,lby),1,Scalar(0,0,255),1);
            //直线平移之后的越界检查
            if(lbx<X_MAX&&lbx>X_MIN&&lby<Y_MAX&&lby>Y_MIN&&rbx<X_MAX&&rbx>X_MIN&&rby<Y_MAX&&rby>Y_MIN){
            }
            else{
                real_count--;
                continue;
            }
            left_sum+=(*p_image).at<Vec3b>(lby,lbx)[i];
            right_sum+=(*p_image).at<Vec3b>(rby,rbx)[i];
        }
//        cout<<left_sum<<" "<<right_sum<<" real_count: "<<real_count<<endl;
        left[i]=(int)(left_sum/(float)real_count);
        right[i]=(int)(right_sum/(float)real_count);
    }
    side_color.first=left;
    side_color.second=right;
}

//Vec3b 通道0、1、2分别为B、G、R
//首先计算直线两边的像素值判断颜色信息，排除没有白色或者黄色的线段
//计算线段两端平均像素的差值是否大于一个阈值,每个通道分开计算,并且判断是左侧的车道线还是右侧的车道线，结果保存在line_result中
void Linesegment::judge_linesegment(){
//    颜色判断
    if(!((side_color.first[0]>MIN_B&&side_color.first[1]>MIN_G&&side_color.first[2]>MIN_R)||(side_color.second[0]>MIN_B&&side_color.second[1]>MIN_G&&side_color.second[2]>MIN_R))){
        line_result.first=false;
        line_result.second=false;
        return;
    }
    if((side_color.first[0]-side_color.second[0]>D_value_lineside_pix)&&(side_color.first[1]-side_color.second[1]>D_value_lineside_pix)&&(side_color.first[2]-side_color.second[2]>D_value_lineside_pix)){
        line_result.first=true;
        line_result.second=true;
    }
    else{
        if((side_color.second[0]-side_color.first[0]>D_value_lineside_pix)&&(side_color.second[1]-side_color.first[1]>D_value_lineside_pix)&&(side_color.second[2]-side_color.first[2]>D_value_lineside_pix)){
            line_result.first=true;
            line_result.second=false;
        }
        else{
            line_result.first=false;
            line_result.second=false;
        }
    }
}
