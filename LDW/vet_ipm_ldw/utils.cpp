#include "utils.h"

//生成二值化图像
void generate_binary_image(Mat& img,float threshold){//img为灰度图，threshold为亮度阈值
    int cols=img.cols;
    int rows=img.rows;

//    float sum;
//    for (int i = 0; i < rows; i++)
//    {
//        for (int j = 0; j < cols; j++)
//        {
////            float pix_value=pow(2,(double)img.at<unsigned char>(i,j)/32);
////            cout<<(float)img.at<unsigned char>(i, j)<<"->"<<pix_value<<endl;
//            sum=sum+img.at<unsigned char>(i,j);
//        }
//    }
//    float aver=sum/(cols*rows);
//    cout<<aver<<endl;
//    imshow("log_img",img);

//    equalizeHist(img,img);

    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            if(img.at<unsigned char>(i,j)>=threshold)//亮度阈值threshold
                img.at<unsigned char>(i,j)=255;
            else
                img.at<unsigned char>(i,j)=0;
        }
    }
}

//计算两条线段之间的距离：
float compute_lines_distance(Linesegment l1,Linesegment l2){
    float result;
    if(l1.angle==0.5&&l2.angle!=0.5){
        result=(fabsf(l1.startp.y-l1.startp.x*l2.k-l2.b)/sqrt(1*1+l2.k*l2.k)+fabsf(l1.endp.y-l1.endp.x*l2.k-l2.b)/sqrt(1*1+l2.k*l2.k))/2;
        return result;
    }
    if(l1.angle!=0.5&&l2.angle==0.5){
        result=(fabsf(l2.startp.y-l2.startp.x*l1.k-l1.b)/sqrt(1*1+l1.k*l1.k)+fabsf(l2.endp.y-l2.endp.x*l1.k-l1.b)/sqrt(1*1+l1.k*l1.k))/2;
        return result;
    }
    if(l1.angle==0.5&&l2.angle==0.5){
        result=fabsf(l1.centerp.x-l2.centerp.x);
        return result;
    }
    if(l1.angle!=0.5&&l2.angle!=0.5){
        result=(fabsf(l1.startp.y-l1.startp.x*l2.k-l2.b)/sqrt(1*1+l2.k*l2.k)+fabsf(l1.endp.y-l1.endp.x*l2.k-l2.b)/sqrt(1*1+l2.k*l2.k))/2;
        return result;
    }
}

//计算两条线段的夹角
float angle_between_lines(Linesegment l1,Linesegment l2){
    float result=fabsf(l1.angle-l2.angle);
    if(result>0.5)
        result=1-result;
    return result;
}

//计算点到点的距离
float compute_pt2pt_distance(Point2f p1,Point2f p2){
    float result;
    result=sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    return result;
}

//将线段src的所有信息拷贝给线段dst
void copy_line(Linesegment dst,Linesegment src){
    dst.angle=src.angle;
    dst.startp=src.startp;
    dst.endp=src.endp;
    dst.centerp=src.centerp;
    dst.k=src.k;
    dst.b=src.b;
    dst.line_result=src.line_result;
    dst.p_image=src.p_image;
    dst.side_color=src.side_color;
    dst.slope=src.slope;
}

//重叠线段、邻近相似线段的合并
void combine_lines(vector<Linesegment>& linesegments,Mat& image){
    for(vector<Linesegment>::iterator iter1=linesegments.begin();iter1!=linesegments.end();iter1++){
        for(vector<Linesegment>::iterator iter2=iter1+1;iter2!=linesegments.end();iter2++){
            if((angle_between_lines(*iter1,*iter2)<0.15)&&(compute_lines_distance(*iter1,*iter2)<5)&&(iter1->line_result.second==iter2->line_result.second)){
                vector<Point2f> four_point;
                four_point.push_back(iter1->startp);
                four_point.push_back(iter1->endp);
                four_point.push_back(iter2->startp);
                four_point.push_back(iter2->endp);
                float max_dst=0;
                pair<Point2f,Point2f> result_2_points;
                for(vector<Point2f>::iterator iter3=four_point.begin();iter3!=four_point.end();iter3++){
                    for(vector<Point2f>::iterator iter4=iter3+1;iter4!=four_point.end();iter4++)
                        if(compute_pt2pt_distance(*iter3,*iter4)>max_dst){
                            max_dst=compute_pt2pt_distance(*iter3,*iter4);
                            result_2_points.first=*iter3;
                            result_2_points.second=*iter4;
                        }
                }
                Linesegment ls=Linesegment(result_2_points.first,result_2_points.second,image);
                linesegments.erase(iter2);

                iter2--;
                copy_line(*iter1,ls);
            }
        }
    }
}

//过滤一个block中的线段
void lines_filter(vector<Linesegment>& linesegments){
    //在双重循环将区域块中的所有线段进行两两匹配，符合所有限制条件则保留，否则删除
    for(vector<Linesegment>::iterator iter=linesegments.begin();iter!=linesegments.end();iter++){
        bool pass=false;
        for(vector<Linesegment>::iterator iter2=linesegments.begin();iter2!=linesegments.end();iter2++){
//            if((fabs(compute_lines_distance(*iter,*iter2))<10)&&
//                    (iter->line_result.second+iter2->line_result.second==1))
                if(//过滤条件判断
                    (angle_between_lines(*iter,*iter2)<0.15)&&//两条线段之间的夹角限制条件
                    (fabs(compute_lines_distance(*iter,*iter2))>0)&&(fabs(compute_lines_distance(*iter,*iter2))<15)&&//两条线段之间的距离限制条件
                    (iter->line_result.second+iter2->line_result.second==1)//成对线的限制条件
                   )
                    pass=true;
        }
        if(pass)
            continue;
        else{
            linesegments.erase(iter);
            iter--;
            continue;
        }
    }
}

//计算点与点之间连线的angle
float pt2pt_angle(Point2f p1, Point2f p2){
    if(p1.x==p2.x)
        return 0.5;
    float slope=-(p1.y-p2.y)/(float)(p1.x-p2.x);
    float k=-slope;
    float angle=-atan(k)/CV_PI;
    if(angle<0)angle+=1;
    return angle;
}

//计算ROI区域的平均亮度
float compute_roi_aver_light_threshold(Mat img){
    float sum=0;
    int rows=img.rows;
    int cols=img.cols;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            sum=sum+img.at<unsigned char>(i,j);
        }
    }
    return 1.5*sum/(rows*cols);
}

