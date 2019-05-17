#include "cluster.h"

Cluster::Cluster(){}

void Cluster::perform_linesegments_cluster(vector<vector<Linesegment> > vec_blocks_linesegments, vector<vector<Point> >& vec_point_for_poly,Mat& image){
    vector<vector<combined_line_vector> > blocks_line_vectors;//所有图像块中的代表向量
    for(int i=0;i<vec_blocks_linesegments.size();i++){
        vector<combined_line_vector> vec_of_line_vectors;//每个图像块中的所有代表向量
        for(vector<Linesegment>::iterator iter1=vec_blocks_linesegments[i].begin();iter1!=vec_blocks_linesegments[i].end();iter1++){
            if(iter1->clustered)//若该线段已经被聚类则跳过，寻找下一个没有被聚类的线段
                continue;
            iter1->clustered=true;
            vector<Linesegment> vec_lines;//聚成一簇的线段
            combined_line_vector line_vector;//该一簇线段所形成的的代表向量
            for(vector<Linesegment>::iterator iter2=vec_blocks_linesegments[i].begin()+1;iter2!=vec_blocks_linesegments[i].end();iter2++){
                if(iter2->clustered)
                    continue;
                //线段聚类成簇（向量）的条件
                if(
                 angle_between_lines(*iter1,*iter2)<0.05//线段之间的角度限制
                 &&compute_lines_distance(*iter1,*iter2)<200//线段之间的距离限制
                 &&compute_pt2pt_distance(iter1->centerp,iter2->centerp)<150//线段中心点之间的距离限制
                 &&fabsf(iter1->centerp.x-iter2->centerp.x)<50//线段横向坐标之间的差值限制
                  ){

                    iter2->clustered=true;
                    vec_lines.push_back(*iter2);
                }
            }
            vec_lines.push_back(*iter1);
            linesegments_to_vector(vec_lines,line_vector);
            vec_of_line_vectors.push_back(line_vector);
        }
        blocks_line_vectors.push_back(vec_of_line_vectors);
    }

////显示聚类完成的向量，圆圈代表向量的起始点
//    for(int i=0;i<blocks_line_vectors.size();i++){
//        for(int j=0;j<blocks_line_vectors[i].size();j++)
//            circle(image,blocks_line_vectors[i][j].startpoint,5,Scalar(255,0,0),4);
//    }

    vector<vector<combined_line_vector> > all_group_of_vector;//存放已经聚类完成的一个个簇

    //将向量聚类成曲线
    for(int i=0;i<blocks_line_vectors.size();i++){
        for(int j=0;j<blocks_line_vectors[i].size();j++){
            if(blocks_line_vectors[i][j].connected==true)
                continue;
            vector<combined_line_vector> group_of_vector;
            group_of_vector.push_back(blocks_line_vectors[i][j]);
            for(int k=1;i+k<blocks_line_vectors.size();k++){
                for(int h=0;h<blocks_line_vectors[i+k].size();h++){
                    if(blocks_line_vectors[i+k][h].connected==true)//若已经连接则跳过该向量
                        continue;

                    //将向量聚类成曲线的条件    group_of_vector[group_of_vector.size()-1]为该group中最末尾的一个向量
                    if(
                        fabsf(group_of_vector[group_of_vector.size()-1].angle-blocks_line_vectors[i+k][h].angle)<0.1//当前向量与该group中最末尾的一个向量的角度差距小于阈值
                        //当前向量与group中最末尾的向量的起始点连线的角度与这两个向量角度均值之间的差距小于阈值
                        &&0.05>fabsf((group_of_vector[group_of_vector.size()-1].angle+blocks_line_vectors[i+k][h].angle)/2-pt2pt_angle(group_of_vector[group_of_vector.size()-1].startpoint,blocks_line_vectors[i+k][h].startpoint))
                        &&fabsf(group_of_vector[group_of_vector.size()-1].startpoint.x-blocks_line_vectors[i+k][h].startpoint.x)<30//当前向量与该group中最末尾的一个向量的距离差距小于阈值
                    ){
                        group_of_vector.push_back(blocks_line_vectors[i+k][h]);
                        blocks_line_vectors[i+k][h].connected=true;//将该向量标记为已连接
                    }
                }
            }
            all_group_of_vector.push_back(group_of_vector);
        }
    }

//    for(int i=0;i<all_group_of_vector.size();i++){
//        for(int j=0;j<all_group_of_vector[i].size();j++)
//            circle(image,all_group_of_vector[i][j].startpoint,7,Scalar(255-50*i,50*i,80*i),5);
//    }

    extern int N_BLOCKS;
    extern int block_height;
    float vertical_skip_dist=N_BLOCKS*(float)block_height/2;

    //将已经局累成曲线的向量的再次进行筛选并分组保存为待拟合的点
    for(int i=0;i<all_group_of_vector.size();i++){
        if(
//           fabsf(all_group_of_vector[i][0].startpoint.y-all_group_of_vector[i][all_group_of_vector.size()-1].startpoint.y)>vertical_skip_dist&&
           all_group_of_vector[i].size()>=5
           ){
            vector<Point> vec_points;
            for(int j=0;j<all_group_of_vector[i].size();j++)
                vec_points.push_back(Point(all_group_of_vector[i][j].startpoint.x,all_group_of_vector[i][j].startpoint.y));
            vec_point_for_poly.push_back(vec_points);
        }
    }
//    cout<<"vec_size: "<<vec_point_for_poly.size()<<endl;

    //简单的实现车道偏离预警：若曲线最末端向量的点横纵坐标在指定范围内，则判定车道偏离，车道线显示为红色；否则显示为天蓝色
    for(int i=0;i<vec_point_for_poly.size();i++){
        if(vec_point_for_poly[i][vec_point_for_poly[i].size()-1].x<200||vec_point_for_poly[i][vec_point_for_poly[i].size()-1].x>300)
            polylines(image,vec_point_for_poly[i], false, cv::Scalar(255, 255, 0), 6, 8, 0);
        else
            polylines(image,vec_point_for_poly[i], false, cv::Scalar(0, 0, 255), 6, 8, 0);
    }

}

//用一簇线段计算该簇线段的代表向量
void Cluster::linesegments_to_vector(vector<Linesegment> vec_lines, combined_line_vector& line_vector){
    float aver_x=0;
    float aver_y=0;
    float aver_angle=0;
    //代表向量的起始点的x、y坐标以及方向角度取该簇中所有线段中心点的x、y坐标以及方向角度的均值
    for(int i=0;i<vec_lines.size();i++){
        aver_angle+=vec_lines[i].angle;
        aver_x+=vec_lines[i].centerp.x;
        aver_y+=vec_lines[i].centerp.y;
    }
    aver_angle=aver_angle/vec_lines.size();
    aver_x=aver_x/vec_lines.size();
    aver_y=aver_y/vec_lines.size();

    line_vector.angle=aver_angle;
    line_vector.startpoint.x=aver_x;
    line_vector.startpoint.y=aver_y;
}
