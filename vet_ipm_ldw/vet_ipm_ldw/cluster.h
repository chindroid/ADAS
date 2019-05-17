#ifndef CLUSTER_H
#define CLUSTER_H

//#include "global.h"
//#include "utils.h"
#include "linesegment.h"
#include "utils.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//经过线段聚类合并形成的向量，每个向量代表着一簇线段
struct combined_line_vector{
    float angle;//向量方向角度
    Point2f startpoint;//向量起始点
    bool connected=false;//是否被连接（向量聚类成为曲线）
};

//用于线段聚类的类
class Cluster{
public:
    Cluster();
    void perform_linesegments_cluster(vector<vector<Linesegment> >,vector<vector<Point> >&,Mat&);
    void linesegments_to_vector(vector<Linesegment>,combined_line_vector& );
};

#endif // CLUSTER_H

