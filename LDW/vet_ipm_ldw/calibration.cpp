
#include<calibration.h>

using namespace std;
using namespace cv;

void initialize(){
    //读取设置文件
    FileStorage _fs("file\\SettingFile.xml", FileStorage::READ);
    if(!_fs.isOpened())
    {
        cout << "failed to open SettingFile.xml" << endl;
    }
    _fs["video_path"]>>video_path;
    _fs["calibrationSettingFile_path"]>>calibrationSettingFile_path;
    _fs["pixel_path"]>>pixel_path;
    _fs["ROI_HEIGHT"]>>ROI_HEIGHT;
    _fs["ROI_WIDTH"]>>ROI_WIDTH;

    _fs.release();
}

Calibration* Calibration::vet_ptr_calibration = NULL;

Calibration::Calibration()
{
    //初始化参数值
    for(int i=0;i<8;i++)
    {
        comprehensive_parameters[i] = 0;
    }
}

Calibration* Calibration::instance()
{
    //获取单例模式
    if(NULL==vet_ptr_calibration)
    {
        vet_ptr_calibration = new Calibration();
    }
    return vet_ptr_calibration;
}

void Calibration::destoryInstance()
{
    if(NULL!=vet_ptr_calibration)
    {
        delete vet_ptr_calibration;
        vet_ptr_calibration = NULL;
    }
}

bool Calibration::initComprehensiveParameters()
{
//    cout<<"$$$$$$$$$$$$$$$$$: "<<calibrationSettingFile_path<<endl;
    FileStorage _fs(calibrationSettingFile_path, FileStorage::READ);
//    cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%"<<endl;
    if(!_fs.isOpened()){
        cout << "Fail to open calibrationSettingFile.xml" << endl;
        return false;
    }

    FileNode arr_node = _fs["comprehensiveParameters"];
    FileNodeIterator fni = arr_node.begin();
    FileNodeIterator fniEnd = arr_node.end();
    for(int i = 0; fni != fniEnd; fni++){
        comprehensive_parameters[i++] = *fni;
    }

    //show only 2018.4.2
//    for(int i=0;i<8;i++)
//    {
//        cout<<comprehensive_parameters[i]<<endl;
//    }

    _fs.release();

    return true;
}

bool Calibration::createPixelFile()//创建一个xml文件，文件包含6个实际点对应的像素点。
{

    FileStorage _fs(calibrationSettingFile_path, FileStorage::READ);
    if(!_fs.isOpened()){
        cout << "Fail to open calibrationSettingFile.xml" << endl;
        return false;
    }
    float _DistNearCircles;
    float _DistFarCircles;
    float _DistVerticalCircles;
    float _CarFrontDist;
    float _ExtandWidth;
    float _CarFrontWidth;

    //标定时确定的实际坐标参数
    _fs["Distance_of_2_near_circles"] >> _DistNearCircles;
    _fs["Distance_of_2_far_circles"] >> _DistFarCircles;
    _fs["Distance_of_2_vertical_circles"] >> _DistVerticalCircles;
    _fs["Distance_from_car_to_original_coordinate"] >> _CarFrontDist;
    _fs["Extand_width"] >> _ExtandWidth;
    _fs["car_front_width"] >> _CarFrontWidth;

    _fs.release();

    //WRITE XML
    FileStorage fs(pixel_path, FileStorage::WRITE);
    if(!fs.isOpened()){
        cout << "failed to create pixl.xml" << endl;
        return false;
    }

    Point2d _Real_coordinate[6];//实际坐标
    Point2d _pixel_coordinate[6];//像素坐标
    _Real_coordinate[0]=Point2d(-0.5*_DistNearCircles,0);
    _Real_coordinate[1]=Point2d(-0.5*_DistNearCircles,_DistVerticalCircles);
    _Real_coordinate[2]=Point2d(0.5*_DistNearCircles,_DistVerticalCircles);
    _Real_coordinate[3]=Point2d(0.5*_DistNearCircles,0);
    _Real_coordinate[4]=Point2d(-0.5*_CarFrontWidth,-_CarFrontDist);
    _Real_coordinate[5]=Point2d(0.5*_CarFrontWidth,-_CarFrontDist);
    for(int ii=0;ii<6;ii++)
    {
        double x = _Real_coordinate[ii].x;
        double y = _Real_coordinate[ii].y;
//        cout<<x<<"  "<<y<<endl;
        //像素坐标
        if(0!=(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y))
        {
            double ux = (comprehensive_parameters[6]+comprehensive_parameters[0]*x+comprehensive_parameters[1]*y)/(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y);
            double vy = (comprehensive_parameters[7]+comprehensive_parameters[2]*x+comprehensive_parameters[3]*y)/(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y);
            _pixel_coordinate[ii] = Point2d(ux,vy);
        }
        else
        {
            cout<<"The Comprehensive Parameters Error!!!"<<endl;
        }
    }

    fs << "one_x" << _pixel_coordinate[0].x;
    fs << "one_y" << _pixel_coordinate[0].y;
    fs << "two_x" << _pixel_coordinate[1].x;
    fs << "two_y"<< _pixel_coordinate[1].y;
    fs << "three_x" << _pixel_coordinate[2].x;
    fs << "three_y" << _pixel_coordinate[2].y;
    fs << "four_x" << _pixel_coordinate[3].x;
    fs << "four_y" << _pixel_coordinate[3].y;
    fs << "five_x" << _pixel_coordinate[4].x;
    fs << "five_y" << _pixel_coordinate[4].y;
    fs << "six_x" << _pixel_coordinate[5].x;
    fs << "six_y" << _pixel_coordinate[5].y;
    fs.release();
    return true;
}


//将像素坐标转换成实际坐标
void Calibration::pixel2Vehicle(StructPixelAndReal& pixelAndReal)
{
    double ux = pixelAndReal._pixel_coordinate.x;
    double vy = pixelAndReal._pixel_coordinate.y;

    double a1 = comprehensive_parameters[0]-ux*comprehensive_parameters[4];
    double a2 = comprehensive_parameters[1]-ux*comprehensive_parameters[5];
    double b1 = comprehensive_parameters[2]-vy*comprehensive_parameters[4];
    double b2 = comprehensive_parameters[3]-vy*comprehensive_parameters[5];
    double c1 = ux-comprehensive_parameters[6];
    double c2 = vy-comprehensive_parameters[7];

    //世界坐标
    if(0!=(a1*b2-a2*b1))
    {
        double real_x = (b2*c1-a2*c2)/(a1*b2-a2*b1);
        double real_y = (a1*c2-b1*c1)/(a1*b2-a2*b1);
        pixelAndReal._real_coordinate = Point2d(real_x,real_y);
    }
    else
    {
        cout<<"The Comprehensive Parameters Error!"<<endl;
    }
}

//将实际坐标转换为像素坐标
void Calibration::vehicle2Pixel(StructPixelAndReal& pixelAndReal)
{
    double x = pixelAndReal._real_coordinate.x;
    double y = pixelAndReal._real_coordinate.y;

    //像素坐标
    if(0!=(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y))
    {
        double ux = (comprehensive_parameters[6]+comprehensive_parameters[0]*x+comprehensive_parameters[1]*y)/(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y);
        double vy = (comprehensive_parameters[7]+comprehensive_parameters[2]*x+comprehensive_parameters[3]*y)/(1+comprehensive_parameters[4]*x+comprehensive_parameters[5]*y);
        pixelAndReal._pixel_coordinate = Point2d(ux,vy);
    }
    else
    {
        cout<<"The Comprehensive Parameters Error!!!"<<endl;
    }
}

bool Calibration::getAllPixelPoint(vector<Point2d>& allPixelPoint)
{
    //2018.4.11 每次换标定文件时必须重新生成一次，覆盖原来的文件（仅仅生成一次）。
    createPixelFile();
    FileStorage _fs(pixel_path, FileStorage::READ);
    if(!_fs.isOpened()){
        cout << "Fail to1 open pixl.xml" << endl;
        return false;
    }

    double tmp[6][2] = {0}; //读取6行
    //标定时确定的实际坐标参数
    _fs["one_x"]>>tmp[0][0];
    _fs["one_y"]>>tmp[0][1];
    _fs["two_x"]>>tmp[1][0];
    _fs["two_y"]>>tmp[1][1];
    _fs["three_x"]>>tmp[2][0];
    _fs["three_y"]>>tmp[2][1];
    _fs["four_x"]>>tmp[3][0];
    _fs["four_y"]>>tmp[3][1];
    _fs["five_x"]>>tmp[4][0];
    _fs["five_y"]>>tmp[4][1];
    _fs["six_x"]>>tmp[5][0];
    _fs["six_y"]>>tmp[5][1];

    _fs.release();

    //所有像素坐标值
    for(int j=0;j<6;j++)
    {
        allPixelPoint.push_back(Point2d(tmp[j][0],tmp[j][1]));
    }
    return true;

}

bool Calibration::getAllRealPoint(vector<Point2d>& allRealPoint)
{
    FileStorage _fs(calibrationSettingFile_path, FileStorage::READ);
    if(!_fs.isOpened()){
        cout << "Fail to open calibrationSettingFile.xml" << endl;
        return false;
    }

    float _DistNearCircles;
    float _DistFarCircles;
    float _DistVerticalCircles;
    float  _CarFrontDist;
    float _ExtandWidth;
    float _CarFrontWidth;

    //标定时确定的实际坐标参数
    _fs["Distance_of_2_near_circles"] >> _DistNearCircles;
    _fs["Distance_of_2_far_circles"] >> _DistFarCircles;
    _fs["Distance_of_2_vertical_circles"] >> _DistVerticalCircles;
    _fs["Distance_from_car_to_original_coordinate"] >> _CarFrontDist;
    _fs["Extand_width"] >> _ExtandWidth;
    _fs["car_front_width"] >> _CarFrontWidth;

    _fs.release();

    //所有世界坐标值
    allRealPoint.push_back(Point2d(-0.5*_DistNearCircles,0));
    allRealPoint.push_back(Point2d(-0.5*_DistNearCircles,_DistVerticalCircles));
    allRealPoint.push_back(Point2d(0.5*_DistNearCircles,_DistVerticalCircles));
    allRealPoint.push_back(Point2d(0.5*_DistNearCircles,0));
    allRealPoint.push_back(Point2d(-0.5*_CarFrontWidth,-_CarFrontDist));
    allRealPoint.push_back(Point2d(0.5*_CarFrontWidth,-_CarFrontDist));
    return true;
}

