#include <iostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
 
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

//#include "ConstVariate.h"

using namespace cv;
using namespace std;

//深度相机参数
extern float g_dpth_ppx,
             g_dpth_fx,
             g_dpth_ppy,
             g_dpth_fy;
//彩色相機參數
extern float g_color_ppx , 
             g_color_ppy , 
             g_color_fx , 
             g_color_fy;
            
//球现在的颜色
//可信度1，2
extern int g_ball_color_now_1st,
            g_ball_color_now_2nd;

//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev);

//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile);

//测量深度
double measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile ,int x, int y);