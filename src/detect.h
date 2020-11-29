#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define NONE  0
#define PINK  1
#define WHITE 2
#define BLACK 3
#define BLUE  4
#define GREEN 5

#define DEBUG

extern int ballOrder[5];

int roi_region_row2 = 0,roi_region_row22 = 0,roi_region_col2 = 0,roi_region_col22;

/*
功能：检测多个球，f返回位置信息
利用全局变量：g_ball_color
!！加入：可信度（在检测圆时从不敏感参数开始，返回最终检测到球时候的参数可信程度）
*/
int CheckBall(int color);

/* 
功能：在启动时候检测球的排序
注意：1。每个车应该限制的ROI应当不同，应该使用Car1等限定具体是哪个车
     2。！！图像颜色要写一个全局变量并且确保用到它的全部地方都是这个，
        防止场地上的环境光变化，临场调整太慢
实现方式：在4m处，球的间隔距离是固定的：（待测），ROI也是固定的，
        利用启动时候空闲时间较长，在ROI中通过遍历图像颜色，找到五个球的顺序
*/

bool CheckBallOrder(Mat roi_init);