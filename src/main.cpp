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

#include "segcolors.h"
#include "getdepth_green.h"
#include "hough_round.h"
#include "deviation_angle.h"
#include "ConstVariate.h"
#include "own_serial.h"               
#include "detect.h"

/* using namespace cv;
using namespace std; */

//數劇說明
//搜寻目标颜色数值设定
#define NONE 0
#define PINK  1
#define WHITE 2
#define BLACK 3
#define BLUE  4
#define GREEN 5

#define DEBUG
#define DEBUG_DEEP
#define DEBUG_MANY
#define IFSERIAL
#define No1Car
//#define NEW
int findBallOrder = 1;

//全局变量，应该加g_ 

//HSV的参数设定
int hmin_Max = 261;int hmax_Max = 360;
int smin_Max = 47 ;int smax_Max = 255;
int vmin_Max = 0  ;int vmax_Max = 255;

//霍夫参数设定
int hough_minDist = 75;          //圆心之间的最小距离= 70
int hough_canny = 20;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
int hough_addthersold = 40;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
int hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
int hough_maxRadius = 30;        //有默认值0，表示圆半径的最大值= 0

//分割时背景的颜色设定
int seg_bkg = BLACK;

//球的颜色
int g_ball_color = GREEN; //注意他的位置
int g_call_color_now_1st = GREEN;
int g_call_color_now_2nd = GREEN;
int ballOrder[5] = {BLUE, PINK, WHITE, GREEN, BLACK};


//球的距离
float g_distance_1 = 0.0;

//球的角度
float g_angle_x = 0.0;

//切割图像参数
int roi_region_row1 = 320;
int roi_region_row  = 320;
int roi_region_col1 = 310;
int roi_region_col  = 310;

//长度参数
float imgRows = 640, imgCols = 480;

//时间记录
double time_main =0;

//深度相机参数
float g_dpth_ppx = 326.615;
float g_dpth_ppy = 241.493;
float g_dpth_fx  = 390.504;
float g_dpth_fy  = 390.504;

//彩色相機參數
float g_color_ppx = 323.233;
float g_color_ppy = 241.493;
float g_color_fx  = 390.504;
float g_color_fy  = 390.504;

Mat g_color_image;

int g_flg = 0,g_wait = 0,g_wait1 = 0,g_wait2 = 0,g_readColor = 0,g_readColorRecord = 0,g_Cnt = 0,g_gs_size = 11;
float g_center2center = 0,g_deltaR = 0;
vector<Vec3f> g_circles;

int main()
{
    #ifdef IFSERIAL
    int fd2 = 0;

    #ifdef No1Car
    own_serial ownSerial("/dev/ttyUSB0",B115200);//tx2 /dev/ttyTHS2 
                                            //up squared /dev/ttyS5 
                                            //nuc /dev/ttyUSB0
    #endif

    #ifdef No2Car
    own_serial ownSerial("/dev/ttyTHS2",B115200);//tx2 /dev/ttyTHS2 
                                            //up squared /dev/ttyS5 
                                            //nuc /dev/ttyUSB0
    #endif

    #ifdef No3Car
    own_serial ownSerial("/dev/ttyUSB0",B115200);//tx2 /dev/ttyTHS2 
                                            //up squared /dev/ttyS5 
                                            //nuc /dev/ttyUSB0
    #endif

    ownSerial.init(fd2);
    //fd = ownSerial.isopen();
    if(fd2 ==-1)
    {
        cout<<"can't open serial\n";
        exit(0);
    }
    
    #endif

    //初始化启动时间
    time_main = static_cast<double>(getTickCount());

    //初始化窗口
    const char* depth_win="depth_Image1nit";
    namedWindow(depth_win,WINDOW_AUTOSIZE);
    const char* color_win="color_Image1nit";
    namedWindow(color_win,WINDOW_AUTOSIZE);
 
    //深度图像颜色map
    rs2::colorizer c;
    // Helper to colorize depth images
 
    //想要获取的位置
    int centerLocation_x = 320;
    int centerLocation_y = 240;

    //循环中的变量
    Mat segcolor_pic;
    Mat result;
    
    //彩色圖像的中間變量
    Mat mid_color_image;

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16 ,60);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,60);
 
    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);
 
    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;
 
    //声明数据流
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
 
    //获取内参
    auto intrinDepth = depth_stream.get_intrinsics();
    auto intrinColor = color_stream.get_intrinsics();

    g_dpth_ppx = intrinDepth.ppx;
    g_dpth_ppy = intrinDepth.ppy;
    g_dpth_fx  = intrinDepth.fx;
    g_dpth_fy  = intrinDepth.fy;

    g_color_fx = intrinColor.fx;
    g_color_fy = intrinColor.fy;
    g_color_ppx = intrinColor.ppx;
    g_color_ppy = intrinColor.ppy;

    cout << "intrinColor.cx = " << intrinColor.ppx ;
    cout << "intrinColor.cy = " << intrinColor.ppy ;
    cout << "intrinColor.fx = " << intrinColor.fx  ;
    cout << "intrinColor.fy = " << intrinColor.fy  ;

    cout << "intrinDepth.cx = " << intrinDepth.ppx ;
    cout << "intrinDepth.cy = " << intrinDepth.ppy ;
    cout << "intrinDepth.fx = " << intrinDepth.fx  ;
    cout << "intrinDepth.fy = " << intrinDepth.fy  ;
    
    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    
    while (cvGetWindowHandle(depth_win)&&cvGetWindowHandle(color_win)) // Application still alive?
    {
        time_main = (double)getTickCount()/getTickFrequency();
        //确定兴趣区域ROI
        roi_region_row =  imgRows*3/4-50;
        roi_region_row1 = imgRows;
        roi_region_col =  imgCols/4;
        roi_region_col1 = imgCols*3/4;

        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        //创建OPENCV类型 并传入数据
        Mat depth_image(Size(depth_w,depth_h),
                                CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Mat depth_image_4_show(Size(depth_w,depth_h),
                                CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color_image(Size(color_w,color_h),
                                CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);


        g_color_image = color_image.clone(); 
        //多个颜色切换检测

        //g_ball_color = GREEN;
        cout << "switch:" << g_ball_color << endl;
        int g_ball_color_Record = g_ball_color;
        switch (g_ball_color){
            case PINK :
                //HSV滤色的参数设定
                hmin_Max = 300;  
                hmax_Max = 325;
                smin_Max = 100 ;  
                smax_Max = 255;
                vmin_Max = 80  ;  
                vmax_Max = 235;
                //霍夫参数设定
                hough_minDist = 500;         //圆心之间的最小距离= 70
                hough_canny = 50;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 20;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 20;        //有默认值0，表示圆半径的最大值= 0
                seg_bkg = WHITE;             //使用背景色
                g_gs_size = 15;              //高斯滤波size
                break;

            case WHITE :
                //HSV滤色的参数设定
                // hmin_Max = 185;  
                // hmax_Max = 319;
                // smin_Max = 32;                 
                // smax_Max = 140;
                // vmin_Max = 100;  
                // vmax_Max = 255;
                hmin_Max = 190;  
                hmax_Max = 300;
                smin_Max = 0;                 
                smax_Max = 115;
                vmin_Max = 180;  
                vmax_Max = 255;
                //霍夫参数设定
                hough_minDist = 500;          //圆心之间的最小距离= 70
                hough_canny = 50;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 10;       //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;          //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 30;         //有默认值0，表示圆半径的最大值= 0
                seg_bkg = BLACK;
                g_gs_size = 11;
                break;

            case BLUE :
                //HSV滤色的参数设定
                hmin_Max = 210;  
                hmax_Max = 310;
                smin_Max = 40;
                smax_Max = 255;
                vmin_Max = 0;  
                vmax_Max = 255;
                //霍夫参数设定
                hough_minDist = 500;          //圆心之间的最小距离= 70
                hough_canny = 50;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 30;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 50;        //有默认值0，表示圆半径的最大值= 0
                seg_bkg = WHITE;
                g_gs_size = 11;
                break;

            case GREEN :
                //HSV滤色的参数设定
                // hmin_Max = 60 ;  //左边
                // hmax_Max = 150;
                // smin_Max = 70;
                // smax_Max = 220;
                // vmin_Max = 50;  
                // vmax_Max = 190;
                
                hmin_Max = 60 ; //右边  
                hmax_Max = 130;
                smin_Max = 70;
                smax_Max = 255;
                vmin_Max = 50;  
                vmax_Max = 190;
                
                //霍夫参数设定
                hough_minDist = 500;          //圆心之间的最小距离= 70
                hough_canny = 50;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 30;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 50;        //有默认值0，表示圆半径的最大值= 0
                seg_bkg = WHITE;
                g_gs_size = 11;
                break;

            case BLACK :
                //HSV滤色的参数设定
                hmin_Max =  0 ;
                hmax_Max = 360;
                smin_Max = 0  ;
                smax_Max = 255;
                vmin_Max = 0  ;
                vmax_Max = 35;
                //霍夫参数设定
                hough_minDist = 500;          //圆心之间的最小距离= 70
                hough_canny = 55;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 30;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 30;        //有默认值0，表示圆半径的最大值= 0
                //修改bkg
                seg_bkg = WHITE;
                g_gs_size = 15;
                break;

            case 6 :
                //HSV滤色的参数设定
                hmin_Max =  0 ;
                hmax_Max = 360;
                smin_Max = 0  ;
                smax_Max = 30;
                vmin_Max = 150  ;
                vmax_Max = 255;
                //霍夫参数设定
                hough_minDist = 500;          //圆心之间的最小距离= 70
                hough_canny = 55;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
                hough_addthersold = 30;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
                hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
                hough_maxRadius = 30;        //有默认值0，表示圆半径的最大值= 0
                //修改bkg
                seg_bkg = BLACK;
                g_gs_size = 15;
                break;    

            default:

                break;
        }
        mid_color_image = color_image.clone();
        //mid_color_image = mid_color_image(Rect(color_image.cols/2,color_image.rows*3/4,color_image.rows/4,color_image.cols/2)).clone(); 
          
        roi_region_row1 = color_image.rows;
        roi_region_row  = color_image.rows*3/4-50;
        roi_region_col1 = color_image.cols*3/4;
        roi_region_col  = color_image.cols/4;
        //CvPoint 
        //cvCircle(color_image, (roi_region_x1,roi_region_y1), 2, BLUE);
        //这里要改roi，变成全局统一的位置
        //mid_color_image = mid_color_image(Range(color_image.rows*3/4-50,color_image.rows),Range(color_image.cols/2,color_image.cols)).clone(); 
        //输出画面参数
        //cout << "color_image.rows=" << color_image.rows << "\t" << color_image.cols << endl;
        mid_color_image = mid_color_image(Range(roi_region_row,roi_region_row1),Range(roi_region_col,roi_region_col1)).clone(); 
        
        //初始化时候找球
        if(findBallOrder)
        {
            Mat findBallOrder = color_image.clone();
            // CheckBallOrder(findBallOrder);
        }
        //namedWindow("tst");
        //imshow("tst",mid_color_image);

        //用测好的参数来HSV分割， trickbar参数测量在 hsv分割地面（保留）
        segcolor_pic = segmentation_HSV(hmin_Max , hmax_Max, 
            smin_Max, smax_Max, vmin_Max, vmax_Max, mid_color_image, seg_bkg);
        namedWindow("segcolor");imshow("segcolor", segcolor_pic);
        

        //找到圆  color_image
        vector<Vec3f> circles_record = g_circles;
        g_circles = houghRound_circles(segcolor_pic, 
            hough_minDist, hough_canny, hough_addthersold, hough_minRadius, hough_maxRadius);
        /*************************************/
        mid_color_image.convertTo(mid_color_image, CV_32FC3, 1.0 / 255, 0);
        //circle(mid_color_image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //imshow("1",mid_color_image);
        cvtColor(mid_color_image, mid_color_image, COLOR_BGR2HSV);

        
        //imshow("2",mid_color_image); 
        if(circles_record.size() != 0)
        {
            cout << "1圆.x:" << circles_record[0][0] << " 1圆.y:" << circles_record[0][1] << " 1圆.r:" << circles_record[0][2] << endl;
        }
        

        //namedWindow("color_image");imshow("color_image", color_image);

        //实现深度图对齐到彩色图
        result = align_Depth2Color(depth_image,color_image,profile);

        //找到的圆的数量：
        cout << "找到的圆的数量:" << g_circles.size() << endl;

        // if(g_circles.size() == 0)
        // {
        //     g_wait1++;
        //     if(g_wait1 > 5)
        //     {
        //         g_wait1 = 0;
        //         g_ball_color++;
        //     }
        // }
        if(g_ball_color > 5)
        {
            g_ball_color = 1;
        }
        if(g_circles.size() != 0)
        {
            //cout << "1圆.x:" << circles_record[0][0] << " 1圆.y" << circles_record[0][1] << " 1圆.r" << circles_record[0][2] << endl;
            cout << "圆.x:" << g_circles[0][0] << " 圆.y:" << g_circles[0][1] << " 圆.r:" << g_circles[0][2] << endl;
            if(circles_record.size() != 0)
            {
                g_center2center = sqrt(pow(g_circles[0][0] - circles_record[0][0],2) + pow(g_circles[0][1] - circles_record[0][1],2));
                g_deltaR = fabs(g_circles[0][2] - circles_record[0][2]);
                cout << "圆心变化距离:" << g_center2center << " 半径变化:" << g_deltaR << endl;          
            }
            
            if(g_ball_color > 5)
            {
                g_ball_color = 1;
            }
            /*******************************判断颜色*******************************/
            Size range_ifcolor = Size(20,20);
            //cout<<"\n\n\n错误："<<g_circles[0][0]<< "\t"<<range_ifcolor.width/2 
            //<< "\t"<< g_circles[0][1]<<endl;
            Rect RectRange_ifcolor(cvRound(g_circles[0][0])-range_ifcolor.width/2,cvRound(g_circles[0][1])-range_ifcolor.height/2,range_ifcolor.width,range_ifcolor.height);
            //遍历该范围
            float midH = 0,midS = 0,midV = 0;
            int effective_pixel_ifcolor = 0;
            for(int y=RectRange_ifcolor.y;y<RectRange_ifcolor.y+RectRange_ifcolor.height;y++)
            {
                for(int x=RectRange_ifcolor.x;x<RectRange_ifcolor.x+RectRange_ifcolor.width;x++)
                {
                    //如果深度图下该点像素不为0，表示有距离信息
                    if(mid_color_image.at<Vec3f>(y,x)[0])
                    {
                        midH += mid_color_image.at<Vec3f>(y,x)[0];
                        midS += mid_color_image.at<Vec3f>(y,x)[1];
                        midV += mid_color_image.at<Vec3f>(y,x)[2]; 
                        effective_pixel_ifcolor++;
                    }
                }
            }
        /*  Vec3f effective_distance=midH/effective_pixel;
            cout << "HSV:" << mid_color_image.at<Vec3f>(center.x,center.y) << endl; */
            cout << "HSV:" << mid_color_image.at<Vec3f>(cvRound(g_circles[0][0]),cvRound(g_circles[0][1])) << endl;
            cout << "H:" <<midH/effective_pixel_ifcolor/360 << " " << midH/effective_pixel_ifcolor << endl;
            cout << "S:" <<midS/effective_pixel_ifcolor << " " << midS/effective_pixel_ifcolor*255 << endl;
            cout << "V:" <<midV/effective_pixel_ifcolor << " " << midV/effective_pixel_ifcolor*255 << endl; 
            
            int readH = midH/effective_pixel_ifcolor,readS = midS/effective_pixel_ifcolor*255,
                readV = midV/effective_pixel_ifcolor*255;
            // PINK  1: 330 160 70
            // WHITE 2: 220 60 220
            // BLACK 3: 190 100 25
            // BLUE  4: 220 210 100
            // GREEN 5: 100 210 120
            if(readH > 280 && readH < 340)
            {
                g_readColor = PINK;
            }
            // if(readH > 190 && readH < 360 && readS < 100 && readS > 40 && readV > 180 && readV < 255)
            if(readS < 100 && readS > 40 && readV > 180 && readV < 255)
            {
                g_readColor = WHITE;
            }        
            // if(readH > 150 && readH < 220 && readS < 180 && readS > 80 && readV < 40 && readV > 0)
            if(readV < 50 && readV > 0)
            {
                g_readColor = BLACK;
            }
            if(readH > 180 && readH < 250 && readS > 180 && readS < 255 && readV < 130 && readV > 60)
            {
                g_readColor = BLUE;
            }
            if(readH > 80 && readH < 150 && readS > 150 && readS < 250 && readV < 150 && readV > 80)
            {
                g_readColor = GREEN;
            }            
            // cout << "读到的颜色:" << g_readColor << endl;
            // cout << "分割出来的是:" << g_ball_color <<endl;
            // if(g_readColor == g_ball_color)
            // {
            //     cout << "匹配到:" << g_readColor << endl; 
            //     g_flg = 1;
            // }
            // if(g_flg == 1)
            // {
            //     g_wait++;
            // }
            // if(g_wait > 10)
            // {
            //     g_flg = 0;
            //     g_wait = 0;
            // }
            // cout << "g_flg:" << g_flg << endl;
            // if(g_flg == 0)
            // {
            //     if(g_readColor != g_ball_color)
            //     {
            //         cout << "切换" << endl;
            //         g_ball_color++;//切换
            //     }
            //     if(g_circles.size() == 0)
            //     {
            //         g_ball_color++;
            //     }
            // }
            
            /**********************************************************/
 

            centerLocation_x = cvRound(g_circles[0][0]) + roi_region_col;
            centerLocation_y = cvRound(g_circles[0][1]) + roi_region_row;
            
            //输出当前roi
            //cout << roi_region_row << "\troi_region_   " << roi_region_col << "\t+  cvRound   " <<  cvRound(g_circles[0][0]) << "\t"
            //<< cvRound(g_circles[0][1]) << "\t=   centerLocation_    " << centerLocation_x << "\t" << centerLocation_y << endl;
            //测量距离
            g_distance_1 = measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //Point startPoint
            line(color_image, Point(centerLocation_x,centerLocation_y),Point(centerLocation_x,g_color_ppx), Scalar(0,0,255),1);
            //测量角度
            g_angle_x = deviation_angle_x( centerLocation_x , centerLocation_y );
            //cout << "\nangle X = " << g_angle_x << "\tangle Y = " << 
            //deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;
            
            //cout << "centerLocation：  " << centerLocation_x << "\t" << centerLocation_y << endl;
            /* centerLocation_x = 0;
            centerLocation_y = 0;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\tangle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 0;
            centerLocation_y = 240;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 0 240 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 0;
            centerLocation_y = 477;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 0 477angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;
            
            centerLocation_x = 320;
            centerLocation_y = 0;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 320 0 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 320;
            centerLocation_y = 240;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 320 240 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
 
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 320;
            centerLocation_y = 477;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 320 477 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 640;
            centerLocation_y = 0;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 640 0 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 640;
            centerLocation_y = 240;
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 640 240 angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;

            centerLocation_x = 640;
            centerLocation_y = 480;
 
 
            //测量距离
            measure_distance(color_image , result , cv::Size(20,20) , profile , centerLocation_x , centerLocation_y);
            //测量角度
            cout << "\t 640 477angle X = " << deviation_angle_x( centerLocation_x , centerLocation_y ) << "\tangle Y = " <<
            deviation_angle_y( centerLocation_x , centerLocation_y ) << endl;  */
            //cout << endl;
            if(g_ball_color > 5)
            {
                g_ball_color = 1;
            }
        }
        else{}
        cout << "读到的颜色:" << g_readColor << endl;
        cout << "分割出来的是:" << g_ball_color <<endl; 
        if(g_readColor == g_ball_color)
        {
            cout << "匹配到:" << g_readColor << endl;
            cout << "Record:" << g_readColorRecord << endl;
            if(g_readColor == g_readColor && g_readColor != 0 && g_readColorRecord != 0)
            {
                g_Cnt++;
                cout << "g_Cnt:" << g_Cnt << endl;
            }
            // cout << g_readColor << endl;
            g_readColorRecord = g_readColor; 
            // cout << "Record2:" << g_readColorRecord << endl;
            g_flg = 1;
            g_wait = 0;
        }
 
        if(g_flg == 1)
        {
            g_wait++;
            if(g_ball_color_Record == 1 && (g_center2center > 30 || g_deltaR > 30))
            {
                //cout << "圆心变化距离:" << g_center2center << " 半径变化:" << g_deltaR << endl;
                cout << "1111111111111111111111111111111111111111111111111111111111" << endl;
                // return -1;
                g_flg = 0;
                // g_ball_color++;
            }
        }
        if(g_wait > 20)
        {
            cout << endl;
            g_flg = 0;
            g_wait = 0;
        }
        cout << "g_flg:" << g_flg << endl;
        if(g_flg == 0)
        {
            if(g_readColor != g_ball_color)
            {
                cout << "切换" << endl;
                g_ball_color++;//切换
            }
            else if(g_circles.size() == 0)
            {
                g_ball_color++;
 
            }
        }
        if(g_ball_color > 5)
        {
            g_ball_color = 1;
        }
        if(g_Cnt > 5)
        {
            g_Cnt = 0;
            cout << "可以发送:" << g_readColor << endl; 
            g_call_color_now_1st = g_readColor;

            #ifdef IFSERIAL 
                #ifdef DEBUG   
                    g_distance_1 = 11.11;
                    g_angle_x = 22.22;
                    g_call_color_now_1st = 3;

                #endif
             cout<<"ditance="<<g_distance_1<<"   amgle="<<g_angle_x
                <<"   Angle="<<g_distance_1<<"  status="<<g_call_color_now_1st<<endl;
                ownSerial.writeData(g_distance_1,g_angle_x,
                                    g_distance_1,g_call_color_now_1st);
            #endif

            cout << "**************************************************************************************************************" << endl;
        }

 
        g_readColor = 0;
       // centerLocation_x += roi_region_x;
       // centerLocation_y += roi_region_y;


        #ifdef IFSERIAL 
            #ifdef DEBUG   
                g_distance_1 = 11.11;
                g_angle_x = 22.22;
                g_call_color_now_1st = 3;

            #endif
            //  cout<<"ditance="<<g_distance_1<<"   amgle="<<g_angle_x
            //     <<"   Angle="<<g_distance_1<<"  status="<<g_call_color_now_1st<<endl;
                ownSerial.writeData(g_distance_1,g_angle_x,
                                    g_distance_1,g_call_color_now_1st);
        #endif

        //显示
        imshow(depth_win,depth_image_4_show);
        imshow(color_win,color_image);

        // namedWindow("result");imshow("result",result);
        // cout << endl;
        // cout << "FPS: " <<(((double)getTickCount())/getTickFrequency() - time_main)) <<"\t"<< (double)getTickCount()/getTickFrequency() - time_main << "*******************\n\n" << endl;
        waitKey(1);
    }

    return 0;
}
 
