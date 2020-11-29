#include "getdepth_green.h"

//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    #ifdef DEBUG_DEEP
        cout <<"开始检查 深度图对齐彩色图" << endl;
    #endif
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
 
    //获取内参
    const auto intrinDepth = depth_stream.get_intrinsics();
    const auto intrinColor = color_stream.get_intrinsics();

    //g_dpth_ppx = intrinDepth.ppx;//326.615
    //g_dpth_fx  = intrinDepth.fx; //230.942
    //g_dpth_ppy = intrinDepth.ppy;//390.504 
    //g_dpth_fy  = intrinDepth.fy; //390.504 

    //g_color_ppx = intrinColor.ppx;//323.233
    //g_color_ppy = intrinColor.ppy;//241.493
    //g_color_fy  = intrinDepth.fy; //390.504
    //g_color_fx  = intrinDepth.fx; //390.504 
    
    #ifdef DEBUG
    cout << "\t深度相机内参：" << g_dpth_ppx << "\t" << g_dpth_ppy << "\t" << g_dpth_fx
         << "\t" << g_dpth_fy  << "\t" << intrinDepth.width << "\t" << intrinDepth.height 
         << "\t" << intrinDepth.model << "\t" << intrinDepth.coeffs << "\n"
         << "\t彩色相机参数：" << g_color_ppx << "\t" << g_color_ppy << "\t" << g_color_fy << "\t" << g_color_fx << endl;
    #endif
    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);
 
    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];
 
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);
 
            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;
 
            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    #ifdef DEBUG_DEEP
        cout <<"完成检查 深度图对齐彩色图" << endl;
    #endif
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}
 
double measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile ,int x, int y)
{
    #ifdef DEBUG_DEEP
        cout <<"开始检查 测量距离" << endl;
    #endif
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    //定义图像中心点

    /* cv::Point center(color.cols/2,color.rows/2); */
    cv::Point center(x, y);
    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    
    float max1=0,max2=0,max3=0,max4=0;
    //遍历该范围
    float distance_sum = 0,distanceNow = 0;
    int effective_pixel = 0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息

            if(depth.at<uint16_t>(y,x)){
                distanceNow = depth_scale*depth.at<uint16_t>(y,x);
                if(distanceNow >= max1)
                {
                    //頂朱子模型
                    max4 = max3;
                    max3 = max2;
                    max2 = max1;
                    max1 = distanceNow;
                }
                else if(distanceNow >= max2)
                {
                    //頂朱子模型
                    max4 = max3;
                    max3 = max2;
                    max2 = distanceNow;
                }
                else if(distanceNow >= max3)
                {
                    //頂朱子模型
                    max4 = max3;
                    max3 = distanceNow;
                }
                else if(distanceNow >= max4)
                {
                    max4 = distanceNow;
                }


                //max4 = max4 >= distanceNow ? max4 : distanceNow;

                distance_sum += distanceNow;
                effective_pixel ++;
            }
        }
    }
    //cout << "四個排序" << max1 << "\t" << max2 << "\t" <<  max3 << "\t" 
      //   << max4  << endl;
    if(max4!=0)
    {
        //cout <<  "四個\t" << 0.25*(max1 + max2 + max3 + max4) << endl;
    }
    else if(max3!=0)
    {   
        //cout << "三個\t" << 0.33*(max1 + max2 + max3) << endl;
    }
    else if(0 != max2)
    {
        //cout << "兩個\t" << 0.5*(max1 + max2 ) << endl;
    }
    
    //cout<<"遍历完成，有效像素点:"<<effective_pixel<<endl;
    float effective_distance=distance_sum/effective_pixel;

    //cout<<x<<"---"<<y<<"---"<<"目标距离："<<effective_distance<<" m"<<endl;
    char distance_str[30]; 
    sprintf(distance_str,"Distance:%f m",effective_distance);
    cv::rectangle(color,RectRange,Scalar(0,0,255),2,8);
    cv::putText(color,(string)distance_str,cv::Point(color.cols*0.02,color.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),2,8);
    #ifdef DEBUG_DEEP
        cout <<"完成检查 测量距离" << endl;
    #endif
    return effective_distance;
}
 