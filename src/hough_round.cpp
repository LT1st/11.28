#include "hough_round.h"

/*
补充功能：根据距离调整minDist min max
*/

int lostCnt;
extern int g_gs_size;

vector<Vec3f> houghRound_circles(
            Mat dst_s,
            int hough_minDist ,          //圆心之间的最小距离= 70
            int hough_canny ,            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
            int hough_addthersold ,      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
            int hough_minRadius ,        //有默认值0，表示圆半径的最小值= 0
            int hough_maxRadius          //有默认值0，表示圆半径的最大值= 0
)
{

    #ifdef DEBUG_DEEP
        cout <<"开始检查 hough 园" << endl;
    #endif
    vector<Vec3f> circles;
    //图像预处理
    //高斯双边模糊，不太好调节

    // ？有报错  // 11.10：COLOR_BGR2RGB,因为RGB就是CV_8UC3
   /*  cvtColor(dst_s,dst_s,COLOR_BGR2RGB);
    Mat dst_tmp = dst_s.clone();
    bilateralFilter(dst_tmp,dst_s, 
                    -1,//对算法效率影响较大，-1为自动选择，视频不超过5
                    50.0,//150明显卡通
                    5.0//大了就不连续
                    );
    cvtColor(dst_s,dst_s,COLOR_RGB2BGR); */
    cout << "高斯参数size:" << g_gs_size << endl;
    GaussianBlur(dst_s,dst_s,Size(g_gs_size,g_gs_size),1,1);
    //namedWindow("hough_ground_高斯双边模糊");
    namedWindow("hough_ground_结果");
    //imshow("hough_ground_高斯双边模糊", dst_s);

    //均值迁移，EPT边缘保留滤波
    //pyrMeanShiftFiltering(dst_s, dst_s, 5, 100);

    //使用高斯模糊，修改卷积核ksize也可以检测出来
    //GaussianBlur(dst_s, dst_s, );

    // cvtColor(dst_s,dst_s,COLOR_BGR2HSV);
    // vector<Mat> hsvSplit;
    // split(dst_s,hsvSplit);
    // equalizeHist(hsvSplit[2], hsvSplit[2]);
    // merge(hsvSplit,dst_s);

    //medianBlur(dst_s, dst_s, 5);

    //Canny(dst_s,dst_s,10,250,5);
    
    cvtColor(dst_s , dst_s , COLOR_BGR2GRAY);
    normalize(dst_s , dst_s , 1.0 , 0.0 , NORM_MINMAX);//归一到0~1之间 
    dst_s.convertTo(dst_s, CV_8UC1, 255, 0); //转换为0~255之间的整数 
    /*
    第一个参数，InputArray类型的image，输入图像，即源图像，需为8位的灰度单通道图像。
    第二个参数，InputArray类型的circles，经过调用HoughCircles函数后此参数存储了检测到的圆的输出矢量，
        每个矢量由包含了3个元素的浮点（vec3d）矢量(x, y, radius)表示。
    第三个参数，int类型的method，即使用的检测方法，目前OpenCV中就霍夫梯度法一种可以使用，它的标识符为CV_HOUGH_GRADIENT，在此参数处填这个标识符即可。
    第四个参数，double类型的dp，用来检测圆心的累加器图像的分辨率于输入图像之比的倒数，且此参数允许创建一个比输入图像分辨率低的累加器。
        上述文字不好理解的话，来看例子吧。例如，如果dp= 1时，累加器和输入图像具有相同的分辨率。如果dp=2，累加器便有输入图像一半那么大的宽度和高度。
    第五个参数，double类型的minDist，为霍夫变换检测到的圆的圆心之间的最小dst_s距离，即让我们的算法能明显区分的两个不同圆之间的最小距离。
        这个参数如果太小的话，多个相邻的圆可能被错误地检测成了一个重合的圆。反之，这个参数设置太大的话，某些圆就不能被检测出来了。
    第六个参数，double类型的param1，有默认值100。它是第三个参数method设置的检测方法的对应的参数。
        对当前唯一的方法霍夫梯度法CV_HOUGH_GRADIENT，它表示传递给canny边缘检测算子的高阈值，而低阈值为高阈值的一半。
    第七个参数，double类型的param2，也有默认值100。它是第三个参数method设置的检测方法的对应的参数。
        对当前唯一的方法霍夫梯度法CV_HOUGH_GRADIENT，它表示在检测阶段圆心的累加器阈值。
        它越小的话，就可以检测到更多根本不存在的圆，而它越大的话，能通过检测的圆就更加接近完美的圆形了。
    第八个参数，int类型的minRadius,有默认值0，表示圆半径的最小值。
    第九个参数，int类型的maxRadius,也有默认值0，表示圆半径的最大值。
    */
    
    HoughCircles(dst_s, circles, HOUGH_GRADIENT, 2, hough_minDist, hough_canny, 
                hough_addthersold, hough_minRadius, hough_maxRadius);
    
    if(circles.size() == 0 )   
    {
        
        lostCnt++;
        //cout << "lostCnt_inHoughCircles = " << lostCnt << endl;
    }
    else
    {
        //cout << " 检测到的圆数量： " << circles.size() ;
    }
    
    //颜色计数器 顺序为 p wt bk b g
    int h=0,s=0,v=0,colorCnt[5];

    //在图中绘制出圆
    for (size_t i = 0; i < circles.size(); i++)
    {
        //参数定义
        int x = cvRound(circles[i][0]), y =cvRound(circles[i][1]),radius = cvRound(circles[i][2]);
        Point center(x , y);
        #ifdef NEW
        
        
        //记得便利图像读取颜色数值时候需要读取彩色图，而不是虑色后的图
        for(int j = x - radius; i <= x + radius; i++)
        {
            for(int k = y- radius; k*k + j*j < radius * radius;k++)
            {
                //h=;s=;v=;
                if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[0] ++;
                if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[1] ++;
                if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[2] ++;
                if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[3] ++;
                if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[4] ++;
            }
        }
        int cntmax=0;
        for(int i = 0; i < 4;i++)
        {
            if(colorCnt[i] < colorCnt[i+1]) 
                cntmax = colorCnt[i+1];
        }
        for(int i = 0; i < 5;i++)
        {
            if( cntmax ==  colorCnt[i])
                g_call_color_now_1st = i;
        }
        #endif
        //绘制圆心
        //circle(RGBImg, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //绘制圆轮廓

        #ifdef DEBUG
            cout << "ROI中的圆心" << center.x << "\t" << center.y <<endl;
        #endif
        circle(dst_s, center, radius, Scalar(155, 50, 255), 3, 8, 0);
        //绘制圆心
        circle(dst_s, center, 3, Scalar(0, 255, 0), -1, 8, 0);

    }
     
/*     for (size_t i = 0; i < circles.size(); i++)
    {
        
    } */
    //输出图像
    #ifdef DEBUG_DEEP
        cout <<"完成检查 hough 圆" << endl;
    #endif
    imshow("hough_ground_结果", dst_s);
    return circles;
}