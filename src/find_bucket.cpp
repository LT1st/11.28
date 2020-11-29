// #include "find_bucket.h"

// int FindBucket(
//     Mat input,
//     int weight,
//     int height,
//     int color,
//     int distance)
// {
//     Mat canvas = Mat::zeros(input.size(), CV_8UC3);
//     //center_all获取特性中心
// 	vector<Point2d> center_all;
//     Mat gray = input.clone();
//     // 转化为灰度图
// 	cvtColor(input, gray, COLOR_BGR2GRAY);
//     // 3X3模糊
// 	//blur(srcGray, srcGray, Size(3, 3));
//     // 计算直方图
// 	//equalizeHist(srcGray, srcGray);
//     // 设置阈值根据实际情况 如视图中已找不到特征 可适量调整
// 	// threshold(srcGray, srcGray, 70, 255, THRESH_BINARY | THRESH_OTSU);
//     threshold(gray, gray, 70, 255, THRESH_BINARY | THRESH_OTSU);
//     #ifdef DEBUG
//     namedWindow("阈值图");
// 	imshow("阈值图", threshold_output);
//     #endif
    

// }