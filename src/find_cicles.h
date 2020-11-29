#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


vector<Point3f> FindCircles(Mat img, int minPoints,     
                int maxOffset, int minr, int maxr);
