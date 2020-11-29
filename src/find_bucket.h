#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#define DEUBG

using namespace std;
using namespace cv;

//
int FindBucket(
    Mat input,
    int weight,
    int height,
    int color,
    int distance
);
