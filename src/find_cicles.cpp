#include "find_cicles.h"

/*其他找圆方法*/
float GetDistance(Point2f p1, Point2f p2)
{
    float dis = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return dis;
}


float ComputeVariance(std::vector<cv::Point> theContour, Point2f theCenter)
{
    int n = theContour.size();
    vector<int> a(n);
 
    float aver,s;
    float sum = 0, e = 0;
 
    for(int i = 0; i < n; i++)
    {
        a[i] = GetDistance(theContour[i],theCenter);
        sum += a[i];
    }
 
    aver = sum / n;
    for(int i = 0; i < n; i++)
        e += (a[i] - aver) * (a[i] - aver);
    e /= n - 1;
    s = sqrt(e);
    return e;
}

vector<Point3f> FindCircles(Mat img, int minPoints, int maxOffset, int minr, int maxr)
{
    vector<Point3f> result;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
 
    findContours( img.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
 
    for (int i = 0; i < contours.size(); i++)
    {
        Point2f center; float radius;  
        if (contours[i].size() > minPoints)
        {
            minEnclosingCircle(contours[i], center, radius);//得到最小外接圆圆心和半径  
            int offset = ComputeVariance(contours[i], center);
            if (offset < maxOffset && radius > minr && radius <= maxr)
            {
                Point3f p(center.x, center.y, radius);
                result.push_back(p);
            }
        }
    }
 
    return result;
}

