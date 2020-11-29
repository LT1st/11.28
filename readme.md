# 

## Overview
This example 
```cpp
// Query frame size (width and height)
resault from : COLOR_BGR2RGB
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.2.0) /home/steven/Library/opencv-4.2.0/modules/imgproc/src/bilateral_filter.dispatch.cpp:166: error: (-215:Assertion failed) (src.type() == CV_8UC1 || src.type() == CV_8UC3) && src.data != dst.data in function 'bilateralFilter_8u'
[1]    23240 abort (core dumped)  ../bin/Test
``` 
调整好了参数   

程序修改在这儿

# 现有问题：  
11-20  
1. 角度参数传递可能错误（检查参数的顺序，做好注释）
2. 深度检测点需要确认是否标准
3. 检测深度的像素位置 fx 
## 这个intrinstics参数在这里
```cpp
/** \brief Video stream intrinsics. */
typedef struct rs2_intrinsics
{
    int           width;     /**< Width of the image in pixels */
    int           height;    /**< Height of the image in pixels */
    float         ppx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float         ppy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
    float         fy;        /**< Focal length of the image plane, as a multiple of pixel height */
    rs2_distortion model;    /**< Distortion model of the image */
    float         coeffs[5]; /**< Distortion coefficients */
} rs2_intrinsics;
```

```cpp
angle X = -18.6116  
angle Y = -28.2434  
centerX = 533  
```
# 错误原因
## grb图像fx对齐后变为 深度图的fx 但ppx没变


```cpp
➜  build make       
make: getcwd: Input/output error
make: *** INTERNAL: readdir: Input/output error.  Stop.
```

## 和超睿的程序和并后，发现了6804 segmentation fault
11-24
```cpp
[1]    6804 segmentation fault (core dumped)  ../bin/Test
```