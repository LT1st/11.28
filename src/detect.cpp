// #include "detect.h"

// bool CheckBallOrder_ifreasonable()
// {
//     bool ifBallOrderRight = 1;
//     for(int i=0; i<5 ;i++)
//     {
//         if(ballOrder[i] == NONE)
//         {
//             cout << "初始化时检测到的球有漏掉的\t" << i << endl;
//             break;
//         }
//     }
    
//     for(int i=0; i<5 ;i++)
//     {
//         for(int j=i+1; j<5; j++)
//         {
//             if(ballOrder[i] == ballOrder[j])
//             {
//                 cout << "初始化时检测到的球顺序异常\t" << i << "\t" << j << endl;
//                 return 0;
//             }
//         }
//     }

//     if((ifBallOrderRight == 1) && (ballOrder[4] != 0))
//     {
//         cout << "球的排序检测到了： " << ballOrder << endl;
//     }

//     return 1;
// }

// bool CheckBallOrder(Mat roi_init)
// {
//     Mat mid_color_image = roi_init(Range(roi_region_row2,roi_region_row22),Range(roi_region_col2,roi_region_col22)).clone(); 
//     int col4Ball_1st = 1, row4Ball_1st = 1, row4Ball = 1, col4Ball = 1;
//     int readH = 1,readS = 1,readV = 1;
//     int jiangeX=50,jiangeY=2;
//     #ifdef No1Car
//         col4Ball_1st = 50 , row4Ball_1st = 50;
//         jiangeX=50,jiangeY=2;
//     #endif
//     Size range_startdetect = Size(4,4);

//     for(int i = 0; i < 5 ; i++)
//     {
//         row4Ball = row4Ball_1st + jiangeX*i;
//         col4Ball = col4Ball_1st + jiangeY*i;
//      /*    Rect RectRange_startDetect(row4Ball - range_startdetect.width/2, 
//                                 col4Ball - range_startdetect.height/2, 
//                                 range_startdetect.width,range_startdetect.height
//                                 ); */
//         Rect RectRange_startDetect(row4Ball, col4Ball, 4, 4 );

//         //遍历该范围
//         float midH = 1,midS = 1,midV = 1;
//         int effective_pixel_startdetect = 1;
//         int initCols = roi_init.cols;
//         int initRows = roi_init.rows;
//         int size =5;
//         for(int i=0; i<initCols-size ; i++ )
//         {
//             for(int j=0 ; j<initRows-size ; j++ )
//             {
//                 for(int col=i; col<(i+4) ;col++)
//                 {
//                     for(int row=j; row<(j+4); row++)
//                     {
//                         midH += roi_init.at<Vec3f>(col,row)[0];
//                         midS += roi_init.at<Vec3f>(col,row)[1];
//                         midV += roi_init.at<Vec3f>(col,row)[2]; 
//                         effective_pixel_startdetect++;
//                     }
//                 }
//                 readH /= effective_pixel_startdetect; 
//                 readS /= effective_pixel_startdetect;
//                 readV /= effective_pixel_startdetect;

//                  if(readH > 280 && readH < 340)
//                 {
//                     ballOrder[i] = PINK;
//                     cout << "\t1 pink\t";
//                 }
//                 if(readS < 100 && readS > 40 && readV > 180 && readV < 255)
//                 {
//                     ballOrder[i] = WHITE;
//                     cout << "2 white\t";
//                 }        
//                 if(readV < 50 && readV > 0)
//                 {
//                     ballOrder[i] = BLACK;
//                     cout << "3 black\t";
//                 }
//                 if(readH > 180 && readH < 250 && readS > 180 && readS < 255 && readV < 130 && readV > 60)
//                 {
//                     ballOrder[i] = BLUE;
//                     cout << "4 blue\t";
//                 }
//                 if(readH > 80 && readH < 150 && readS > 150 && readS < 250 && readV < 150 && readV > 80)
//                 {
//                     ballOrder[i] = GREEN;
//                     cout << "5 green\n";
//                 }
//                 else
//                 {
//                     ballOrder[i] = 0;
//                 }  

//                 if(CheckBallOrder_ifreasonable())
//                 {
//                     cout << "!!!!!!!!!!!!成功检测到球的顺序：  " << i << "\t" << j << endl;
//                 }
//             }
//         }
        
//         for(int col=col4Ball; col<(col4Ball+4) ;col++)
//         {
//             for(int row=row4Ball; row<(row4Ball+4); row++)
//             {
//                 midH += roi_init.at<Vec3f>(col,row)[0];
//                 midS += roi_init.at<Vec3f>(col,row)[1];
//                 midV += roi_init.at<Vec3f>(col,row)[2]; 
//                 effective_pixel_startdetect++;
//             }
//         }
        
//         readH /= effective_pixel_startdetect; 
//         readS /= effective_pixel_startdetect;
//         readV /= effective_pixel_startdetect;

//         #ifdef DEBUG
//         cout << "\n";
//         cout <<"颜色读取： "<< readH << "\t" << readS << "\t" << readV << endl;
//         cout <<"选取的位置：" << row4Ball << "\t" << col4Ball <<endl;
//         #endif 

//         if(readH > 280 && readH < 340)
//         {
//             ballOrder[i] = PINK;
//             cout << "\t1 pink\t";
//         }
//         if(readS < 100 && readS > 40 && readV > 180 && readV < 255)
//         {
//             ballOrder[i] = WHITE;
//             cout << "2 white\t";
//         }        
//         if(readV < 50 && readV > 0)
//         {
//             ballOrder[i] = BLACK;
//             cout << "3 black\t";
//         }
//         if(readH > 180 && readH < 250 && readS > 180 && readS < 255 && readV < 130 && readV > 60)
//         {
//             ballOrder[i] = BLUE;
//             cout << "4 blue\t";
//         }
//         if(readH > 80 && readH < 150 && readS > 150 && readS < 250 && readV < 150 && readV > 80)
//         {
//             ballOrder[i] = GREEN;
//             cout << "5 green\n";
//         }
//         else
//         {
//             ballOrder[i] = 0;
//         }  
            


//     }
//     /* row_CheckBallOrder = roi_init.rows;
//     col_CheckBallOrder = roi_init.cols; */
//     #ifdef STARTCHECK
//         //记得便利图像读取颜色数值时候需要读取彩色图，而不是虑色后的图
//         for(int j = x - radius; i <= x + radius; i++)
//         {
//             for(int k = y- radius; k*k + j*j < radius * radius;k++)
//             {
//                 //h=;s=;v=;
//                 if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[0] ++;
//                 if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[1] ++;
//                 if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[2] ++;
//                 if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[3] ++;
//                 if(((h<=)||(h>=))||((s<=)||(s>=))||((v<=)||(v>=))) colorCnt[4] ++;
//             }
//         }
//         int cntmax=0;
//         for(int i = 0; i < 4;i++)
//         {
//             if(colorCnt[i] < colorCnt[i+1]) 
//                 cntmax = colorCnt[i+1];
//         }
//         for(int i = 0; i < 5;i++)
//         {
//             if( cntmax ==  colorCnt[i])
//                 g_call_color_now_1st = i;
//         }
//         #endif


//     //for()

    
//     if(CheckBallOrder_ifreasonable())
//         return 0;//这次没找到
//     if(CheckBallOrder_ifreasonable())
//         return 1;//找到了
// }