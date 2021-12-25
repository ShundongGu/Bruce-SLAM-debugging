/* +---------------------------------------------------------------------------+
   |                   ZiyunDongqi Technology Programming                      |
   |                                                                           |
   | Copyright (c++) 2020-2021, Group Contributors, Bruce Coding Team          |
   | 						All rights reserved.                   			   |
   | File 	 : CChristmasHappy.cpp    	                 						   |
   | Author  : Bruce													       |
   | Version : V1.0     													   |
   +---------------------------------------------------------------------------+ 

*	涉及cmake编译cpp工程及shell脚本自动化 
* 涉及动态库和静态库的编译并链接使用
* 涉及Google Glog日志库的使用
* 涉及C++基础数学库打印输出圣诞树二维模型

*******************************************************************************/

/****************************** File includes *********************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "libHelloSLAM.h"  // 引用编译的静态库头文件
#include "glog/logging.h"
#include "glog/raw_logging.h"

/*----------------------------const Variables---------------------------------*/ 
#define PI 3.14159265359
#define T px + scale * r * cosf(theta), py + scale * r * sin(theta)
static float sx = 0, sy = 0;

/*-------------------------------functions-----------------------------------*/
float sdCircle(float px, float py, float r) {
    float dx = px - sx, dy = py - sy;
    return sqrtf(dx * dx + dy * dy) - r;
}

float opUnion(float d1, float d2) {
    return d1 < d2 ? d1 : d2;
}
 
float f(float px, float py, float theta, float scale, int n) {
    float d = 0.0f;
    for (float r = 0.0f; r < 0.8f; r += 0.02f)
    d = opUnion(d, sdCircle(T, 0.05f * scale * (0.95f - r)));
    if (n > 0)
      for (int t = -1; t <= 1; t += 2) {
        float tt = theta + t * 1.8f;
        float ss = scale * 0.9f;
        for (float r = 0.2f; r < 0.8f; r += 0.1f) {
          d = opUnion(d, f(T, tt, ss * 0.5f, n - 1));
          ss *= 0.8f;
        }
      }
  return d;
}

int main( int argc, char** argv )
{
    //FLAGS_log_dir=".";   //设置log目录  没有指定则输出到控制台
    FLAGS_logtostderr = 1;  //输出到控制台
    std::string test = "this is slam demo";
    int i = 2, number = 8;
    int n = argc > 1 ? atoi(argv[1]) : 3;
    google::InitGoogleLogging(argv[0]);  // 初始化glog
    printHello();  // 调用libHelloSLAM库接口
    LOG(INFO) << test << "start";     // 打印log：“hello glog.  类似于C++ stream
/*
    LOG_IF(INFO, number > 10) << "number >  10"; 
    LOG_IF(INFO, number < 10) << "number <  10";
    for(i=0; i<20 ;i++){
        LOG_EVERY_N(INFO, 5) << "log i = " << i;
    }
    LOG(WARNING) << "It is error info"; 
    LOG(ERROR) << "It is error info"; 
    DLOG(INFO) << "it is debug mode";
    DLOG_IF(INFO, number > 10) << "debug number > 10";  
    RAW_LOG(INFO, "it is pthread log");
*/
    for (sy = 0.8f; sy > 0.0f; sy -= 0.02f, putchar('\n'))
        for (sx = -0.35f; sx < 0.35f; sx += 0.01f)
            putchar(f(0, 0, PI * 0.5f, 1.0f, n) < 0 ? '*' : ' ');
    return 0;
}


