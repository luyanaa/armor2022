//2021 1 26
#ifndef WINDMILL_HPP
#define WINDMILL_HPP

#include <string>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "base.hpp"
#include "imageshow.hpp"
#include "Target.hpp"
#include "opencv2/opencv.hpp"

#define WIDTH 215                          // 装甲板宽度相对值
#define HEIGHT 125                         // 装甲板高度相对值
#define RADIUS 740                         // 风车中心到边缘装甲板中心相对值(风车半径相对值)
#define ANGULAR_VELOCITY 60 / 180.0 * M_PI // 风车角速度
#define EPS 1e-2                           // 误差值

namespace wm
{
    typedef enum {TRACK, JUMP} State;//风车的状态
    //----------------------------------------------------------------------------------------------------------------------
    // 风车类
    //    为了保持接口不变，保留了之前的数据成员，但是很多没有用到
    // ---------------------------------------------------------------------------------------------------------------------
    class Windmill 
    {
    public:
    //datas
        std::string mode;      //风车模式 匀速还是三角函数运转
        State state = JUMP;    //状态值
        int stateCount = 0;    //状态计数器

        Target findedTarget;   //找到的目标 Target是目标类，详见Target.hpp
        Target hitTarget;      //预测后的击打目标

        cv::Point2f center;    //风车中心点坐标

        double timeStamp = 0.0;//相机时间戳
        double delay;          //子弹飞行延迟时间

        cv::Mat camMatrix;     //相机参数
        cv::Mat distCoeffs;    //畸变矩阵

        cv::Mat TvCtoL;        //相机到云台的变换矩阵

        float radius = RADIUS;     //风车半径
        float v = ANGULAR_VELOCITY;//旋转角速度

        float gPitch = 0.0;        //电控传来的pitch 云台与世界参照系
        float gYaw = 0.0;          //电控传来的yaw   云台与世界参照系

        int sampleNumber = 0;      //样本数目
        double spinAngle = 0.0;    //预测用到角度

        double maxPitchError = 0.0;//最大pitch误差
        double maxYawError = 0.0;  //最大yaw误差
        
        int thre;                  //二值化阈值
        bool close = true;                //灰度图是否进行形态学闭运算
    //fuctions
        bool run(const cv::Mat &frame, float &pitch, float &yaw, double time);
        bool detect(const cv::Mat &frame);
        void predictTarget();
        void hit(float &pitch, float &yaw);
        static Windmill *GetInstance(const cv::Mat &cam, const cv::Mat &dist,
                                     const cv::Mat &TvCtoL, const double delay, 
                                     const cv::String &modelName, ImageShowClient *is,
                                     const double maxPitchError, const double maxYawError) 
        {
            static Windmill instance(cam, dist, TvCtoL, delay, modelName, is,maxPitchError,maxYawError);
            return &instance;
        };
    private:
    //datas
        ImageShowClient *is;
    //fuctions
        explicit Windmill(const cv::Mat &cam, const cv::Mat &dist, 
                          const cv::Mat &TvCtoL, const double delay,
                          const cv::String& modelName, ImageShowClient *is,
                          const double maxPitchError, const double maxYawError)
                : camMatrix(cam), distCoeffs(dist), TvCtoL(TvCtoL), delay(delay), is(is), maxPitchError(maxPitchError), maxYawError(maxYawError) {};

    };//end of class Windmill

    void set_hsv(int &LowH, int &LowS, int &LowV, int &HighH, int &HighS, int &HighV);
    
    bool _judge(cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D);

    double _dot(cv::Point2f p, cv::Point2f q);

    void draw_rotated(const cv::Mat &mat, cv::RotatedRect &t, cv::Scalar color);

    void set_hsv(int &LowH, int &LowS, int &LowV, int &HighH, int &HighS, int &HighV);

}
#endif
