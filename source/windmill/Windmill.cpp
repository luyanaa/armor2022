#include "windmill/Windmill.hpp"
#include "Tool.hpp"

namespace wm {

//----------------------------------------------------------------------------------------------------------------------
// 下面是一些工具函数
// ---------------------------------------------------------------------------------------------------------------------
// 此函数用于改变hsv阈值
void set_hsv(int &LowH, int &LowS, int &LowV, int &HighH, int &HighS, int &HighV)
{
    LowH = 100;
    LowS = 0;
    LowV = 220;//200//220

    HighH = 200;
    HighS = 255;//255//236
    HighV = 255;
};
// 此函数用于绘制旋转矩形
void draw_rotated(const cv::Mat &mat, cv::RotatedRect &t, cv::Scalar color = cv::Scalar(0, 255, 0)) //旋转矩形绘制
{
    cv::Point2f *vertices = new cv::Point2f[4];
    t.points(vertices);
    for (size_t i = 0; i < 4; i++)
        cv::line(mat, vertices[i], vertices[(i + 1) % 4], color, 2);
}
// 此函数计算向量p和向量q的叉乘
double _dot(cv::Point2f p, cv::Point2f q)
{
    return p.x * q.y - q.x * p.y;
}; 
// 此函数用于判断线段AB和线段CD是否有交点 详见博客https://www.cnblogs.com/tuyang1129/p/9390376.html
bool _judge(cv::Point2f A, cv::Point2f B, cv::Point2f C, cv::Point2f D)
{
    cv::Point2f AC = C - A;
    cv::Point2f AB = B - A;
    cv::Point2f AD = D - A;
    cv::Point2f CA = A - C;
    cv::Point2f CD = D - C;
    cv::Point2f CB = B - C;
    if (_dot(AB, AC) * _dot(AB, AD) >= 0)
        return false;
    if (_dot(CD, CA) * _dot(CD, CB) >= 0)
        return false;
    return true;
};

inline double distance(cv::Point A, cv::Point B)
{
    return sqrt(pow((A.x - B.x), 2) + pow((A.y - B.y), 2));
}

void pushInOrder(std::vector<cv::Point2f>& dst, const cv::Point2f* src, const std::initializer_list<size_t>& order )
{
    for (auto&& i : order)
        dst.push_back(src[i]);
}

double Windmill::templateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method)
{
    int result_cols = image.cols - tepl.cols + 1;
    int result_rows = image.rows - tepl.rows + 1;
    //std::cout << "rc" << result_cols << " " << result_rows << std::endl;
    cv::Mat result = cv::Mat(result_cols, result_rows, CV_32FC1);
    cv::matchTemplate(image, tepl, result, method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    switch (method)
    {
    case cv::TM_SQDIFF:
    case cv::TM_SQDIFF_NORMED:
        point = minLoc;
        return minVal;

    default:
        point = maxLoc;
        return maxVal;
    }
}

//----------------------------------------------------------------------------------------------------------------------
// 此函数通过pitch yaw和旋转顺序计算旋转矩阵
// 输入：pitch yaw和旋转顺序
// 输出：旋转矩阵
// ---------------------------------------------------------------------------------------------------------------------
bool Windmill::run(const cv::Mat &frame, float &pitch, float &yaw, double time) 
{
    findedTarget.clear();
    hitTarget.clear();
    timeStamp = time;
    if (detect(frame)) {//如果找到目标
        //计算从装甲片坐标系到摄像头坐标系的旋转矩阵和平移矩阵
        calRvTv(findedTarget.vertexs, WIDTH, HEIGHT, camMatrix, distCoeffs, findedTarget.RvTtoL, findedTarget.TvTtoL);
        //进行目标点的预测 需要用到上面计算的两个矩阵
        predictTarget();
        //计算pitch和yaw角度
        hit(pitch, yaw);                                                   
        return true;
    }
    // STATE(tensorflow::INFO, "no target", 1)
    return false;
};

//----------------------------------------------------------------------------------------------------------------------
// 此函数检测装甲板目标并将目标存入数据成员findedTarget里
// 输入：
// 输出：
// ---------------------------------------------------------------------------------------------------------------------
bool Windmill::detect(const cv::Mat &frame) 
{
    assert(!frame.empty());
#ifdef TIME
    auto t1 = std::chrono::high_resolution_clock::now();
#endif

    bool isFind = false;//是否能在这一帧里找到目标
    std::vector<cv::Mat> tmp(8);//分类用装甲板模板

    //auto t3 = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < 8; ++i)
    {
        cv::Mat templatePic = cv::imread(
            "../pics/template/template" + std::to_string(i + 1) + ".jpg", 
            cv::IMREAD_GRAYSCALE);
        assert(!templatePic.empty());
        tmp[i] = templatePic;
    }
    //auto t4 = std::chrono::high_resolution_clock::now();
    //std::cout << "load period: " << (static_cast<std::chrono::duration<double, std::milli>>(t4 - t3)).count() << " ms" << std::endl;

    std::vector<cv::Mat> imgChannels;
    cv::split(frame, imgChannels);
#ifdef RED
    cv::Mat midImage2 = imgChannels.at(2) - imgChannels.at(0);
#else   
    cv::Mat midImage2 = imgChannels.at(0) - imgChannels.at(2);
#endif
    cv::threshold(midImage2, midImage2, 100, 255, cv::THRESH_BINARY);
#ifdef DEBUG
    cv::imshow("binaryImage", midImage2);
#endif
    //is->addImg("binaryImage", midImage2);

    int structElementSize = 2;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * structElementSize + 1, 2 * structElementSize + 1),
        cv::Point(structElementSize, structElementSize)
    );
    cv::dilate(midImage2, midImage2, element, {-1,-1}, 2);
    structElementSize = 3;
    element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * structElementSize + 1, 2 * structElementSize + 1),
        cv::Point(structElementSize, structElementSize)
    );
    cv::morphologyEx(midImage2, midImage2, cv::MORPH_CLOSE, element);
#ifdef DEBUG
    cv::imshow("dilate", midImage2);
#endif

    is->addImg("dilate", midImage2);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(
        midImage2,
        contours,
        hierachy,
        cv::RETR_TREE,
        cv::CHAIN_APPROX_SIMPLE
    );
    cv::RotatedRect contourRect;
    if(hierachy.size() == 0)
        return isFind = false;
    for(int i = 0; i >= 0; i = hierachy[i][0])
    {
        contourRect = cv::minAreaRect(contours[i]);
        cv::Point2f contourRectVertex[4];
        contourRect.points(contourRectVertex);

        cv::Point2f srcRect[4], dstRect[4];
        double width = wm::distance(
            contourRectVertex[0],
            contourRectVertex[1]
        ),     height = wm::distance(
            contourRectVertex[1],
            contourRectVertex[2]
        );
        if(width > height)
            for(size_t j = 0; j < 4; ++j)
                srcRect[j] = contourRectVertex[j];
        else
        {
            std::swap(width, height);
            for(size_t j = 0; j < 4; ++j)
                srcRect[j] = contourRectVertex[(j + 1) % 4];
        }

        double area = height * width;
        if(area < 800 && area > 60000)
            continue;

        if(area > 200 && area < 2000 && width / height > 1 && width / height < 2)
        {
            this->center = contourRect.center;
            is->addCircle("circle center", this->center);
            continue;
        }

        dstRect[0] = cv::Point2f(0, 0);
        dstRect[1] = cv::Point2f(width, 0);
        dstRect[2] = cv::Point2f(width, height);
        dstRect[3] = cv::Point2f(0, height);
        cv::Mat transform = cv::getPerspectiveTransform(srcRect, dstRect);
        cv::Mat perspectMat, testim;
        cv::warpPerspective(
            midImage2,
            perspectMat,
            transform,
            cv::Size(2000, 1000) // 开大点
        );
        std::cout << dstRect[0] << ' ' << dstRect[1] << ' ' << dstRect[2] << ' ' << dstRect[3] << ' ';
printf("w:%f, h:%f; w: %d, h:%d\n",width, height, perspectMat.size[0], perspectMat.size[1]);
        testim = perspectMat(cv::Rect(0, 0, width, height));
        cv::Point matchLoc;
        cv::Mat templ;
        cv::resize(testim, templ, cv::Size(42, 20));
#ifdef DEBUG
        cv::imshow("warpdst", perspectMat);   
        cv::imshow("testim", testim);
        cv::imshow("templ", templ); 
#endif 
        std::vector<double> vValue1, vValue2;
        for(size_t j = 0; j < 6; ++j)
        {
            double value = templateMatch(templ, tmp[j], matchLoc, cv::TM_CCOEFF_NORMED);
            vValue1.push_back(value);
        }
        for(size_t j = 6; j < 8; ++j)
        {
            double value = templateMatch(templ, tmp[j], matchLoc, cv::TM_CCOEFF_NORMED);
            vValue2.push_back(value);
        }
        auto maxv1 = std::max_element(vValue1.begin(), vValue1.end());
        auto maxv2 = std::max_element(vValue2.begin(), vValue2.end());
        cv::Point tgtcenter;
        if(*maxv1 > *maxv2 && *maxv1 > 0.6)
        {
            if(hierachy[i][2] < 0)
                continue;
            cv::RotatedRect tgt = cv::minAreaRect(contours[hierachy[i][2]]);
            cv::Point2f tgtVertex[4];
            tgt.points(tgtVertex);
            constexpr float maxHWRation = 0.7153846f;
            constexpr float maxArea = 6000.f, minArea = 500.f;

            if (area > maxArea || area < minArea || height / width > maxHWRation);
                //TODO

            isFind = true;
            float width = tgt.size.width, height = tgt.size.height;
            if(height > width)
                std::swap(height, width);
            float area = height * width;
            tgtcenter = tgt.center;
            // is->addCircle("center", tgtcenter);
            double radius = distance(tgtcenter, this->center);
            // is->addCircle("circle", this->center, radius, 2);
            std::vector<cv::RotatedRect> target = {tgt};
            is->addRotatedRects("targrt", target);
            
            std::vector<cv::Point2f> tgtVertexInOrder;

            if      (_judge(tgtVertex[0], tgtVertex[1], this->center, tgtcenter))
                pushInOrder(tgtVertexInOrder, tgtVertex, {2, 1, 0, 3});
            else if (_judge(tgtVertex[1], tgtVertex[2], this->center, tgtcenter))
                pushInOrder(tgtVertexInOrder, tgtVertex, {3, 2, 1, 0});
            else if (_judge(tgtVertex[2], tgtVertex[3], this->center, tgtcenter))
                pushInOrder(tgtVertexInOrder, tgtVertex, {0, 3, 2, 1});
            else if (_judge(tgtVertex[3], tgtVertex[0], this->center, tgtcenter))
                pushInOrder(tgtVertexInOrder, tgtVertex, {1, 0, 3, 2});
            else
                continue;

            findedTarget.vertexs = std::move(tgtVertexInOrder);
        }
        
    }

#ifdef TIME
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Total period: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
#endif
    return isFind;
};

//----------------------------------------------------------------------------------------------------------------------
// 此函数计算要打击的目标点的世界坐标系坐标
// 输入：
// 输出：
// ---------------------------------------------------------------------------------------------------------------------
void Windmill::predictTarget() {
    //预测子弹飞行过程中扇叶旋转的角度 delay为飞行的时间 v是飞行的速度
    float theta = this->delay * this->v;
    this->hitTarget = this->findedTarget;
    //预测的目标点在装甲板坐标系中的坐标
    (this->hitTarget).tPt = cv::Point3f(sin(theta) * this->radius + WIDTH / 2, -(this->radius - cos(theta) * this->radius - HEIGHT / 2), 0.0);
    (this->hitTarget).vertexs.clear();
    
    cv::Point2f outputPt;
    cv::Point2f detectedPt;
    //将击打的目标点从三维装甲板坐标系转换到二维图片坐标系
    coodiTrans(hitTarget.tPt, outputPt, 
                hitTarget.RvTtoL, hitTarget.TvTtoL,
                camMatrix, distCoeffs);
    //将检测的目标点从三维装甲板坐标系转换到二维图片坐标系
    coodiTrans(cv::Point3f(WIDTH / 2, HEIGHT / 2, 0.0), detectedPt, 
                hitTarget.RvTtoL, hitTarget.TvTtoL,
                camMatrix, distCoeffs);

    //要击打的目标
    hitTarget.vertexs.emplace_back(outputPt.x - 5, outputPt.y + 5);
    hitTarget.vertexs.emplace_back(outputPt.x - 5, outputPt.y - 5);
    hitTarget.vertexs.emplace_back(outputPt.x + 5, outputPt.y - 5);
    hitTarget.vertexs.emplace_back(outputPt.x + 5, outputPt.y + 5);

    //检测到的目标
    std::vector<cv::Point2f> detectedPts;
    detectedPts.emplace_back(detectedPt.x - 5, detectedPt.y + 5);
    detectedPts.emplace_back(detectedPt.x - 5, detectedPt.y - 5);
    detectedPts.emplace_back(detectedPt.x + 5, detectedPt.y - 5);
    detectedPts.emplace_back(detectedPt.x + 5, detectedPt.y + 5);

    is->addEvent("detected target", detectedPts);
    is->addEvent("hit target", hitTarget.vertexs);
    return;
};

//----------------------------------------------------------------------------------------------------------------------
// 此函数计算要返回给电控的pitch和yaw角度
// 输入：
// 输出：
// ---------------------------------------------------------------------------------------------------------------------
void Windmill::hit(float &pitch, float &yaw) 
{
    //计算打击目标点在摄像头坐标系下的坐标
    coodiTrans(hitTarget.tPt, hitTarget.lPt, hitTarget.RvTtoL, hitTarget.TvTtoL);
    cv::Mat eye = cv::Mat::eye(3, 3, CV_64FC1);
    //计算打击目标点在云台坐标系下的坐标
    coodiTrans(hitTarget.lPt, hitTarget.lPt, eye, this->TvCtoL);
    //计算打击目标点的pitch和yaw角度
    calPitchYaw(hitTarget.lPt, pitch, yaw);
    
    if (delay > 0) {//顺时针旋转
        //pitch -= maxPitchError * cos(spinAngle / 180.0 * M_PI);
        //yaw += maxYawError * sin(spinAngle / 180 * M_PI);
    }
    else if (delay < 0) {//逆时针旋转
        //pitch += maxPitchError * cos(spinAngle / 180.0 * M_PI);
        //yaw -= maxYawError * sin(spinAngle / 180 * M_PI);
    }
    else;

    if (this->state == JUMP && !(pitch < 2.0 && yaw < 2.0)) 
        this->state = JUMP;
    else if (this->state == JUMP && pitch < 2.0 && yaw < 2.0) 
        this->state = TRACK;
    else if (this->state == TRACK && pitch < 2.0 && yaw < 2.0) {
        this->stateCount = 0;
        this->state = TRACK;
    }
    else if (this->state == TRACK && !(pitch < 2.0 && yaw < 2.0)){
        (this->stateCount)++;
        if (this->stateCount >= 3) 
            this->state = JUMP;
        else 
            pitch = yaw = 0.0;
    }
    else 
        abort();    
}
};