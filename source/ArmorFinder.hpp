#pragma once

#include <opencv2/opencv.hpp>

#include "imageshow.hpp"
#include "target.hpp"

class ArmorFinder {
    bool _colorMode = true;   // 模式:红t蓝f
    bool _useDialte = false;  // 是否膨胀

    static bool shareEdge(const Target &t1, const Target &t2) {
        if (t1.pixelPts2f.tr == t2.pixelPts2f.tl && t1.pixelPts2f.br == t2.pixelPts2f.bl) {
            return true;
        }
        if (t2.pixelPts2f.tr == t1.pixelPts2f.tl && t2.pixelPts2f.br == t1.pixelPts2f.bl) {
            return true;
        }
        return false;
    }

  public:
    ArmorFinder() {}

    bool colorMode() const { return _colorMode; }
    void colorMode(bool val) { _colorMode = val; }

    bool useDialte() const { return _useDialte; }
    void useDialte(bool val) { _useDialte = val; }

    // 判断两个灯条区域是同一个灯条
    bool isSameBlob(Light blob1, Light blob2) {
        auto dist = blob1.centerPt - blob2.centerPt;
        return (dist.x * dist.x + dist.y * dist.y) < 9;
    }

    // 轮廓面积和其最小外接矩形面积之比
    double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
        return cv::contourArea(contour) / rect.size.area();
    }

    /**
     * 对contours进行筛选
     * 
     */
    bool selectBlobs(std::vector<cv::Point2i> &contour,Light &light){
        // 长度宽度筛选
        cv::RotatedRect rRect = cv::minAreaRect(contour);
        if (rRect.size.height < 10 || rRect.size.width < 10) {
            return false;
        }
        // 面积比
        double area = cv::contourArea(contour);
        if (area / rRect.size.area() < 0.3) {
           return false;
        }
        // 最小外接矩形长宽比2/3～3/2
        double hw = rRect.size.width / rRect.size.height;
        if (hw > 0.666667 && hw < 1.5) {
            return false;
        }

        /* 寻找灯条的顶部中点，底部中点与倾斜角 */
        Light _light;
        cv::Point2f topPt;     //顶部中点
        cv::Point2f bottomPt;  //底部中点
        cv::Point2f pts[4];    // 四个角点
        rRect.points(pts);
        if (rRect.size.width > rRect.size.height)  //根据外接矩形的特性需调整点
        {
            bottomPt = (pts[2] + pts[3]) / 2.0;
            topPt = (pts[0] + pts[1]) / 2.0;
            _light.angle = cv::abs(rRect.angle);
        } else {
            bottomPt = (pts[1] + pts[2]) / 2;
            topPt = (pts[0] + pts[3]) / 2;
            _light.angle = cv::abs(rRect.angle - 90);
        }
        /* 判断顶部和底部中点是否设置正确，并将中心点与长度一并写入_light参数中 */
        if (topPt.y > bottomPt.y) {
            _light.topPt = bottomPt;
            _light.bottomPt = topPt;
        } else {
            _light.topPt = topPt;
            _light.bottomPt = bottomPt;
        }
        _light.centerPt = rRect.center;              //中心点
        _light.length = cv::norm(bottomPt - topPt);  //长度

        /* 判断长度和倾斜角是否合乎要求 */
        if (_light.length < 3.0 || 800.0 < _light.length || cv::abs(_light.angle - 90) > 30.0) {
            return false;
        }

        light = _light;

        return true;
    }

    /**
     * 通过hsv筛选和进行预处理获得装甲板
     * @change m_preTargets 预检测得到的装甲板列表, 可能有两个装甲板共享一个灯条的情况发生
     */
    void detect(const cv::Mat &bgr, std::vector<Target> &m_preTargets, ImageShowClient &m_is, const cv::Point2f &m_startPt)  {
        cv::Mat bgrChecked;
        cv::Mat color_channel;
        cv::Mat src_bin_light, src_bin_dim;
        std::vector<cv::Mat> channels;       // 通道拆分

        /* 使用inRange对颜色进行筛选: bgr -> bgrChecked */
        m_is.clock("inRange");
        // if (_colorMode) {
        //     /* 红色 */
        //     cv::inRange(bgr, cv::Scalar(0, 0, 140), cv::Scalar(70, 70, 255), bgrChecked);
        // } else {
        //     /* 蓝色 */
        //     cv::inRange(bgr, cv::Scalar(130, 100, 0), cv::Scalar(255, 255, 65), bgrChecked);
        // }
        m_is.clock("inRange");

        //channel
        cv::split(bgr, channels);               /************************/
        if (_colorMode == 0) {                  /*        BLUE          */
            color_channel = channels[0];        /* 根据目标颜色进行通道提取 */
        } else if (_colorMode == 1) {           /*        RED           */
            color_channel = channels[2];        /************************/
        }

        int light_threshold;
        if(_colorMode == 0){
            light_threshold = 225;
            cv::threshold(color_channel, src_bin_light, light_threshold, 255, cv::THRESH_BINARY); // 二值化对应通道
            if (src_bin_light.empty()) return ;

            cv::threshold(color_channel, src_bin_dim, 140, 255, cv::THRESH_BINARY); // 二值化对应通道
            if (src_bin_dim.empty()) return ;
        }else{
            light_threshold = 200;
            cv::threshold(color_channel, src_bin_light, light_threshold, 255, cv::THRESH_BINARY); // 二值化对应通道
            if (src_bin_light.empty()) return ;

            cv::threshold(color_channel, src_bin_dim, 140, 255, cv::THRESH_BINARY); // 二值化对应通道
            if (src_bin_dim.empty()) return ;
        }
        // cv::threshold(color_channel, src_bin_light, light_threshold, 255, cv::THRESH_BINARY); // 二值化对应通道
        // if (src_bin_light.empty()) return ;

        // cv::threshold(color_channel, src_bin_dim, 140, 255, cv::THRESH_BINARY); // 二值化对应通道
        // if (src_bin_dim.empty()) return ;
        //

        /* 进行膨胀操作（默认关闭）: bgrChecked -> bgrChecked */
        // m_is.addImg("bgrChecked", bgrChecked, false);
        if (_useDialte) {
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            dilate(bgrChecked, bgrChecked, element);
            m_is.addImg("dilate", bgrChecked, false);
        }

        //
        // 使用两个不同的二值化阈值同时进行灯条提取，减少环境光照对二值化这个操作的影响。
        // 同时剔除重复的灯条，剔除冗余计算，即对两次找出来的灯条取交集。
        std::vector<std::vector<cv::Point2i>> light_contours_light, light_contours_dim;
        std::vector<Light> light_blobs_light,light_blobs_dim;
        std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
        cv::findContours(src_bin_light, light_contours_light, hierarchy_light, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

        for (int i = 0; i < light_contours_light.size(); i++) {
            if (hierarchy_light[i][2] == -1) {
                cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
                Light light;
                if (selectBlobs(light_contours_light[i],light)) {
                    light.area_ratio = areaRatio(light_contours_light[i], rect);
                    light_blobs_light.emplace_back(light);
                    // light_blobs_light.emplace_back(
                    //         rect, areaRatio(light_contours_light[i], rect), get_blob_color(src, rect)
                    // );
                }
            }
        }
        for (int i = 0; i < light_contours_dim.size(); i++) {
            if (hierarchy_dim[i][2] == -1) {
                cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
                Light light;
                if (selectBlobs(light_contours_dim[i], light)) {
                    light.area_ratio = areaRatio(light_contours_dim[i], rect);
                    light_blobs_dim.emplace_back(light);             
                    // light_blobs_dim.emplace_back(
                    //         rect, areaRatio(light_contours_dim[i], rect), get_blob_color(src, rect)
                    // );
                }
            }
        }
        std::vector<int> light_to_remove, dim_to_remove;
        for (int l = 0; l != light_blobs_light.size(); l++) {
            for (int d = 0; d != light_blobs_dim.size(); d++) {
                if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                    if (light_blobs_light[l].area_ratio > light_blobs_dim[d].area_ratio) {
                        dim_to_remove.emplace_back(d);
                    } else {
                        light_to_remove.emplace_back(l);
                    }
                }
            }
        }

        std::vector<Light> lights;

        std::sort(light_to_remove.begin(), light_to_remove.end(), [](int a, int b) { return a > b; });
        std::sort(dim_to_remove.begin(), dim_to_remove.end(), [](int a, int b) { return a > b; });
        for (auto x : light_to_remove) {
            light_blobs_light.erase(light_blobs_light.begin() + x);
        }
        for (auto x : dim_to_remove) {
            light_blobs_dim.erase(light_blobs_dim.begin() + x);
        }
        for (const auto &light : light_blobs_light) {
            lights.emplace_back(light);
        }
        for (const auto &dim : light_blobs_dim) {
            lights.emplace_back(dim);
        }
        //

        /* 寻找边缘，并圈出contours: bgrChecked -> contours */
        //std::vector<std::vector<cv::Point2i>> contours;
        //cv::findContours(bgrChecked, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // m_is.addContours("contours", contours, m_startPt);

        /* 对contours进行筛选 */
        //std::vector<Light> lights;
        // for (const auto &_pts : contours) {

        //     // 长度宽度筛选
        //     cv::RotatedRect rRect = cv::minAreaRect(_pts);
        //     if (rRect.size.height < 3 || rRect.size.width < 3) {
        //         continue;
        //     }
        //     // 面积比
        //     double area = cv::contourArea(_pts);
        //     if (area / rRect.size.area() < 0.3) {
        //         continue;
        //     }
        //     // 最小外接矩形长宽比2/3～3/2
        //     double hw = rRect.size.width / rRect.size.height;
        //     if (hw > 0.666667 && hw < 1.5) {
        //         continue;
        //     }

        //     /* 寻找灯条的顶部中点，底部中点与倾斜角 */
        //     Light _light;
        //     cv::Point2f topPt;     //顶部中点
        //     cv::Point2f bottomPt;  //底部中点
        //     cv::Point2f pts[4];    // 四个角点
        //     rRect.points(pts);
        //     if (rRect.size.width > rRect.size.height)  //根据外接矩形的特性需调整点
        //     {
        //         bottomPt = (pts[2] + pts[3]) / 2.0;
        //         topPt = (pts[0] + pts[1]) / 2.0;
        //         _light.angle = cv::abs(rRect.angle);
        //     } else {
        //         bottomPt = (pts[1] + pts[2]) / 2;
        //         topPt = (pts[0] + pts[3]) / 2;
        //         _light.angle = cv::abs(rRect.angle - 90);
        //     }
        //     /* 判断顶部和底部中点是否设置正确，并将中心点与长度一并写入_light参数中 */
        //     if (topPt.y > bottomPt.y) {
        //         _light.topPt = bottomPt;
        //         _light.bottomPt = topPt;
        //     } else {
        //         _light.topPt = topPt;
        //         _light.bottomPt = bottomPt;
        //     }
        //     _light.centerPt = rRect.center;              //中心点
        //     _light.length = cv::norm(bottomPt - topPt);  //长度

        //     /* 判断长度和倾斜角是否合乎要求 */
        //     if (_light.length < 3.0 || 800.0 < _light.length || cv::abs(_light.angle - 90) > 30.0) {
        //         continue;
        //     }

        //     lights.emplace_back(_light);
        // }
        m_is.addLights("lights", lights, m_startPt);

        /* 对筛选出的灯条按x大小进行排序 */
        std::sort(lights.begin(), lights.end(), [](const Light &a_, const Light &b_) -> bool {
            return a_.centerPt.x < b_.centerPt.x;
        });

        /* 对灯条进行两两组合并筛选出预检测的装甲板 */
        for (size_t i = 0; i < lights.size(); ++i) {
            for (size_t j = i + 1; j < lights.size(); ++j) {
                const auto &li = lights[i];
                const auto &lj = lights[j];

                double maxLength = cv::max(li.length, lj.length);
                double minLength = cv::min(li.length, lj.length);
                double avgLength = (li.length + lj.length) / 2.0;
                double deltaAngle = cv::abs(li.angle - lj.angle);
                // 两灯条倾斜角
                // if (deltaAngle > 20.0) {
                //     continue;
                // }
                cv::Vec2f crossVec = lj.centerPt - li.centerPt;
                double crossVecLength = cv::norm(crossVec);
                float avgAngle = (li.angle + lj.angle) / 2.0 / 180.0 * M_PI;  // in rad
                // 垂直坐标差值
                if (li.bottomPt.y < lj.topPt.y || lj.bottomPt.y < li.topPt.y) {
                    printf("Vertical \n");
                    continue;
                }
                // if (abs(crossVec[1]) > minLength) {
                //     continue;
                // }
                // 水平坐标差值
                if (crossVecLength <= minLength || crossVecLength > 5 * minLength) {
                    printf("Horizontal \n");
                    continue;
                }
                // 平行四边形度
                double crossAngle = cv::fastAtan2(crossVec[1], crossVec[0]) / 180.0 * M_PI;
                if (abs(avgAngle - crossAngle) >= 45) {
                    printf("angle45 \n");
                    continue;
                }
                // 长边短边比
                //if (maxLength > 1.5 * minLength) {
                //    continue;
                //}

                Target target;
                /* 计算像素坐标 */
                target.setPixelPts(li.topPt, li.bottomPt, lj.bottomPt, lj.topPt,
                    m_startPt);

                /** 
                 * small: 135 x 55 -> 2.454
                 * large: 230 x 55 -> 4.1818
                 */
                if (crossVecLength / minLength > 2.5)
                    target.type = TARGET_LARGE;  // 大装甲

                /* 获得扩展区域像素坐标, 若无法扩展则放弃该目标 */
                if (!target.convert2ExternalPts2f())
                    continue;
                m_preTargets.emplace_back(std::move(target));
            }
        }
        m_is.addTargets("preTargets", m_preTargets);
    }

};
