//
// Created by 杨智宇 on 2019-04-27.
//

#pragma once

#include <climits>
#include <cstdio>
#include <iostream>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <chrono>
#include <condition_variable>

#include "DebugSocket.hpp"
#include "base.hpp"
#include "sort/sort.h"
#include "target.hpp"

/**
 * ImageShow 基类, 静态变量用于线程间同步和共享
 */
class ImageShowBase {
  protected:
    static std::vector<std::pair<int, cv::Mat>> s_frameVec;
    struct ImageData {
        cv::String name;
        cv::Mat mat;
        bool isUpdated = false;
    };
    static std::vector<std::vector<ImageData>> s_imgVec;
    static std::atomic_int s_currentCallID;
    static std::atomic<double> s_fontSize;
    static std::atomic_int s_mode;
    static std::atomic_bool s_isClockPrintEnable;
    static std::mutex s_mutex;
    static std::atomic_bool s_isPause;
    static std::atomic_int s_currentKey;
    static std::pair<int, double> s_cost;
    static std::atomic_bool s_isAverageCostPrint;
};

std::vector<std::pair<int, cv::Mat>> ImageShowBase::s_frameVec;
std::vector<std::vector<ImageShowBase::ImageData>> ImageShowBase::s_imgVec;
std::atomic_int ImageShowBase::s_currentCallID(0);
std::atomic<double> ImageShowBase::s_fontSize(1.25);
std::atomic_int ImageShowBase::s_mode(0);
std::atomic_bool ImageShowBase::s_isClockPrintEnable(false);
std::mutex ImageShowBase::s_mutex;
std::atomic_bool ImageShowBase::s_isPause(false);
std::atomic_int ImageShowBase::s_currentKey(-1);
std::pair<int, double> ImageShowBase::s_cost = std::pair<int, double>{0, 0};
std::atomic_bool ImageShowBase::s_isAverageCostPrint(false);
std::condition_variable s_conVar;
std::atomic_bool m_isWakeUp(false);
std::atomic_bool m_isWillExit(false);


/**
 * ImageShowClient 子线程绘图客户类
 */
class ImageShowClient : ImageShowBase {
  private:
    int m_id;  // 当前线程index
    std::vector<std::pair<cv::String, int64_t>> m_clocks;
    int64_t m_startClock;
    cv::Mat m_frame;
    std::vector<cv::Scalar> m_colorLibrary = {
        // rgb(255, 27, 19)
        // rgb(214, 200, 75)
        // rgb(224, 60, 192)
        // rgb(24, 208, 0)
        // rgb(250, 218, 141)
        // rgb(137, 190, 178)
        // rgb(254, 47, 101)
        // rgb(35, 255, 185)
        // rgb(0, 89, 239)
        // rgb(222, 125, 44)
        // rgb(69, 137, 148)
        cv::Scalar(137, 190, 178), cv::Scalar(255, 27, 19), cv::Scalar(35, 255, 185), cv::Scalar(214, 200, 75),
        cv::Scalar(224, 60, 192),
        cv::Scalar(24, 208, 0), cv::Scalar(250, 218, 141), cv::Scalar(254, 47, 101),
        cv::Scalar(0, 89, 239), cv::Scalar(222, 125, 44), cv::Scalar(69, 137, 148)};

    int m_currentColorIndex;

    cv::Scalar &m_getCurrentColor() {
        m_currentColorIndex = m_currentColorIndex > 10 ? 0 : m_currentColorIndex;
        return m_colorLibrary[m_currentColorIndex++];
    }

    cv::Point2d m_currentLeftMarginLoc;
    cv::Point2d m_currentRightMarginLoc;

    cv::Point2d &m_getCurrentMarginTextLoc(bool isLeftMargin = true, double txtXPos = 0) {
        if (isLeftMargin) {
            m_currentLeftMarginLoc.y += s_fontSize * 16.0 + 4.0;
            return m_currentLeftMarginLoc;
        } else {
            m_currentRightMarginLoc.y += s_fontSize * 16.0 + 4.0;
            m_currentRightMarginLoc.x = txtXPos;
            return m_currentRightMarginLoc;
        }
    }

    void m_putMarginText(const cv::String &txt, const cv::Scalar &color, int thickness, bool isLeftMargin = true) {
        double txtPos = 0;
        if (!isLeftMargin)
            txtPos = m_frame.cols - s_fontSize * 8.5 * txt.length() - s_fontSize * 16 - 5;
        cv::putText(m_frame, txt, m_getCurrentMarginTextLoc(isLeftMargin, txtPos), cv::FONT_HERSHEY_PLAIN,
            s_fontSize, color, thickness);
    }


    /**
     * 输出 clock 计时的结果, 有锁, 保证 client 间输出不互相打断
     */
    /*
    void m_clockPrint() {
        if (!s_isClockPrintEnable)
            return;

        double tickFreq = cv::getTickFrequency();
        double totalCost = 1000.0 * (cv::getTickCount() - m_startClock) / tickFreq;

        // 以下有锁, 保证client间输出不被打断
        std::unique_lock<std::mutex> lock(s_mutex);
        printf("\n| thread %d ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|\n", m_id);
        printf("|++++++++++ frame tick %9d ++++++++++|\n", s_frameVec[m_id].first);
        printf("|------ name ------|- per(%%) -|- cost(ms) -|\n");
        for (const auto &_clock : m_clocks) {
            double cost = 1000.0 * _clock.second / tickFreq;
            printf("| %16s | %8.3f | %10.4f |\n", _clock.first.c_str(), 100.0 * cost / totalCost, cost);
        }
        printf("|------------------|----------|------------|\n");
        printf("| %16s |  100.000 | %10.4f |\n", "update 2 show", totalCost);
        printf("|________________________________ thread %d |\n\n", m_id);
        m_clocks.clear();

        if (s_isAverageCostPrint) {
            s_cost.first++;
            // cpu有睿频, 不是一下子快起来的
            if (s_cost.first > 100) {
                s_cost.second += totalCost;
                printf("average cost = %.4fms \n", s_cost.second / (s_cost.first - 100));
            }
            if (s_cost.first > 100000) {
                s_cost.first = 101;
                s_cost.second = 0;
            }
        }
        lock.unlock();
        printf("m_clockPrint unlock\n");
    }
    */

  public:
    /**
     * 客户端构造函数
     */
    explicit ImageShowClient()
        : m_id(-1),
          m_startClock(0),
          m_currentColorIndex(0),
          m_currentLeftMarginLoc(cv::Point2d(0, 0)),
          m_currentRightMarginLoc(cv::Point2d(0, 0)) {
        // rgb to bgr
        for (auto &_color : m_colorLibrary) {
            const auto temp = _color[0];
            _color[0] = _color[2];
            _color[2] = temp;
        }
    }

    /**
     * 设置客户端编号
     * @param id 客户端编号
     */
    void setClientId(int id) { m_id = id; }

    /**
     * 拷贝传递本次处理的基础图像, 后续绘图都在这张图上
     * @param frame 基础图像
     * @param frameTick 帧编号
     */
    void update(const cv::Mat &frame, int frameTick) {
        /* clear */
        m_currentColorIndex = 0;
        m_currentLeftMarginLoc = cv::Point2d(0, 0);
        m_currentRightMarginLoc = cv::Point2d(0, 0);
        if (s_mode != 0) {
            frame.copyTo(m_frame);
            s_frameVec[m_id].first = frameTick;
        }
        if (s_isClockPrintEnable)
            m_startClock = cv::getTickCount();
    }

    /**
     * 在右上角的绘制文本, mode > 0 情况下均显示
     * @param txt 文本内容
     */
    void addText(const cv::String &txt) {
        if (s_mode == 0)
            return;
        int thickness = 1;
        m_putMarginText(txt, cv::Scalar(70, 123, 255), thickness, false);
    }
    /**
     * 绘制旋转矩形
     */
    void addRotatedRects(const cv::String &eventName, const std::vector<cv::RotatedRect> &rRects) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;
        for (const auto &rRect : rRects) {
            cv::Point2f pts[4];
            rRect.points(pts);
            for (int i = 0; i < 4; i++)
                cv::line(m_frame, pts[i], pts[(i + 1) % 4], currentColor, thickness);
        }
        m_putMarginText(eventName + cv::format(": %d", int(rRects.size())), currentColor, thickness);
    }

    void addEvent(const cv::String &eventName, const std::vector<cv::RotatedRect> &rRects) {
        addRotatedRects(eventName, rRects);
    }
    /**
     * 绘制矩形
     */
    void addRect(const cv::String &eventName, const cv::Rect &rect) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;

        cv::Point leftUp(rect.x, rect.y);
        cv::Point rightUp(rect.x + rect.width, rect.y);
        cv::Point leftDown(rect.x, rect.y + rect.height);
        cv::Point rightDown(rect.x + rect.width, rect.y + rect.height);

        line(m_frame, leftUp, leftDown, currentColor, thickness);
        line(m_frame, leftDown, rightDown, currentColor, thickness);
        line(m_frame, rightDown, rightUp, currentColor, thickness);
        line(m_frame, rightUp, leftUp, currentColor, thickness);
        m_putMarginText(eventName, currentColor, thickness);
    }

    void addEvent(const cv::String &eventName, const cv::Rect &rect) {
        addRect(eventName, rect);
    }

    void addCircle(const cv::String &eventName, const cv::Point &p) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;
        cv::circle(m_frame, p, 10, currentColor, -1);
        m_putMarginText(eventName, currentColor, thickness);
    }

    /**
     * 绘制点
     * @param eventName
     * @param contours
     */
    void addEvent(const cv::String &eventName, const cv::Point &p) {
        addCircle(eventName, p);
    }

    /**
     * 绘制轮廓
     * @param eventName
     * @param contours
     * @param offset (ROI使用)左上角坐标
     */
    void addContours(const cv::String &eventName, const std::vector<std::vector<cv::Point2i>> &contours, const cv::Point &offset) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;
        cv::drawContours(m_frame, contours, -1, currentColor, thickness, 8, cv::noArray(), INT_MAX, offset);
        m_putMarginText(eventName + cv::format(": %d", int(contours.size())), currentColor, thickness);
    }

    /**
     * 绘制轮廓
     * @param eventName
     * @param contours
     * @param offset
     */
    void addEvent(const cv::String &eventName, const std::vector<std::vector<cv::Point2i>> &contours, const cv::Point &offset) {
        addContours(eventName, contours, offset);
    }

    /**
     * 绘制灯条
     * @param eventName 事件名, 显示在左上角
     * @param lights 绘制对象
     */
    void addLights(const cv::String &eventName, const std::vector<Light> &lights, const cv::Point &offset) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor(0, 255, 0);
        int thickness = 2;
        for (const auto &light : lights) {
            cv::line(m_frame, cv::Point(light.topPt.x + offset.x, light.topPt.y + offset.y),
                cv::Point(light.centerPt.x + offset.x, light.centerPt.y + offset.y), currentColor, thickness);
            cv::line(m_frame, cv::Point(light.centerPt.x + offset.x, light.centerPt.y + offset.y),
                cv::Point(light.bottomPt.x + offset.x, light.bottomPt.y + offset.y), currentColor, thickness);
        }
        m_putMarginText(eventName + cv::format(": %d", int(lights.size())), currentColor, thickness);
    }

    /**
     * 绘制灯条, 与 addLights 相同
     * @param eventName 事件名, 显示在左上角
     * @param lights 绘制对象
     */
    void addEvent(const cv::String &eventName, const std::vector<Light> &lights, const cv::Point &offset) {
        addLights(eventName, lights, offset);
    }

    /**
     * 绘制目标
     * @param eventName 事件名, 显示在左上角
     * @param targets 绘制对象
     */
    void addTargets(const cv::String &eventName, const std::vector<Target> &targets) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor(0, 0, 255);
        int thickness = 2;
        for (const auto &_tar : targets) {
            cv::line(m_frame, _tar.pixelPts2f.tl, _tar.pixelPts2f.tr, currentColor, thickness);
            cv::line(m_frame, _tar.pixelPts2f.bl, _tar.pixelPts2f.br, currentColor, thickness);
        }
        m_putMarginText(eventName + cv::format(": %d", int(targets.size())), currentColor, thickness);
    }

    /**
     * 绘制目标
     * @param eventName 事件名, 显示在左上角
     * @param tracks 绘制对象
     */
    void addTracks(const std::vector<sort::Track> &tracks) {
        if (s_mode == 0 || s_mode == 1)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;
        for (const auto &_tar : tracks) {
            cv::rectangle(m_frame, _tar.bbox.tl(), _tar.bbox.br(), currentColor, thickness);
            cv::putText(m_frame, cv::format("%ld", _tar.id), _tar.bbox.tl() + cv::Point2f(12, 0),
                cv::FONT_HERSHEY_PLAIN, s_fontSize, currentColor);
        }
    }

    /**
     * 绘制目标, 与 addTargets 相同
     * @param eventName 事件名, 显示在左上角
     * @param targets 绘制对象
     */
    void addEvent(const cv::String &eventName, const std::vector<Target> &targets) {
        addTargets(eventName, targets);
    }

    /**
     * 绘制矩形区域
     * used by windmill
     * @param eventName
     * @param region
     */
    void addEvent(const cv::String &eventName, const std::vector<cv::Point2f> &region) {
        if (s_mode == 0)
            return;
        int thickness = 1;
        cv::Scalar currentColor = m_getCurrentColor();
        std::vector<cv::Point> region2;
        for (const auto &reg : region) {
            region2.emplace_back(cv::Point(reg));
        }
        std::vector<cv::Point> hull;
        cv::convexHull(region2, hull, true);
        polylines(m_frame, hull, true, currentColor, thickness);
        m_putMarginText(eventName + cv::format(": %d", int(region.size())), currentColor, thickness);
    }
    /**
     * 绘制旋转矩形区域
     * used by windmill
     * @param eventName
     * @param rRect
     */
    void addEvent(const cv::String &eventName, const cv::RotatedRect &rRect) {
        if (s_mode == 0)
            return;
        cv::Scalar currentColor = m_getCurrentColor();
        int thickness = 1;
        cv::Point2f pts[4];
        rRect.points(pts);
        for (int i = 0; i < 4; i++)
            line(m_frame, pts[i], pts[(i + 1) % 4], currentColor, thickness);
    }
    /**
     * 绘制矩形区域
     * used by windmill
     * @param eventName
     * @param sRotatedRect
     */
    template <typename T>
    void addEvent(const cv::String &eventName, const std::vector<std::vector<T>> &region) {
        if (s_mode == 0)
            return;
        int thickness = 1;
        cv::Scalar currentColor = m_getCurrentColor();
        std::vector<cv::Point> hull;
        std::vector<std::vector<cv::Point>> region2;
        int begin = region.size() - 70 > 0 ? region.size() - 70 : 0;
        int i = 0;
        for (const auto &regs : region) {
            if (i < begin) {
                i++;
                continue;
            }
            std::vector<cv::Point> tem;
            for (const auto &reg : regs) {
                tem.push_back(cv::Point(reg));
            }
            region2.push_back(tem);
        }
        for (const auto &pts : region2) {
            cv::convexHull(pts, hull, true);
            polylines(m_frame, hull, true, currentColor, thickness);
        }
        m_putMarginText(eventName + cv::format(": %d", int(region.size())), currentColor, thickness);
    }
    /**
     * 绘制过分类器的目标
     * @param eventName 事件名, 显示在左上角
     * @param targets 绘制对象
     */
    void addClassifiedTargets(const cv::String &eventName, const std::vector<Target> &targets) {
        if (s_mode == 0)
            return;
        cv::Scalar currentColor(0, 255, 0);
        int thickness = 2;
        for (const auto &_tar : targets) {
            cv::line(m_frame, _tar.pixelPts2f.tl, _tar.pixelPts2f.bl, currentColor, thickness);
            cv::line(m_frame, _tar.pixelPts2f.bl, _tar.pixelPts2f.br, currentColor, thickness);
            cv::line(m_frame, _tar.pixelPts2f.br, _tar.pixelPts2f.tr, currentColor, thickness);
            cv::line(m_frame, _tar.pixelPts2f.tr, _tar.pixelPts2f.tl, currentColor, thickness);
        }
        m_putMarginText(eventName + cv::format(": %d", int(targets.size())), currentColor, thickness);
    }

    /**
     * 绘制最终目标
     * @param eventName 事件名, 显示在左上角
     * @param targets 绘制对象
     */
    void addFinalTargets(const cv::String &eventName, const Target &target) {
        if (s_mode == 0)
            return;
        cv::Scalar currentColor(0, 255, 255);
        int thickness = 3;
        cv::line(m_frame, target.pixelPts2f.tl, target.pixelPts2f.bl, currentColor, thickness);
        cv::line(m_frame, target.pixelPts2f.bl, target.pixelPts2f.br, currentColor, thickness);
        cv::line(m_frame, target.pixelPts2f.br, target.pixelPts2f.tr, currentColor, thickness);
        cv::line(m_frame, target.pixelPts2f.tr, target.pixelPts2f.tl, currentColor, thickness);
        cv::putText(m_frame, cv::format("%d", target.rTick), target.pixelPts2f.tl + cv::Point2f(12, 0),
            cv::FONT_HERSHEY_PLAIN, s_fontSize, currentColor);
        cv::String carType = target.type == TARGET_SMALL ? "small" : "large";
        cv::putText(m_frame, carType, target.pixelPts2f.tl + cv::Point2f(12, 12),
            cv::FONT_HERSHEY_PLAIN, s_fontSize, currentColor);
        m_putMarginText(eventName, currentColor, thickness);
    }

    /**
     * 显示额外图像, 拷贝传递, 性能损失严重
     * @param winName 窗口名
     * @param img 图像
     * @param want 是否需要在 mode = 1 时显示
     */
    void addImg(const cv::String &winName, const cv::Mat &img, bool want = false) {
        if (s_mode == 0 || (s_mode == 1 && !want))
            return;
        size_t i = 0;
        size_t size = s_imgVec[m_id].size();

        for (i = 0; i < size; ++i) {
            if (s_imgVec[m_id][i].name == winName)
                break;
        }

        cv::Mat tempImg;
        img.copyTo(tempImg);
        if (i == size) {
            /* 新窗口 */
            ImageData imgData;
            imgData.name = winName;
            imgData.mat = tempImg;
            imgData.isUpdated = true;
            std::unique_lock<std::mutex> lock(s_mutex);
            s_imgVec[m_id].emplace_back(imgData);
            // lock.unlock();
        } else {
            /* 旧窗口 */
            std::unique_lock<std::mutex> lock(s_mutex);
            s_imgVec[m_id][i].mat = tempImg;
            s_imgVec[m_id][i].isUpdated = true;
            // lock.unlock();
        }
    }

    /**
     * 计算两次相同name传入之间的耗时, 有多个同名调用时, 取最远的两次
     * @param name 命名
     */
    void clock(const cv::String &name) {
        if (!s_isClockPrintEnable)
            return;
        for (auto &_clock : m_clocks) {
            if (std::strcmp(_clock.first.c_str(), name.c_str()) == 0) {
                _clock.second = cv::getTickCount() - _clock.second;
                return;
            }
        }
        std::pair<cv::String, int64_t> _clock;
        _clock.first = name;
        _clock.second = cv::getTickCount();
        m_clocks.emplace_back(_clock);
    }

    int get_and_clearCurrentKey() {
        int k = s_currentKey.load();
        s_currentKey.exchange(-1);
        return k;
    }

    /**
     * 当前窗口是否暂停刷新
     * @return
     */
    bool isPause() { return s_isPause.load(); }

    /**
     * 通知服务端刷新图像显示
     */
    void show() {
        using namespace std::chrono_literals;

        while (s_isPause.load() && s_mode != 0)
            std::this_thread::sleep_for(1us);
        // m_clockPrint();
        if (s_mode == 0)
            return;
        std::unique_lock<std::mutex> lock(s_mutex, std::try_to_lock);
        if(lock.owns_lock()){
            cv::swap(s_frameVec[m_id].second, m_frame);
            s_currentCallID.exchange(m_id);
            m_isWakeUp.exchange(true);
            s_conVar.notify_one();        
            // lock.unlock();
        }
    }
};

/**
 * ImageShowServer 主线程图像显示服务类
 * 基本线程安全
 */
class ImageShowServer : ImageShowBase {
  private:
    std::atomic_bool m_isWillExit;

    std::vector<ImageShowClient> m_clients;

    bool m_layoutRestFlag = true;
    DebugSocket ds;

    struct WinProps {
        cv::Point2i offset = cv::Point2i(0, 0);
        cv::Point2i startOffset = cv::Point2i(0, 0);
        cv::Size winSize = cv::Size(0, 0);
        double scale = 1.0;
        cv::Size boundarySize;  // 单侧边框宽度
        cv::Size winBorderSize;
        cv::Point2i currentLocation = cv::Point2i(0, 0);

        void setWinSize(int w, int h) {
            this->winSize.height =
                this->winSize.height > int(h * this->scale) ? this->winSize.height : int(h * this->scale);
            this->winSize.width =
                this->winSize.width > int(w * this->scale) ? this->winSize.width : int(w * this->scale);
        }

        void updateCurrentLocation() {
            this->currentLocation.x += this->winSize.width + this->winBorderSize.width;
            if (this->currentLocation.x > this->boundarySize.width) {
                this->currentLocation.x = this->offset.x + this->winBorderSize.width;
                this->currentLocation.y += this->winSize.height + this->winBorderSize.height;
                this->offset.y += this->winSize.height;
                winSize = cv::Size(0, 0);
            }
        };

        void restore() {
            this->winSize = cv::Size(0, 0);
            this->currentLocation = this->offset = this->startOffset;
        }
    } m_winProps;

    /**
     * 集中处理 waitKey(1) 捕获的用户按键输入
     * @param key waitKey(1)返回值
     * @return true = 用户需要退出, false = 其他
     */
    bool m_keyEvent(int key) {
        s_currentKey.exchange(key);
        switch (key) {
            case 'q':
                return true;
            case ' ':
                s_isPause.exchange(true);
                while (cv::waitKey(0) != ' ')
                    ;
                s_isPause.exchange(false);
                break;
            case 'r':
                m_layoutRestFlag = true;
                break;
            default:
                break;
        }
        return false;
    }

    /**
     * 生成调整过位置和大小的窗口
     * @param name 窗口名
     * @param w 图像原始宽
     * @param h 图像原始高
     */
    void m_createModifiedWindow(const cv::String &name, int w, int h) {
        cv::namedWindow(name, cv::WINDOW_GUI_NORMAL);
        m_winProps.updateCurrentLocation();
        cv::moveWindow(name, m_winProps.currentLocation.x, m_winProps.currentLocation.y);
        cv::resizeWindow(name, int(w * m_winProps.scale), int(h * m_winProps.scale));
        m_winProps.setWinSize(w, h);
    }

  public:
    /**
     * 构造函数
     * @param clientNum     客户端数量,
     * @param scale         窗口缩放大小(0.5 = 缩小一半)
     * @param winBorderSize 窗口边框宽度（双侧和, px）
     * @param boundarySize  屏幕像素边界大小(px)
     * @param offset        窗口组起始偏移量(px)
     */
    explicit ImageShowServer(int clientNum, double scale = 1.0, const cv::Size &winBorderSize = cv::Size(10, 80), const cv::Size &boundarySize = cv::Size(1920, 1080), const cv::Point2i &offset = cv::Point2i(120, 80))
        : m_isWillExit(false) {
        printf("\n"
               "| [isServer] help ¯¯¯¯¯¯¯¯¯¯¯¯¯¯|\n"
               "| press `space` to pause        |\n"
               "| press `q` to quit             |\n"
               "| press `r` to reset layout     |\n"
               "|___ [isServer] start main loop |\n"
               "\n");
        s_frameVec.resize(clientNum);
        s_imgVec.resize(clientNum);
        m_clients.resize(clientNum);

        m_winProps.scale = scale;
        m_winProps.boundarySize = boundarySize;
        m_winProps.winBorderSize = winBorderSize;
        m_winProps.offset = m_winProps.startOffset = m_winProps.currentLocation = offset;
    }
    /**
     * 设置字体
     */
    void setFontSize(double size) { s_fontSize.exchange(size); }
    /**
     * 设置显示模式
     */
    void setMode(int mode) { s_mode.exchange(mode); }
    /**
     * 设置是否输出clock的计时
     */
    void enableClockPrint(bool enable = true) { s_isClockPrintEnable.exchange(enable); }
    /**
     * 设置是否输出cpu平均耗时
     */
    void enableAverageCostPrint(bool enable = true) { s_isAverageCostPrint.exchange(enable); }

    /**
     * 依次生成客户端
     * @param id 客户端编号, 必须小于构造时设置的客户端数量
     * @return 客户端
     */
    ImageShowClient &getClient(int id) {
        m_clients[id].setClientId(id);
        return m_clients[id];
    }

    /**
     * 是否将要退出图像显示
     * @return true/false
     */
    bool isWillExit() { return m_isWillExit.load(); }

    /**
     * 获得最近一次的用户输入按键值并清空
     * @return int 按键值
     */
    int get_and_clearCurrentKey() {
        int k = s_currentKey.load();
        s_currentKey.exchange(-1);
        return k;
    }

    /**
     * 当前窗口是否暂停刷新
     * @return true/false
     */
    bool isPause() { return s_isPause.load(); }
    /**
     * 进行图像显示
     */
    void mainloop() {
        using namespace std::chrono_literals;

        if (s_mode == 0) {
            PRINT_INFO("[is Server] mode = 0 quit\n");
            return;
        }
        printf("[is Server] enter mainloop ...\n");
        while (true) {
            std::unique_lock<std::mutex> lock(s_mutex);
            if (s_conVar.wait_for(lock, 12s, [&](){return m_isWakeUp.load() || m_isWillExit.load();})){
                    int id = s_currentCallID.load();
                    if (m_layoutRestFlag) {
                        m_createModifiedWindow("main", s_frameVec[s_currentCallID].second.cols,
                            s_frameVec[s_currentCallID].second.rows);
                        cv::setMouseCallback(
                            "main", [](int event, int x, int y, int flags, void *ustc) {
                                cv::Mat frame = *(cv::Mat *)ustc;
                                cv::Point p(x, y);
                                /* 按下左键 */
                                if (event == cv::EVENT_LBUTTONDOWN) {
                                    PRINT_WARN("x: %d, y: %d, b: %d, g: %d, r: %d \n", x, y, frame.at<cv::Vec3b>(p)[0],
                                        frame.at<cv::Vec3b>(p)[1], frame.at<cv::Vec3b>(p)[2]);
                                }
                            },
                            &s_frameVec[s_currentCallID].second);
                    }
                    /* 显示主图 */
                    cv::imshow("main", s_frameVec[id].second);
                    ds.sendFrame(s_frameVec[id].second);
                    for (auto &_img : s_imgVec[id]) {
                        if (_img.isUpdated) {
                            _img.isUpdated = false;
                            if (m_layoutRestFlag)
                                m_createModifiedWindow(_img.name, _img.mat.cols, _img.mat.rows);
                            cv::imshow(_img.name, _img.mat);
                        } else {
                            cv::line(_img.mat, cv::Point(0, 0), cv::Point(_img.mat.cols, _img.mat.rows),
                                cv::Scalar(0, 0, 255));
                            cv::line(_img.mat, cv::Point(_img.mat.cols, 0), cv::Point(0, _img.mat.rows),
                                cv::Scalar(0, 0, 255));
                            cv::imshow(_img.name, _img.mat);
                        }
                    }
                    lock.unlock();
                    m_isWakeUp.exchange(false);
                }
            else{
                lock.unlock();
                m_isWakeUp.exchange(false);
                PRINT_ERROR("[isServer] mainloop timeout\n");
                break;
            }
            if (m_layoutRestFlag) {
                m_winProps.restore();
                m_layoutRestFlag = false;
            }
            if (m_keyEvent(cv::waitKey(1)))
                break;
        }
        cv::destroyAllWindows();
        m_isWillExit.exchange(true);
        PRINT_INFO("[is Server] quit\n");
    }
};