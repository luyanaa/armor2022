#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <vector>

#include "ArmorFinder.hpp"
#include "base.hpp"
#include "capture/capture.hpp"
#include "communicator.hpp"
#include "imageshow.hpp"
#include "kalman.hpp"
#include "pid.hpp"
#include "sort/sort.h"
#include "target.hpp"
#include "TfClassifier.hpp"

const unsigned int max_age = 10;
const unsigned int min_hits = 1;
const double iou_threshold = 0.1;
const int height_threshold = 0;    //前后两帧装甲板中心高度差阈值
const int weight_threshold = 0;    //前后两帧装甲板中心宽度差阈值
const int drop_threshold = 0;      //掉帧系数阈值
const int continue_threshold = 0;  //持续识别系数阈值
const int anti_top_threshold = 0;  //陀螺系数阈值

/**
 * 自瞄基类, 多线程共享变量用
 */
class AttackBase {
  protected:
    static std::mutex s_mutex;                      // 互斥锁
    static std::atomic<int64_t> s_latestTimeStamp;  // 已经发送的帧编号
    static std::deque<Target> s_historyTargets;     // 打击历史, 最新的在头部, [0, 1, 2, 3, ....]
    static ArmorFinder s_armorFinder;
    static TfClassifier s_tfClassifier;

    Target target_box, last_box;  // 目标装甲板
    int drop_frame = 0;           //掉帧系数
    int anti_top = 0;             //陀螺系数
    int continue_identify = 0;    //持续识别系数

    static Kalman kalman;                              // 卡尔曼滤波
    static std::unique_ptr<sort::SORT> s_sortTracker;  // DeepSORT 跟踪
    static size_t s_trackId;                           // DeepSORT 跟踪对象Id
};

/**
 * 自瞄主类
 */
class Attack : AttackBase {
  private:
    Communicator &m_communicator;
    ImageShowClient &m_is;
    cv::Mat m_bgr;      // ROI图
    cv::Mat m_bgr_raw;  // 原图
    // 目标
    std::vector<Target> m_preTargets;  // 预检测目标
    std::vector<Target> m_targets;     // 本次有效目标集合
    // 开小图
    cv::Point2i m_startPt;   // ROI左上角点坐标
    bool m_isEnablePredict;  // 是否开预测

    int64_t m_currentTimeStamp = 0;  // 当前时间戳
    PID &m_pid;                      // PID

  public:
    explicit Attack(Communicator &communicator, PID &pid, ImageShowClient &isClient);
    void setMode(bool colorMode) { s_armorFinder.colorMode(colorMode); }

  private:
    static float distance(const sort::Track &track, const Target &target);
    
    emSendStatusA m_match();

  public:
    void enablePredict(bool enable = true);

    static void getBoundingRect(Target &tar, cv::Rect &rect, cv::Size &size, bool extendFlag = false);

    bool is_antitop();

    bool run(cv::Mat &src, int64_t timeStamp, float gYaw, float gPitch);
};
