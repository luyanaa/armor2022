//
//created by 刘雍熙 on 2020-02-01
//

#pragma once

#include <atomic>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <serial/serial.h>
#include <thread>

#include "crc_table.hpp"
#include "usbio/usbio.hpp"

//与电控的通信，需要与电控协商确定发的数据（共三个标识符）
//1:击打状态
typedef enum {
    SEND_STATUS_AUTO_PLACEHOLDER = 0x00,  // 占位符
    SEND_STATUS_AUTO_AIM_FORMER = 0x30,   // 打上一帧
    SEND_STATUS_AUTO_AIM = 0x31,          // 去瞄准
    SEND_STATUS_AUTO_SHOOT = 0x32,        // 去打
    SEND_STATUS_AUTO_NOT_FOUND = 0x33,    // 没找到
    SEND_STATUS_WM_AIM = 0x34             // 使用大风车
} emSendStatusA;
//2:大小符标识符
typedef enum {
    SEND_STATUS_WM_PLACEHOLDER = 0x00,  // 占位符
    SEND_STATUS_WM_FIND = 0x41,         // 大风车瞄准
    SEND_STATUS_WM_NO = 0x42            // 大风车没识别
} emSendStatusB;
//3：工作模式（自瞄/风车）
typedef enum {
    RM_AUTO_ATTACK = 0x11,
    RM_WINDMILL_SMALL_CLOCK = 0x21,
    RM_WINDMILL_SMALL_ANTIC = 0x22,
    RM_WINDMILL_LARGE_CLOCK = 0x23,
    RM_WINDMILL_LARGE_ANTIC = 0x24
} emWorkMode;

/**
 * 通讯基类
 */
class Communicator {
  protected:
#pragma pack(1)  //为整个包减少空字符使用pragma
    //头帧 时间戳 yaw pitch 标识符 校验 尾帧
    struct __FrameSt {
        uint8_t head = 0xf1;
        uint16_t timeStamp = 0;
        float yaw = 0.0;
        float pitch = 0.0;
        float speed = 0.0;
        unsigned char state; //
        unsigned short time;
        uint8_t extra[2] = {0, 0};  // additional imformation
        uint8_t crc8check = 0;
        uint8_t end = 0xf2;
    } m_frame;
#pragma pack()

    static uint8_t m_calcCRC8(const uint8_t *buff, size_t len);

    constexpr static size_t m_frameSize = sizeof(__FrameSt);

    bool m_checkFrame(const uint8_t *buff);

    //多线程用
    std::mutex m_mutex;

    std::atomic_bool m_isEnableReceiveGlobalAngle;
    std::atomic_int m_WorkMode;  // 当前工作模式: 摸鱼, 自瞄, 大风车
    std::deque<float> m_gYaws;
    std::deque<float> m_gPitches;
    std::deque<float> m_bulletSpeed;
    std::thread m_receiveThread;  // 接收线程
    std::atomic_bool m_letStop;   // 暂停接收线程

    std::atomic_bool m_isDisable;  // 是否使能

    std::atomic<int64_t> m_lastTick;
    std::atomic<int64_t> m_currentInterval;

  public:
    //构造函数
    explicit Communicator() : m_isEnableReceiveGlobalAngle(false), m_WorkMode(RM_AUTO_ATTACK),
                              m_letStop(false), m_isDisable(false), m_lastTick(cv::getTickCount()),
                              m_currentInterval(-1110) {
        m_gYaws.resize(100);
        m_gPitches.resize(100);
    }

    /**
     * 禁用, debug 用
     * @param disable
     */
    void disable(bool disable) {
        m_isDisable.exchange(disable);
    }

    /**
     * 发送给电控
     * @param rYaw
     * @param rPitch
     * @param extra0
     * @param extra1
     */
    virtual void send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1){};

    /**
     * 接收线程, 循环读取电控发来的数据
     * @return m_WorkMode 工作模式
     * @return m_gYaws    yaw角
     * @return m_gPitches pitch角
     */
    virtual void startReceiveService(){};

    emWorkMode getWorkMode();

    /**
     * 获得当前 wait_and_get 函数获得的图像时间间隔
     * @return 采集时间间隔us
     */
    int64_t getCurrentInterval() {
        return m_currentInterval.load();
    }

    void getGlobalAngle(float *gYaw, float *gPitch, uint8_t delay = 0);

    void getBulletSpeed(float *bulletSpeed, uint8_t delay = 0);

    void enableReceiveGlobalAngle(bool enable = true) {
        m_isEnableReceiveGlobalAngle.exchange(enable);
    }

    /**
     * 结束所有
     */
    void letStop() { m_letStop.exchange(true); }

    /**
     * 等待线程结束
     */
    void join() {
        if (m_receiveThread.joinable()) m_receiveThread.join();
    };

    ~Communicator() { join(); }
};

/**
 * 使用串口通信的子类
 */
class CommunicatorSerial : public Communicator {
  private:
    serial::Serial m_ser;

  public:
    explicit CommunicatorSerial() = default;

    void open(const cv::String &portName, uint32_t baudrate = 115200);

    void send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1) override;

    void startReceiveService() override;
};

/**
 * USB 通讯子类
 */
class CommunicatorUSB : public Communicator {
  private:
    usbio::spUSB *m_usb;

  public:
    explicit CommunicatorUSB() : m_usb(nullptr) {}

    /**
     * 打开设备, 带后台线程
     * @param vid 0x0477 16进制
     * @param pid 0x5620 16进制
     */
    void open(int vid, int pid) {
        m_usb = new usbio::spUSB(vid, pid);
    }

    void send(float rYaw, float rPitch,emSendStatusA extra0, emSendStatusB extra1) override;

    void startReceiveService() override;
};
