#include "communicator.hpp"
#include "debug.h"

//crc校验用
uint8_t Communicator::m_calcCRC8(const uint8_t *buff, size_t len) {
    uint8_t ucIndex, ucCRC8 = (uint8_t)CRC8_INIT;
    while (len--) {
        ucIndex = ucCRC8 ^ (*buff++);
        ucCRC8 = CRC8_Table[ucIndex];
    }
    return (ucCRC8);
}

//包校验
bool Communicator::m_checkFrame(const uint8_t *buff) {
    if (buff[0] == m_frame.head && buff[m_frameSize - 1] == m_frame.end) {
        return buff[m_frameSize - 2] == m_calcCRC8(buff, m_frameSize - 2);
    }
    return false;
}

/**
 * 获得当前工作模式: 摸鱼, 自瞄, 大风车
 * @return
 */
emWorkMode Communicator::getWorkMode() {
    switch (m_WorkMode.load()) {
        default:
        case RM_AUTO_ATTACK:
            return RM_AUTO_ATTACK;
        case RM_WINDMILL_SMALL_CLOCK:
            return RM_WINDMILL_SMALL_CLOCK;
        case RM_WINDMILL_SMALL_ANTIC:
            return RM_WINDMILL_SMALL_ANTIC;
        case RM_WINDMILL_LARGE_CLOCK:
            return RM_WINDMILL_LARGE_CLOCK;
        case RM_WINDMILL_LARGE_ANTIC:
            return RM_WINDMILL_LARGE_ANTIC;
    }
};

/**
 * 获得最新的云台全局欧拉角
 * @param gYaw 存放云台 yaw 角，单位：度
 * @param gPitch 存放云台 pitch 角，单位：度
 * @param delay 取第 delay 个值, [0-最新, 1-第二新, ...]
 */
void Communicator::getGlobalAngle(float *gYaw, float *gPitch, uint8_t delay) {
    if (m_isEnableReceiveGlobalAngle) {
    
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        *gYaw = m_gYaws[delay];
        *gPitch = m_gPitches[delay];
    } else {
        *gYaw = 1;
        *gPitch = 0;
    }
}

/**
 * 获得最新的子弹速度
* @param bulletSpeed 存放子弹速度
* @param delay 取第 delay 个值, [0-最新, 1-第二新, ...]
*/
void Communicator::getBulletSpeed(float *bulletSpeed, uint8_t delay) {
    if (m_isDisable) {
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        *bulletSpeed = m_bulletSpeed[delay];
    } else {
        *bulletSpeed = 0.0;
    }
}

/**
 * 打开设备, 带后台线程
 * @param portName
 * @param baudrate
 */
void CommunicatorSerial::open(const cv::String &portName, uint32_t baudrate) {
    using namespace std::chrono_literals;

    if (m_isDisable.load()) return;
    m_ser.setPort(portName);
    m_ser.setBaudrate(baudrate);

    /* 守护线程 */
    int openSerialCounter = 0;
    while (!m_ser.isOpen() && !m_letStop.load()) {
        PRINT_WARN("[serial] try open %d\n", openSerialCounter++);
        try {
            m_ser.open();
        } catch (serial::IOException &e) {
            PRINT_ERROR("[serial] error: %s\n", e.what());
        }
        /* 转移时间片 */
        std::this_thread::sleep_for(100ms);
        if (openSerialCounter > 10) break;
    }
    if (m_ser.isOpen())
        PRINT_INFO("[serial] open\n");
    else
        exit(-666);
}

/**
 * @brief 向电控发送控制信息
 * @param rYaw 
 * @param rPitch 
 * @param extra0 
 * @param extra1 
 */
void CommunicatorSerial::send(float rYaw, float rPitch, emSendStatusA extra0, emSendStatusB extra1) {
    if (m_isDisable.load()) return;
    if (!m_ser.isOpen()) return;
    /* 刷新结构体 */
    if (m_frame.timeStamp > 0xfffe) m_frame.timeStamp = 0;
    m_frame.timeStamp++;
    m_frame.yaw = rYaw;
    m_frame.pitch = rPitch;
    m_frame.extra[0] = extra0;
    m_frame.extra[1] = extra1;

    /* 间隔3ms以内不发送 */
    if (1000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency() > 3) {
        m_currentInterval.exchange(1000000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency());
        m_lastTick.exchange(cv::getTickCount());

        /* 组织字节 */
        uint8_t msg[m_frameSize];
        memcpy(msg, &m_frame, m_frameSize);
        msg[m_frameSize - 2] = m_calcCRC8(msg, m_frameSize - 2);

        /* 发送 */
        m_ser.flushOutput();
        m_ser.write(msg, m_frameSize);
    }
};

void CommunicatorSerial::startReceiveService() {
    using namespace std::chrono_literals;
    if (m_isDisable.load()) return;
    m_receiveThread = std::thread([&]() {
        while (!m_ser.isOpen() && !m_letStop.load()) {
            std::this_thread::sleep_for(200us);
        }
        std::vector<uint8_t> buffer;
        while (!m_letStop.load()) {
            size_t size_temp = m_ser.available();
            if (size_temp > 0) {
                /* 读值 */
                std::vector<uint8_t> buffer_tmp;
                size_t readed = m_ser.read(buffer_tmp, size_temp);
                PRINT_INFO("[serial] read %lu\n", readed);
                buffer.insert(buffer.end(), buffer_tmp.begin(), buffer_tmp.end());
                /* 校验 */
                while (buffer.size() >= m_frameSize) {
                    size_t i = 0;
                    for (i = 0; i < buffer.size() - m_frameSize + 1; ++i) {
                        const unsigned char *start = buffer.data() + i;
                        if (m_checkFrame(start)) {
                            /* 更新值 begin */
                            struct __FrameSt frame;
                            memcpy(&frame, start, m_frameSize);

                            /* 更新成员变量 */
                            //m_WorkMode.exchange(frame.extra[0]);

                            if (m_isEnableReceiveGlobalAngle) {
                                std::lock_guard<std::mutex> lock(m_mutex);
                                m_bulletSpeed.emplace_front(frame.speed);
                                m_gYaws.emplace_front(frame.yaw * 180.0 / M_PI);  // 转化成角度
                                m_gPitches.emplace_front(frame.pitch - 3);
                                if (m_bulletSpeed.size() > 10) m_bulletSpeed.pop_back();
                                if (m_gYaws.size() > 10) m_gYaws.pop_back();
                                if (m_gPitches.size() > 10) m_gPitches.pop_back();
                                PRINT_INFO("receive (yaw, pitch, bulletSpeed) = (%3.2f, %3.2f, %3.2f)\n", frame.yaw, frame.pitch, frame.speed);
                            }
                            /* 更新值 end */
                        } else {
                            // PRINT_ERROR("check package failed\n");
                        }
                    }
                    buffer.erase(buffer.begin(), buffer.begin() + i + m_frameSize - 1);
                }
            }
            /* 转移时间片 */
            std::this_thread::sleep_for(5us);
        }
    });
}

void CommunicatorUSB::send(float rYaw, float rPitch,emSendStatusA extra0, emSendStatusB extra1) {
    /* 刷新结构体 */
    if (m_frame.timeStamp > 0xfffe) m_frame.timeStamp = 0;
    m_frame.timeStamp++;
    m_frame.yaw = rYaw;
    m_frame.pitch = rPitch;
    m_frame.extra[0] = extra0;
    m_frame.extra[1] = extra1;

    /* 组织字节 */
    uint8_t msg[m_frameSize];
    memcpy(msg, &m_frame, m_frameSize);
    msg[m_frameSize - 2] = m_calcCRC8(msg, m_frameSize - 2);

    /* 发送 */
    int len;
    m_usb->write(msg, m_frameSize, len);
    m_currentInterval.exchange(1000000.0 * (cv::getTickCount() - m_lastTick.load()) / cv::getTickFrequency());
    m_lastTick.exchange(cv::getTickCount());
}

void CommunicatorUSB::startReceiveService() {
    using namespace std::chrono_literals;

    m_receiveThread = std::thread([&]() {
        uint8_t buffer[1024] = {0};
        while (!m_letStop.load()) {
            size_t size_temp = m_usb->available();
            if (size_temp > 0) {
                uint8_t buf[1024];
                uint8_t size = m_usb->read(buf, size_temp);

                memcpy(buffer, buf, size);

                printf("%d\n", size);
                while (size >= m_frameSize) {
                    int i = 0;
                    for (i = 0; i < size - m_frameSize + 1; ++i) {
                        if (m_checkFrame(&buffer[i])) {
                            /* 更新值 begin */
                            struct __FrameSt frame;
                            memcpy(&frame, &buffer[i], m_frameSize);

                            /* 更新成员变量 */
                            m_WorkMode.exchange(frame.extra[0]);

                            if (m_isEnableReceiveGlobalAngle) {
                                std::lock_guard<std::mutex> lock(m_mutex);
                                m_gYaws.emplace_front(frame.yaw);
                                m_gPitches.emplace_front(frame.pitch);
                                printf("receive (yaw, pitch) = (%3.2f, %3.2f)\n", frame.yaw, frame.pitch);
                            }
                            /* 更新值 end */
                        }
                    }
                    size = size - i - m_frameSize;
                    // 0, {1, 2, 3}, 4, 5, 6, 7
                    // size = 8, i = 1, m_frameSize = 3
                    // size = 8 - 1 - 3 = 4
                    memcpy(buffer, &buf[i + m_frameSize], size);
                }
                /* 转移时间片 */
                std::this_thread::sleep_for(5us);
            }
        }
    });
}
