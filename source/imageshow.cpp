#include "imageshow.hpp"

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
cv::Scalar &ImageShowClient::m_getCurrentColor() {
    m_currentColorIndex = m_currentColorIndex > 10 ? 0 : m_currentColorIndex;
    return m_colorLibrary[m_currentColorIndex++];
}

cv::Point2d &ImageShowClient::m_getCurrentMarginTextLoc(bool isLeftMargin, double txtXPos) {
    if (isLeftMargin) {
        m_currentLeftMarginLoc.y += s_fontSize * 16.0 + 4.0;
        return m_currentLeftMarginLoc;
    } else {
        m_currentRightMarginLoc.y += s_fontSize * 16.0 + 4.0;
        m_currentRightMarginLoc.x = txtXPos;
        return m_currentRightMarginLoc;
    }
}

void ImageShowClient::m_putMarginText(const cv::String &txt, const cv::Scalar &color, int thickness, bool isLeftMargin) {
    double txtPos = 0;
    if (!isLeftMargin)
        txtPos = m_frame.cols - s_fontSize * 8.5 * txt.length() - s_fontSize * 16 - 5;
    cv::putText(m_frame, txt, m_getCurrentMarginTextLoc(isLeftMargin, txtPos), cv::FONT_HERSHEY_PLAIN,
        s_fontSize, color, thickness);
}


void ImageShowClient::update(const cv::Mat &frame, int frameTick) {
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


void ImageShowClient::addText(const cv::String &txt) {
    if (s_mode == 0)
        return;
    int thickness = 1;
    m_putMarginText(txt, cv::Scalar(70, 123, 255), thickness, false);
}


void ImageShowClient::addRotatedRects(const cv::String &eventName, const std::vector<cv::RotatedRect> &rRects) {
    if (s_mode == 0 || s_mode == 1)
        return;
    cv::Scalar currentColor = m_getCurrentColor();
    int thickness = 4;
    for (const auto &rRect : rRects) {
        cv::Point2f pts[4];
        rRect.points(pts);
        for (int i = 0; i < 4; i++)
            cv::line(m_frame, pts[i], pts[(i + 1) % 4], currentColor, thickness);
    }
    m_putMarginText(eventName + cv::format(": %d", int(rRects.size())), currentColor, thickness);
}


void ImageShowClient::addRect(const cv::String &eventName, const cv::Rect &rect) {
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

void ImageShowClient::addEvent(const cv::String &eventName, const cv::Rect &rect) {
    addRect(eventName, rect);
}

void ImageShowClient::addCircle(const cv::String &eventName, const cv::Point &p, double radius, int thickness) {
    if (s_mode == 0 || s_mode == 1)
        return;
    cv::Scalar currentColor = m_getCurrentColor();
    int _thickness = 1;
    cv::circle(m_frame, p, radius, currentColor, thickness);
    m_putMarginText(eventName, currentColor, _thickness);
}

void ImageShowClient::addEvent(const cv::String &eventName, const cv::Point &p) {
    addCircle(eventName, p);
}


void ImageShowClient::addContours(const cv::String &eventName, const std::vector<std::vector<cv::Point2i>> &contours, const cv::Point &offset) {
    if (s_mode == 0 || s_mode == 1)
        return;
    cv::Scalar currentColor = m_getCurrentColor();
    int thickness = 1;
    cv::drawContours(m_frame, contours, -1, currentColor, thickness, 8, cv::noArray(), INT_MAX, offset);
    m_putMarginText(eventName + cv::format(": %d", int(contours.size())), currentColor, thickness);
}


void ImageShowClient::addEvent(const cv::String &eventName, const std::vector<std::vector<cv::Point2i>> &contours, const cv::Point &offset) {
    addContours(eventName, contours, offset);
}


void ImageShowClient::addLights(const cv::String &eventName, const std::vector<Light> &lights, const cv::Point &offset) {
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


void ImageShowClient::addEvent(const cv::String &eventName, const std::vector<Light> &lights, const cv::Point &offset) {
    addLights(eventName, lights, offset);
}


void ImageShowClient::addTargets(const cv::String &eventName, const std::vector<Target> &targets) {
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


void ImageShowClient::addTracks(const std::vector<sort::Track> &tracks) {
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


void ImageShowClient::addEvent(const cv::String &eventName, const std::vector<Target> &targets) {
    addTargets(eventName, targets);
}


void ImageShowClient::addEvent(const cv::String &eventName, const std::vector<cv::Point2f> &region) {
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


void ImageShowClient::addEvent(const cv::String &eventName, const cv::RotatedRect &rRect) {
    if (s_mode == 0)
        return;
    cv::Scalar currentColor = m_getCurrentColor();
    int thickness = 1;
    cv::Point2f pts[4];
    rRect.points(pts);
    for (int i = 0; i < 4; i++)
        line(m_frame, pts[i], pts[(i + 1) % 4], currentColor, thickness);
}


void ImageShowClient::addClassifiedTargets(const cv::String &eventName, const std::vector<Target> &targets) {
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


void ImageShowClient::addFinalTargets(const cv::String &eventName, const Target &target) {
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


void ImageShowClient::addImg(const cv::String &winName, const cv::Mat &img, bool want) {
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


void ImageShowClient::clock(const cv::String &name) {
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


void ImageShowClient::show() {
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


void ImageShowServer::m_createModifiedWindow(const cv::String &name, int w, int h) {
    cv::namedWindow(name, cv::WINDOW_GUI_NORMAL);
    m_winProps.updateCurrentLocation();
    cv::moveWindow(name, m_winProps.currentLocation.x, m_winProps.currentLocation.y);
    cv::resizeWindow(name, int(w * m_winProps.scale), int(h * m_winProps.scale));
    m_winProps.setWinSize(w, h);
}


void ImageShowServer::mainloop() {
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
