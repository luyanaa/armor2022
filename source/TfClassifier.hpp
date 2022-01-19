#pragma once

#include <imageshow.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


#include "net.h"


class TfClassifier {
  private:
    ncnn::Net model;
    static int s_cropNameCounter;  // 存文件时候使用
    // Normalization parameters
    const float mean_vals[1]={0};
    const float normal_vals[1]={1.0/255.0};

  public:
    TfClassifier();

    ~TfClassifier() { }
    
    int m_classify_single_image(const cv::Mat &gray);

    void m_classify_single_tensor(const cv::Mat &m_bgr_raw, const std::vector<Target> &preTargets, std::vector<Target> &targets, ImageShowClient &is);
};


