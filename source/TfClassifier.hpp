#pragma once

#include <imageshow.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


#include "debug.h"
#include "net.h"


class TfClassifier {
  private:
    ncnn::Net model;
    static int s_cropNameCounter;  // 存文件时候使用
    // Normalization parameters
    const float mean_vals[1]={0};
    const float normal_vals[1]={1.0/255.0};

  public:
    TfClassifier() {
    	// Read the model
    	model.load_param("lenet-opt-shape-opt-fp16.param");
    	model.load_model("lenet-opt-shape-opt-fp16.bin");
    	model.opt.use_vulkan_compute = false;
    	// Build the pipeline.

    }

    ~TfClassifier() {

    }
    
    int m_classify_single_image(const cv::Mat &gray){
        // clock_t start = clock();;
        ncnn::Extractor ex=model.create_extractor();    
    	ex.set_num_threads(4);
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(gray.data, ncnn::Mat::PIXEL_BGR2GRAY, gray.cols, gray.rows, 32, 32), out;
        in.substract_mean_normalize(mean_vals, normal_vals);
        ex.input("input", in);    
        ex.extract("dense_2", out);
        float f = out[0];
        int output = 0;
        for (int j = 0; j < out.w; j++)
            if(f<out[j]) output = j;
        printf("output: %d\n", output);
        PRINT_INFO("[NCNN] Output %f, %f, %f, %f, %f, %f\n", out[0], out[1], out[2], out[3], out[4], out[5]);
        // PRINT_INFO("[NCNN] %f ms\n", (clock()-start)/(double)CLOCKS_PER_SEC*1000);
        return output;
    }
    /**
     * Backward Compatibility     
     * @param isSave 是否保存样本图片
     * @change targets 经过分类器的装甲板
     */
    void m_classify_single_tensor(const cv::Mat &m_bgr_raw, const std::vector<Target> &preTargets, std::vector<Target> &targets, ImageShowClient &is){
         if (preTargets.empty())
            return;   
         for (auto &_tar : preTargets) {
             auto pixelPts2f_Ex_array = _tar.pixelPts2f_Ex.toArray();
             cv::Rect tmp = cv::boundingRect(pixelPts2f_Ex_array);
             cv::Mat _crop = m_bgr_raw(tmp).clone();
             /* 将图片变成目标大小 */
             cv::Mat transMat = cv::getPerspectiveTransform(pixelPts2f_Ex_array, pixelPts2f_Ex_array);
             // cv::Mat _crop;
             /* 投影变换 */
             cv::warpPerspective(_crop, _crop, transMat, cv::Size(_crop.size()));
             /* 转灰度图 */
//             cv::cvtColor(_crop, _crop, cv::COLOR_BGR2GRAY);            
//             cv::medianBlur(_crop, _crop, 3);
             if(m_classify_single_image(_crop)!=0&&m_classify_single_image(_crop)!=7)             
                 targets.emplace_back(_tar);

            is.addImg("_crop",_crop);
         }
         
         is.addClassifiedTargets("After Classify", targets);
         std::cout << "Targets: " << targets.size() << std::endl;
    }

};


