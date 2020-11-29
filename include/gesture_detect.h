#ifndef _GESTURE_DETECT_H_
#define _GESTURE_DETECT_H_

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <cassert>
#include <vector>
#include "NvInfer.h"
#include "NvUffParser.h"
#include "NvUtils.h"
#include "common.h"
#include <opencv2/opencv.hpp>

#define MAX_WORKSPACE (1 << 30)
#define DEBUG_MODEL 0
static Logger gLogger;
#define RETURN_AND_LOG(ret, severity, message)                                   \
    do                                                                           \
    {                                                                            \
        std::string error_message = "gesture_model: " + std::string(message);    \
        gLogger.log(ILogger::Severity::k##severity, error_message.c_str());      \
        return (ret);                                                            \
    } while (0)

class GestureDetector
{
public:
    GestureDetector();
    ~GestureDetector();
    void init(char* modelPath = "gesture_model.engine");
    int detect(cv::Mat src);
    void release(void);
    void* safeCudaMalloc(size_t memSize);
    void* createImgCudaBuffer(cv::Mat src, int64_t eltCount, DataType dtype, int run);
    std::vector<std::pair<int64_t, DataType>> calculateBindingBufferSizes(const ICudaEngine& engine, int nbBindings, int batchSize);
    int printOutput(int64_t eltCount, DataType dtype, void* buffer);
    inline int64_t volume(const Dims& d)
    {
        int64_t v = 1;
        for (int64_t i = 0; i < d.nbDims; i++)
            v *= d.d[i];
        return v;
    }

    inline unsigned int elementSize(DataType t)
    {
        switch (t)
        {
        case DataType::kINT32:
            // Fallthrough, same as kFLOAT
        case DataType::kFLOAT: return 4;
        case DataType::kHALF: return 2;
        case DataType::kINT8: return 1;
        }
        assert(0);
        return 0;
    }

public:
    bool initFailFlag = false;

private:
    static const int INPUT_H = 300;
    static const int INPUT_W = 300;
    std::vector<char> trtModelStream_;
    ICudaEngine* engine;
    IExecutionContext* context;
    std::vector<std::pair<int64_t, DataType>> buffersSizes;
    std::vector<void*> buffers;
    int nbBindings = 0;
    int batchSize = 1;
    int bindingIdxInput = 0;
};

#endif
