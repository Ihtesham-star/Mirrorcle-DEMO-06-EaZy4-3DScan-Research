// ThreadedCamera.h
//
// Helper class for reading camera frames on a separate thread to free up
// processing on main thread

#ifndef THREADED_CAMERA_H
#define THREADED_CAMERA_H

#include <opencv2/opencv.hpp>

#ifdef MTI_UNIX
#include <future>
#endif

class ThreadedCamera {
    public:
        ThreadedCamera();
        ThreadedCamera(cv::VideoCapture capture);
        ~ThreadedCamera();

        void SetRotation(int degrees);  // Currently only 0 and 180 supported
        void StartCamera();
        void StopCamera();
        bool IsRunning();
        cv::Mat GetFrame();
        int GetFPS();

        unsigned int ContinuousRead();

    private:
        cv::VideoCapture capture_;
        bool stopped_;
        cv::Mat frame_;
        int rotation_;
        int fps_;
#ifdef MTI_UNIX
        std::future<void> future_;
#endif
};
     
#endif
