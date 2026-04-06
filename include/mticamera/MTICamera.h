//////////////////////////////////////////////////////////////////////
// MTICamera
// Version: 11.1.1.0
//////////////////////////////////////////////////////////////////////

#ifndef MTI_CAMERA
#define MTI_CAMERA

#define _USE_MATH_DEFINES
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <cmath>

#include <vector>

#include <opencv2/opencv.hpp>

#include "MTIDefinitions.h"
#include "ThreadedCamera.h"

class MTICamera;
// User can set a callback function for cameras that support it. That way, 
// we get called for each frame from the camera, instead of polling for frames
typedef void (*MTICameraCallback)(cv::Mat frame, MTICamera* cvManager);

class DLLEXPORT MTICamera
{
public:
	MTICamera();
    ~MTICamera();

    bool SaveCameraSettings(const std::string &filename);
    bool LoadCameraSettings(const std::string &filename);
	bool SaveDeviceSettings(const std::string &filename);
	bool LoadDeviceSettings(const std::string &filename);

    // Virtual methods
    virtual cv::Mat GetFrame();

    class DLLEXPORT ColorThreshold {
    public:
        int ColorCode;
        std::vector<int> ThresholdMin;
        std::vector<int> ThresholdMax;
        ColorThreshold();
        ColorThreshold(const std::vector<int> &thresholdMin, const std::vector<int> &thresholdMax);
        void ResetThreshold();
    };
    ColorThreshold *DeviceThreshold;
	
    /* General CV Methods */
    cv::Mat CropToCFOR(const cv::Mat &frame);
    cv::Point CFORPixelToCamPixel(const cv::Point &point);
    cv::Mat ThresholdFrame(const cv::Mat &frame, const ColorThreshold *colorThreshold);
    bool TrackSpot(const cv::Mat &frame, cv::Point &spot, const ColorThreshold *spotThreshold);
	bool TrackSpot(const cv::Mat &frame, cv::Point2d &spot, const ColorThreshold *spotThreshold);

    /* Lookup Table Methods */
    bool IsILUTCalibrated();

    // Convert to/from Camera Pixel and "Calibrated Field of Regard" Angle
    // Used for driving laser to target pixel
    cv::Point2d CamPixelToCFORAngle(const cv::Point2d &pixel);
    cv::Point CFORAngleToCamPixel(const cv::Point2d &angle);

    // Convert to/from Camera Pixel and Angle
    cv::Point2d CamPixelToCamAngle(const cv::Point2d &pixel);
    cv::Point CamAngleToCamPixel(const cv::Point2d &angle);

	void SetDeviceVbias(float voltage) { m_ILUTVbias = voltage; }
	void SetDeviceVdifferenceMax(float voltage) { m_ILUTVdifferenceMax = voltage; }
	void SetDeviceHardwareFiliter(float filter) { m_ILUTHardwareFilterBw = filter; }
	float GetDeviceVbias() { return m_ILUTVbias; }
	float GetDeviceVdifferenceMax() { return m_ILUTVdifferenceMax; }
	float GetDeviceHardwareFilterBw() { return m_ILUTHardwareFilterBw; }
	bool BilinearInterpolateILUTForAngle(const cv::Point2d &angle, cv::Point2d &laser);
	void SetILUTData(int row, int cols, const std::vector<cv::Point2d> &values);
	cv::Point2d GetILUTMaxAngles();
	int GetILUTRows() { return m_ILUTRows; }
	int GetILUTCols() { return m_ILUTCols; }
	void SetILUTMaxAngles(const cv::Point2d &maxAngles);
	void SetFORCenter(const cv::Point &center);
	void SetCameraAngleCorrectionX(float angle) { m_CameraAngleCorrectionX = angle; }
	float GetCameraAngleCorrectionX() { return m_CameraAngleCorrectionX; }
	void SetBaseDistance(float baseDistance) { m_CameraBaseDistance = baseDistance; }
	float GetBaseDistance() { return m_CameraBaseDistance; }
	void SetPlaneDistance(float planeDistance) { m_CameraPlaneDistance = planeDistance; }
	float GetPlaneDistance() { return m_CameraPlaneDistance; }

    /* Camera Methods */
	virtual int InitializeCamera(int rotation = 0, int camera_index = 0);
	virtual int SetCallbackFunction(MTICameraCallback callback);
	virtual void DisableCallbackFunction();
	virtual void ShutdownCamera();
	virtual void StartCamera();
	virtual void StopCamera();
	virtual int SetHardwareTrigger(bool enabled); // Hardware trigger unsupported in base class for now
	virtual int SetExposureTime(double time); // Exposure time in microseconds

	int GetCameraWidth();
	int GetCameraHeight();
	void SetCameraHFOV(float cameraHFOV) { m_CameraHFOV = cameraHFOV; }
	float GetCameraHFOV() { return m_CameraHFOV; }
	void SetCameraVFOV(float cameraVFOV) { m_CameraVFOV = cameraVFOV; }
	float GetCameraVFOV() { return m_CameraVFOV; }
	void SetCameraResolution(int widthPx, int heightPx);
	void SetCameraFPS(int fps);
	int GetCameraFPS();
	virtual int GetActiveFPS();
	static cv::Point3i GrayToRGBJet(float brightness, float Maxbrightness);

protected:
	float m_ILUTVbias, m_ILUTVdifferenceMax, m_ILUTHardwareFilterBw;
	int m_CameraWidth, m_CameraHeight;
    float m_CameraHFOV, m_CameraVFOV;
    float m_CamPixelDistance;
    int m_CameraFPS;
	int m_CameraRotation;

    cv::Point m_FORCenter;

private:
    void SetValue(const std::string &key, const std::string &value);
    std::string VecToString(const std::vector<int> &vec);
    std::vector<int> StringToIntVec(const std::string &str);
    std::vector<float> StringToFloatVec(const std::string &str);

    /* Camera variables */
    cv::VideoCapture m_VideoCapture;
    std::unique_ptr<ThreadedCamera> m_Camera;
    void SetCaptureProperties();        // Sets the VideoCapture properties from class variables

    /* iLUT Variables */
    // The 'center' of the Calibrated Field of Regard should be where the
    // laser spot is at 0,0 in device coordinates
    bool m_IsILUTCalibrated;

    std::vector<cv::Point2d> m_ILUTValues;
    int m_ILUTRows, m_ILUTCols;
    float m_CFORMaxAngX, m_CFORMaxAngY;
    float m_CameraAngleCorrectionX;
	float m_CameraBaseDistance;
	float m_CameraPlaneDistance;
    // The frame of the Calibrated Field of Regard in Camera coordinates
    cv::Rect m_CFORRect;
    void CalculateCFOR();
	void UpdatePixelDistance();
};

#endif
