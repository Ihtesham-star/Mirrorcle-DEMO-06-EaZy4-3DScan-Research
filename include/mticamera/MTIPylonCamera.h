#ifndef MTI_PYLON_CAMERA
#define MTI_PYLON_CAMERA

#include "MTICamera.h"

#include <pylon/PylonIncludes.h>

class DLLEXPORT MTIPylonCamera : public MTICamera
{
public:
	MTIPylonCamera();
	~MTIPylonCamera();

	bool IsColorCamera() {return m_isColorCamera;};
	// Base class overrides
	int InitializeCamera(int rotation = 0, int camera_index = 0);
	int SetCallbackFunction(MTICameraCallback callback);
	void ShutdownCamera();
	void StartCamera();
	void StopCamera();
	cv::Mat GetFrame(); // For manually polling camera for new frames
	int GetActiveFPS();
	int SetHardwareTrigger(bool enabled);
	int SetExposureTime(double time); // in microseconds. Send 0 to select automatic shutter

private:
	Pylon::CInstantCamera m_camera;
	Pylon::CImageFormatConverter m_formatConverter;
	Pylon::CPylonImage m_pylonImage;
	bool m_hardwareTriggerOn;
	bool m_isColorCamera;
	// FPS counters
	int m_fps;
	int m_frameCounter;
	int m_tick;
	time_t m_timeBegin;
	void UpdatePixelDistance();
};

#endif