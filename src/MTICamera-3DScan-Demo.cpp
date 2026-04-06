// 8mm mindvision lens on BASLER acA720-520um gives FoV 34.5 x 26.2
// 6mm mindvision lens on BASLER acA720-520um gives FoV 45 x 34.5

#include "MTIDefinitions.h"
#include "MTIPylonCamera.h"
#include "MTIDevice.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>
inline float triWave(float freq, float t, float phase)

{
    float period = 1.0f / freq;
    float phaseTime = phase / (2.0f * M_PI * freq);
    float adjustedTime = fmod(t + phaseTime, period);
    
    if (adjustedTime < 0) adjustedTime += period;
    float normalizedTime = adjustedTime / period;
    
    // Generate symmetric triangular wave: -1 to +1
    if (normalizedTime < 0.5f) {
        return 4.0f * normalizedTime - 1.0f;  // Rising edge
    } else {
        return 3.0f - 4.0f * normalizedTime;  // Falling edge
    }
}
// Pattern 1: Sinusoidal Lissajous (new)
inline float sinusoidalWave(float freq, float t, float phase = 0.0f)
{
    return sinf(2.0f * 3.14159265f * freq * t + phase);
}

// Pattern 3: Bidirectional Cartesian helper
inline float bidirectionalWave(float freq, float t, float phase, bool isSecondHalf)
{
    if (!isSecondHalf) {
        // First half uses the base frequency
        return sinf(2.0f * 3.14159265f * freq * t + phase);
    } else {
        // Second half doubles the frequency and swaps X/Y roles later
        return sinf(2.0f * 3.14159265f * (freq * 2.0f) * t + phase);
    }
}

// Pattern 4: Radial Lissajous helpers for X and Y
inline float radialLissajousX(float freq, float t, float phase = 0.0f)
{
    return sinf(2.0f * 3.14159265f * freq * t + phase)
         * sinf(2.0f * 3.14159265f * freq * t);
}

inline float radialLissajousY(float freq, float t, float phase = 0.0f)
{
    return sinf(2.0f * 3.14159265f * freq * t + phase)
         * cosf(2.0f * 3.14159265f * freq * t);
}


#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <fstream>
// ADD THESE INCLUDES:
#include <map>
#include <iomanip>
#include <sstream>

#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif

#define OPENCV_MOUSE_WHEEL_UP 3
#define OPENCV_MOUSE_WHEEL_DOWN 4
#define EVENT_MOUSEWHEEL 10  // Custom define for mouse wheel event
#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING  // Silence TR1 warnings  // Silence TR1 warnings
// Add these definitions at the top of the file
#define CV_EVENT_MOUSEWHEEL 10
#define CV_EVENT_FLAG_WHEELUP 0x0001
#define CV_EVENT_FLAG_WHEELDOWN 0x0002
#ifndef CV_EVENT_MOUSEWHEEL
    #define CV_EVENT_MOUSEWHEEL 10
#endif

#ifndef CV_EVENT_FLAG_WHEELUP
    #define CV_EVENT_FLAG_WHEELUP 0x0001
#endif

#ifndef CV_EVENT_FLAG_WHEELDOWN
    #define CV_EVENT_FLAG_WHEELDOWN 0x0002
#endif

// Add scan mode enumeration
enum ScanMode {
    SCAN_RASTER = 0,
    SCAN_SINUSOIDAL_LISSAJOUS = 1,
    SCAN_TRIANGULAR_LISSAJOUS = 2,
    SCAN_BIDIRECTIONAL_CARTESIAN = 3,
    SCAN_RADIAL_LISSAJOUS = 4
};



// MTI global objects
MTIDevice *mti;
MTIPylonCamera *camera;
MTIDataGenerator *datagen;



// 1. ADD THESE GLOBALS (after your existing globals):
static std::ofstream masterFile;
static bool masterFileOpen = false;


// OpenCV windows
constexpr auto windowCamera = "Camera";
constexpr auto windowControl = "Control";
constexpr auto windowPointCloud = "Point Cloud";

// Device parameters
constexpr char cameraSettingsFile[] = "mticamera.ini";
constexpr char deviceSettingsFile[] = "mtiscanmodule.ini";
constexpr unsigned int spsDefault = 20000;
constexpr unsigned int maxSamplesPerFrame = 100000U;
constexpr unsigned int maxNumLines = 200U;

int exposure_time = 25000;
constexpr int exposure_slider_max = 50000;

// Threshold slider for laser detection - REDUCED FROM 250 TO 150 FOR BETTER DETECTION
int BWThreshold = 150;
constexpr int threshold_slider_max = 255;

unsigned int npts = maxSamplesPerFrame;
unsigned int numScanLines = 100;
float lineDuration = 0.006f; // seconds
unsigned int sps = 10000;
float xData[maxSamplesPerFrame];
float yData[maxSamplesPerFrame];
unsigned char mData[maxSamplesPerFrame];
float MEMSAngles[maxNumLines];
float calMEMSAnglesNominal[maxNumLines];
float calMEMSAnglesActual[maxNumLines];
unsigned int calMEMSLines;
cv::Point2f scanAngleOffsets;
cv::Point2f scanAngleMagnitudes;
bool xAveraging = true;
bool mouseDown = false;
cv::Point pClick, pClickPrevious;
unsigned int windowH = 700, windowW = 800;

// add these globals up near the top of your file:
static float lissajousMEMSX[maxNumLines];
static float lissajousMEMSY[maxNumLines];






// Point cloud variables
cv::Mat pointCloud3D; // To store 3D points (x,y,z)
std::vector<cv::Point3f> points3D; // Vector of 3D points
float pointCloudScale = 1.0f; // Scale factor for visualization
// Add after the other global variables
static float rotationX = 0.25f;          // Changed from 0.0f
static float rotationY = 0.15f;          // Changed from 0.0f
static float cameraDistance = 300.0f;    // Changed from 1000.0f
static cv::Point2f lastMousePos;
static bool mousePressed = false;
static float viewAngleX = 0.25f;         // Changed from 0.0f
static float viewAngleY = 0.15f;         // Changed from 0.0f
static float zoomLevel = 0.2f;           // Changed from 1.0f
static bool autoRotate = false;
static float autoRotateSpeed = 0.01f;
static float mouseWheelPos = 0.0f;
// Add this global variable declaration with the other global variables
static float viewMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; // Identity matrix

std::vector<float> triggerTimes;  // ADD THIS at the top!
// Add these for camera pattern export
static bool rectifyPointCloud = false;
static cv::Mat rectificationMatrix = cv::Mat::eye(4, 4, CV_32F);


// Scanning amplitude and offset
float xAmplitude = 0.8f, yAmplitude = 0.7f, xOffset = 0.f, yOffset = 0.f;

// Debug visualization flag
bool showDebugInfo = true;

// Add scan mode variable
ScanMode currentScanMode = SCAN_RASTER;

const int   NP             = 100;         // ← change to 20, 40, 100, etc.
const float baseFreqX      = 250.0f;     // ← choose 200–800 Hz for crisp triangles
float       lissajousFreqX = baseFreqX;  
float       lissajousFreqY = baseFreqX * (NP - 1) / float(NP);  // Fixed calculation order  // e.g. 19/20×400 = 380 Hz
float       lissajousPhase = 0.0f;       // φ=0 (or φ=M_PI/2 for a 90° “rotation” if you need it)

static void OnTrackbarChange(int, void* userData)
{
    if (exposure_time < 100)
    {
        exposure_time = 100;
    }
    camera->SetExposureTime(exposure_time);
    cv::setTrackbarPos("Exposure Time", windowControl, exposure_time);
}

static void OnThresholdChange(int, void* userData)
{
    BWThreshold = std::max(10, std::min(BWThreshold, 255));
    cv::setTrackbarPos("Laser Threshold", windowControl, BWThreshold);
}

void ScaleAndOffsetArray(cv::Rect limits, unsigned int numPoints) {
    // Scale xData and yData arrays by scanAngleMagnitudes.x and scanAngleMagnitudes.y, respectively.
    // Offset xData and yData arrays post-scaling by scanAngleOffsets.x and scanAngleOffsets.y, respectively.
    // xData cannot exceed limits.x and limits.width/2
    // yData cannot exceed limits.y and limits.height/2
    // Useful for scaling normalized +-1 to desired angle
    for (int i = 0; i < (int)numPoints; i++) {
        xData[i] = (xData[i] * scanAngleMagnitudes.x) + scanAngleOffsets.x;
        // Saturation
        xData[i] = std::max((float)limits.x, std::min((float)limits.width / 2, xData[i]));
        yData[i] = (yData[i] * scanAngleMagnitudes.y) + scanAngleOffsets.y;
        yData[i] = std::max((float)limits.y, std::min((float)limits.height / 2, yData[i]));
    }
}

void CameraAngleToMEMSCommands(unsigned int numPoints) {
    // Translate xData and yData arrays (in camera space angles) to MEMS angle
    cv::Point2d cameraAngle;
    cv::Point2d laser;
    for (int i = 0; i < (int)numPoints; i++) {
        cameraAngle.x = xData[i];
        cameraAngle.y = yData[i];
        if (!camera->BilinearInterpolateILUTForAngle(cameraAngle, laser)) {
            // Handle interpolation failure
            printf("Warning: Interpolation failed for point %d\n", i);
            xData[i] = 0;
            yData[i] = 0;
        } else {
            xData[i] = laser.x;
            yData[i] = laser.y;
        }
    }
}

// Improved triangulation function with more robust error handling and logging
// Replace your existing CalculateTriangulationPoint function with this improved version
void CalculateTriangulationPoint(
    const cv::Point2f& cameraPixel,  // 2D pixel coords (x, y) in camera image
    float memsX_deg,                 // MEMS mirror X tilt at this frame, in degrees
    float memsY_deg,                 // MEMS mirror Y tilt at this frame, in degrees
    cv::Point3f& point3D             // (out) the computed 3D point in camera coords
) {
    // 1) Fetch the “base distance” between camera center and mirror pivot:
    float baseDistance = camera->GetBaseDistance();
    if (baseDistance <= 0.0f) {
        // Invalid base distance → no intersection
        point3D = cv::Point3f(0.0f, 0.0f, 0.0f);
        return;
    }

    // 2) Get camera intrinsics (image size + FOV):
    float camW = static_cast<float>(camera->GetCameraWidth());
    float camH = static_cast<float>(camera->GetCameraHeight());
    float hfov = camera->GetCameraHFOV() * MTI_DEGTORAD;  // horizontal FOV in radians
    float vfov = camera->GetCameraVFOV() * MTI_DEGTORAD;  // vertical   FOV in radians

    // 3) Compute the pixel’s angular offsets (in radians) from optical axis:
    float px = cameraPixel.x;
    float py = cameraPixel.y;
    //   θ_camX ∈ [–hfov/2, +hfov/2], with px=0→–, px=camW→+
    float theta_camX = ((px / camW) - 0.5f) * hfov;
    //   θ_camY ∈ [+vfov/2, –vfov/2], with py=0→+, py=camH→–
    float theta_camY = ((0.5f - py / camH)) * vfov;

    // 4) Convert MEMS angles to radians:
    float theta_mX = memsX_deg * MTI_DEGTORAD;
    float theta_mY = memsY_deg * MTI_DEGTORAD;

    // 5) Build the MEMS‐plane normal “n” by first tilting about X, then about Y:
    //    Start with (0,0,1).  Tilt by theta_mX around X-axis:
    //      ( 0, sin(theta_mX), cos(theta_mX) )
    //    Then tilt that vector around Y by theta_mY:
    //      n.x =  sin(theta_mY)*cos(theta_mX)
    //      n.y =  sin(theta_mX)
    //      n.z =  cos(theta_mX)*cos(theta_mY)
    cv::Point3f n;
    n.x = std::sin(theta_mY) * std::cos(theta_mX);
    n.y = std::sin(theta_mX);
    n.z = std::cos(theta_mX) * std::cos(theta_mY);

    // 6) The plane constant “d”: the tilted plane still passes through (0,0,baseDistance).
    //    Therefore d = n · (0,0, baseDistance) = baseDistance * n.z
    float d = baseDistance * n.z;

    // 7) Solve for Z by intersecting the camera ray with this tilted plane.
    //    Camera ray R_cam(Z) = (Z*tan(theta_camX),  Z*tan(theta_camY),  Z).
    //    We require: n · R_cam(Z) = d  → 
    //      Z * [n.x*tan(theta_camX) + n.y*tan(theta_camY) + n.z] = d
    float tan_camX = std::tan(theta_camX);
    float tan_camY = std::tan(theta_camY);
    float denom    = n.x * tan_camX 
                   + n.y * tan_camY 
                   + n.z;
    if (std::fabs(denom) < 1e-6f) {
        // Ray nearly parallel to the mirror plane → no valid intersection
        point3D = cv::Point3f(0.0f, 0.0f, 0.0f);
        return;
    }
    float Z = d / denom;
    Z = Z * 2.41f;

    // (Optional) If you need to clamp Z to a valid depth range, uncomment:
    // if (Z < minDepth || Z > maxDepth) {
    //     point3D = cv::Point3f(0, 0, 0);
    //     return;
    // }

    // 8) Finally compute X and Y:
    float X = Z * tan_camX;
    float Y = Z * tan_camY;

    point3D.x = X;
    point3D.y = Y;
    point3D.z = Z;
}

// Add this helper function above DisplayPointCloud()
cv::Scalar GetDepthColor(float normalizedDepth) {
    // Enhanced color map similar to reference image
    static const cv::Scalar colors[] = {
        cv::Scalar(50, 0, 200),     // Deep blue (far)
        cv::Scalar(0, 100, 255),    // Brighter blue
        cv::Scalar(0, 200, 255),    // Cyan
        cv::Scalar(0, 255, 100),    // Cyan-green
        cv::Scalar(100, 255, 0),    // Yellow-green
        cv::Scalar(255, 255, 0),    // Yellow
        cv::Scalar(255, 100, 0),    // Orange
        cv::Scalar(255, 0, 0)       // Red (near)
    };
    
    const float scaledDepth = normalizedDepth * 7.0f;  // 8 colors - 1
    const int idx1 = std::min(static_cast<int>(scaledDepth), 6);
    const int idx2 = std::min(idx1 + 1, 7);
    const float fraction = scaledDepth - static_cast<float>(idx1);
    
    // Smoother interpolation between colors
    return cv::Scalar(
        colors[idx1][0] * (1.0f - fraction) + colors[idx2][0] * fraction,
        colors[idx1][1] * (1.0f - fraction) + colors[idx2][1] * fraction,
        colors[idx1][2] * (1.0f - fraction) + colors[idx2][2] * fraction
    );
}



// SOLUTION 3: Fixed default view angles for camera-matching perspective
void SetCameraMatchingView() {
    // Set view angles to match camera perspective
    viewAngleX = 0.0f;        // No pitch rotation
    viewAngleY = 0.0f;        // No yaw rotation
    zoomLevel = 1.0f;         // Normal zoom
    
    if (showDebugInfo) {
        printf("Debug: Set camera-matching view\n");
    }
}

// SOLUTION 4: Add this function to toggle between views
void ToggleViewMode() {
    static bool cameraMatchingMode = false;
    
    if (cameraMatchingMode) {
        // Switch to 3D perspective view
        viewAngleX = 0.25f;
        viewAngleY = 0.15f;
        zoomLevel = 0.5f;
        printf("Switched to 3D perspective view\n");
    } else {
        // Switch to camera-matching view
        SetCameraMatchingView();
        printf("Switched to camera-matching view\n");
    }
    
    cameraMatchingMode = !cameraMatchingMode;
}




// Forward declarations
void RenderPointCloud(cv::Mat& image, const std::vector<cv::Point3f>& points, float minZ, float maxZ);
cv::Scalar GetDepthColor(float normalizedDepth);
void ExportCameraViewPattern(const std::vector<std::vector<cv::Point>>& allLines, const cv::Mat& currentFrame); // ADD THIS LINE
void Export3DVisualization(const std::vector<cv::Point3f>& points3D, const cv::Mat& currentFrame);
// ADD THESE NEW FORWARD DECLARATIONS:
struct ProfileAnalysis {
    cv::Mat depthMap;
    cv::Mat countMap;
    cv::Mat varianceMap;
    std::vector<float> allDepths;
    float minDepth, maxDepth, meanDepth, stdDepth;
    int totalPoints;
    std::string scanMode;
    int NP_value;
    float frequency;
};



cv::Mat CreateDetailedColorbar(float minZ, float maxZ, int height, int width);
ProfileAnalysis Analyze3DProfile(const std::vector<cv::Point3f>& points3D, const std::string& scanMode, int NP, float freq);
void ExportComprehensive3DProfile(const std::vector<cv::Point3f>& points3D, const std::string& scanMode, int NP, float frequency);
void RunNPComparisonStudy();


// Mouse event handler definition
static void OnMouseCallback(int event, int x, int y, int flags, void*) {
    static cv::Point2f lastPos;
    static bool dragging = false;

    // Debug output to verify events are being detected
    if (showDebugInfo) {
        if (event == cv::EVENT_LBUTTONDOWN) printf("Debug: Mouse down at %d, %d\n", x, y);
        if (event == cv::EVENT_LBUTTONUP) printf("Debug: Mouse up at %d, %d\n", x, y);
        if (event == CV_EVENT_MOUSEWHEEL) printf("Debug: Mouse wheel event, flags: %d\n", flags);
    }

    switch(event) {
        case cv::EVENT_LBUTTONDOWN:
            dragging = true;
            lastPos = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
            mousePressed = true;
            break;
            
        case cv::EVENT_LBUTTONUP:
            dragging = false;
            mousePressed = false;
            break;
            
        case cv::EVENT_MOUSEMOVE:
            if (dragging) {
                cv::Point2f currentPos(static_cast<float>(x), static_cast<float>(y));
                cv::Point2f delta = currentPos - lastPos;
                viewAngleY += delta.x * 0.01f;
                viewAngleX += delta.y * 0.01f;
                lastPos = currentPos;
                
                // Force redraw when rotating to see immediate effect
                if (showDebugInfo) printf("Debug: Rotating to X: %.2f, Y: %.2f\n", viewAngleX, viewAngleY);
            }
            break;
            
        // FIXED: Handle all possible mouse wheel event variations for different OpenCV versions
        case CV_EVENT_MOUSEWHEEL:
            if ((flags & CV_EVENT_FLAG_WHEELUP) != 0 || 
                (flags & 0x0001) != 0 || 
                flags > 0) {
                zoomLevel *= 1.2f; // Stronger zoom in
                if (showDebugInfo) printf("Debug: Zoom in to %.2f\n", zoomLevel);
            } else if ((flags & CV_EVENT_FLAG_WHEELDOWN) != 0 || 
                      (flags & 0x0002) != 0 || 
                      flags < 0) {
                zoomLevel /= 1.2f; // Stronger zoom out
                if (showDebugInfo) printf("Debug: Zoom out to %.2f\n", zoomLevel);
            }
            break;
    }

    // Prevent extreme zooming, but allow higher maximum zoom
    zoomLevel = std::max(0.1f, std::min(zoomLevel, 10.0f));
}


// Add this helper function before DisplayPointCloud():
void UpdateViewMatrix(float rotX, float rotY) {
    // Create rotation matrices
    float cosX = cos(rotX);
    float sinX = sin(rotX);
    float cosY = cos(rotY);
    float sinY = sin(rotY);
    
    // Rotation around X
    float rotMatX[16] = {
        1, 0,    0,     0,
        0, cosX, -sinX, 0,
        0, sinX, cosX,  0,
        0, 0,    0,     1
    };
    
    // Rotation around Y
    float rotMatY[16] = {
        cosY,  0, sinY, 0,
        0,     1, 0,    0,
        -sinY, 0, cosY, 0,
        0,     0, 0,    1
    };
    
    // Multiply matrices
    float temp[16];
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            temp[i*4 + j] = 0;
            for(int k = 0; k < 4; k++) {
                temp[i*4 + j] += rotMatX[i*4 + k] * rotMatY[k*4 + j];
            }
        }
    }
    
    // Copy result to viewMatrix
    memcpy(viewMatrix, temp, sizeof(float) * 16);
}

void RenderPointCloud(cv::Mat& image, const std::vector<cv::Point3f>& points, float minZ, float maxZ) {
    // Create vector for sorting points by depth
    std::vector<std::pair<float, const cv::Point3f*>> sortedPoints;
    sortedPoints.reserve(points.size());
    
    // Filter and store valid points
    // FIXED: Expanded valid depth range to capture more points
    const float MIN_DEPTH = 20.0f;
    const float MAX_DEPTH = 8000.0f;
    
    // FIXED: Calculate the point cloud center for better centering
    float avgX = 0, avgY = 0, avgZ = 0;
    int validCount = 0;
    
    for (const auto& pt : points) {
        if (pt.z > MIN_DEPTH && pt.z < MAX_DEPTH) {
            sortedPoints.push_back(std::make_pair(pt.z, &pt));
            avgX += pt.x;
            avgY += pt.y;
            avgZ += pt.z;
            validCount++;
        }
    }
    
    // Calculate center point for centering
    if (validCount > 0) {
        avgX /= validCount;
        avgY /= validCount;
        avgZ /= validCount;
        if (showDebugInfo) printf("Debug: Point cloud center at %.2f, %.2f, %.2f\n", avgX, avgY, avgZ);
    }
    
    // Sort points by depth (farther points drawn first)
    std::sort(sortedPoints.begin(), sortedPoints.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });

    const float cosX = std::cos(viewAngleX);
    const float sinX = std::sin(viewAngleX);
    const float cosY = std::cos(viewAngleY);
    const float sinY = std::sin(viewAngleY);

    const int w = static_cast<int>(windowW);
    const int h = static_cast<int>(windowH);
    const int centerX = w/2;
    const int centerY = h/2;

    // FIXED: Improved projection with better centering
    for (const auto& pair : sortedPoints) {
        const cv::Point3f& pt = *pair.second;
        const float depth = pair.first;

        // Center the point cloud by subtracting the average position
        const float centeredX = pt.x - avgX;
        const float centeredY = pt.y - avgY;
        const float centeredZ = pt.z - avgZ;

        // Apply rotation
        const float x = centeredX * cosY + centeredZ * sinY;
        const float y = centeredY * cosX - (-centeredX * sinY + centeredZ * cosY) * sinX;
        const float z = centeredY * sinX + (-centeredX * sinY + centeredZ * cosY) * cosX;

        // FIXED: Improved projection with better scale factors for larger display
         float scale = zoomLevel; // Adjusted for better scaling
        const int screenX = static_cast<int>(centerX + x * scale);
        const int screenY = static_cast<int>(centerY - y * scale);

        // Draw point if in bounds
        if (screenX >= 0 && screenX < w && screenY >= 0 && screenY < h) {
            // Normalize depth for color mapping (improved normalization)
            float normalizedZ = (depth - minZ) / (maxZ - minZ);
            // Ensure normalizedZ is between 0 and 1
            normalizedZ = std::max(0.0f, std::min(1.0f, normalizedZ));
            const cv::Scalar color = GetDepthColor(normalizedZ);
            
            // FIXED: Larger point sizes for better visibility
            const int pointSize = 1; 
            cv::circle(image, cv::Point(screenX, screenY), pointSize, color, -1);
        }
    }
}


// Improved point cloud display with better visualization and controls
void DisplayPointCloud() {
    if (points3D.empty()) {
        cv::Mat emptyImage = cv::Mat::zeros(windowH, windowW, CV_8UC3);
        cv::putText(emptyImage, "No valid depth points detected", 
                    cv::Point(50, windowH/2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::imshow(windowPointCloud, emptyImage);
        return;
    }

    bool running = true;
    while (running) {
        cv::namedWindow(windowPointCloud, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowPointCloud, windowW, windowH);
        cv::setMouseCallback(windowPointCloud, OnMouseCallback, nullptr);

        // Auto-find optimal depth range
        float minZ = FLT_MAX, maxZ = -FLT_MAX;
        std::vector<float> depths;
        depths.reserve(points3D.size());
        
        for (const auto& pt : points3D) {
            if (pt.z > 20.0f && pt.z < 5000.0f) {
                depths.push_back(pt.z);
                minZ = std::min(minZ, pt.z);
                maxZ = std::max(maxZ, pt.z);
            }
        }
        
        if (depths.size() > 100) {
            std::sort(depths.begin(), depths.end());
            size_t lowIdx = depths.size() * 0.02;
            size_t highIdx = depths.size() * 0.98;
            if (lowIdx < depths.size() && highIdx < depths.size()) {
                minZ = depths[lowIdx];
                maxZ = depths[highIdx];
            }
        }

        if (autoRotate) {
            viewAngleY += autoRotateSpeed;
        }

        cv::Mat depthImage = cv::Mat::zeros(windowH, windowW, CV_8UC3);
        
        // Draw grid
        int gridSize = 50;
        for (int x = 0; x < windowW; x += gridSize) {
            cv::line(depthImage, cv::Point(x, 0), cv::Point(x, windowH), cv::Scalar(20, 20, 20), 1);
        }
        for (int y = 0; y < windowH; y += gridSize) {
            cv::line(depthImage, cv::Point(0, y), cv::Point(windowW, y), cv::Scalar(20, 20, 20), 1);
        }
        
        RenderPointCloud(depthImage, points3D, minZ, maxZ);

        // Draw color scale and info
        int barWidth = 30;
        int barHeight = static_cast<int>(windowH - 100);
        cv::rectangle(depthImage, 
                     cv::Point(10, 50), 
                     cv::Point(barWidth + 10, 50 + barHeight), 
                     cv::Scalar(255, 255, 255), 1);
                     
        for (int y = 0; y < barHeight; y++) {
            float normalizedZ = 1.0f - static_cast<float>(y) / barHeight;
            cv::Scalar color = GetDepthColor(normalizedZ);
            cv::line(depthImage, 
                    cv::Point(11, y + 50), 
                    cv::Point(barWidth + 9, y + 50), 
                    color, 1);
        }

        char minZStr[20], maxZStr[20];
        sprintf_s(minZStr, sizeof(minZStr), "%.1f mm", minZ);
        sprintf_s(maxZStr, sizeof(maxZStr), "%.1f mm", maxZ);
        
        cv::putText(depthImage, minZStr, 
                   cv::Point(barWidth + 15, barHeight + 45), 
                   cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
        cv::putText(depthImage, maxZStr, 
                   cv::Point(barWidth + 15, 60), 
                   cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);

        auto addText = [&](const std::string& text, int y) {
            cv::putText(depthImage, text, 
                       cv::Point(barWidth + 15, y), 
                       cv::FONT_HERSHEY_PLAIN, 1.2, 
                       cv::Scalar(0, 0, 0), 3);
            cv::putText(depthImage, text, 
                       cv::Point(barWidth + 15, y), 
                       cv::FONT_HERSHEY_PLAIN, 1.2, 
                       cv::Scalar(255, 255, 255), 1);
        };

        addText("Controls:", 30);
        addText("Left mouse: Rotate", 50);
        addText("Mouse wheel: Zoom", 70);
        addText("A: Toggle auto-rotate", 90);
        addText("R: Reset view", 110);
        addText("V: Toggle view mode", 130);  // NEW
        addText("C: Camera-match view", 150); // NEW
        addText("E: Export camera pattern", 170); // NEW
        addText("Points: " + std::to_string(points3D.size()), 190);  // was 170
addText("Range: " + std::to_string(static_cast<int>(minZ)) + 
        " - " + std::to_string(static_cast<int>(maxZ)) + " mm", 210);  // was 190

        cv::imshow(windowPointCloud, depthImage);

        int key = cv::waitKey(1);
        switch(key) {
            case 'r': case 'R':
                viewAngleX = 0.25f;
                viewAngleY = 0.15f;
                zoomLevel = 0.5f;
                break;
            case 'c': case 'C':  // NEW: Camera-matching view
                SetCameraMatchingView();
                break;
            case 'v': case 'V':  // NEW: Toggle view mode
                ToggleViewMode();
                break;
            case 'a': case 'A':
                autoRotate = !autoRotate;
                break;
            case '+':
                zoomLevel *= 1.2f;
                break;
            case '-':
                zoomLevel /= 1.2f;
                break;
            case 27: case 'q': case 'Q':
                running = false;
                break;
        }
        zoomLevel = std::max(0.1f, std::min(zoomLevel, 10.0f));
        addText("X: Export 3D visualization", 230); // NEW
    }
}

float LookupMEMSAngleCorrection(float angleNominal) {
    float dividend, divisor;
    for (int i = 0; i < calMEMSLines; i++) {
        if (angleNominal < calMEMSAnglesNominal[i]) {
            dividend = (calMEMSAnglesActual[i] * (angleNominal - calMEMSAnglesNominal[i - 1]) + calMEMSAnglesActual[i - 1] * (-angleNominal + calMEMSAnglesNominal[i]));
            divisor = (calMEMSAnglesNominal[i] - calMEMSAnglesNominal[i - 1]);
            if (divisor == 0.f)
                divisor = 0.001f;
            return dividend / divisor;
        }
    }
    return -1000000.f;
}

// Add new function for Lissajous pattern generation
// ===============================
// REPLACED PrepareLissajousScanData: Handles all Lissajous-type and Bidirectional/Radial patterns
// ===============================
unsigned int PrepareLissajousScanData(unsigned int &sampleRate, float scanDuration, float* angles) {
    sampleRate = spsDefault;
    unsigned int totalPoints = static_cast<unsigned int>(sampleRate * scanDuration);
    if (totalPoints > maxSamplesPerFrame) {
        totalPoints = maxSamplesPerFrame;
        scanDuration = static_cast<float>(totalPoints) / static_cast<float>(sampleRate);
    }
    float timeStep = 1.0f / static_cast<float>(sampleRate);

    triggerTimes.clear();

    // We’ll keep the same ON/OFF cycle for the laser
    int pointsPerTrigger = 50; 
    int offPoints = 10;        
    int cycle = pointsPerTrigger + offPoints;

    if (showDebugInfo) {
        printf("PrepareLissajousScanData: mode=%d, fX=%.1f Hz, fY=%.1f Hz, duration=%.3f s\n",
                currentScanMode, lissajousFreqX, lissajousFreqY, scanDuration);
        printf("  TotalPoints: %u, SampleRate: %u\n", totalPoints, sampleRate);
    }

    for (unsigned int i = 0; i < totalPoints; i++) {
        float t = i * timeStep;

        // ===== PATTERN SWITCH =====
        if (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS) {
            // Pattern 1: Sinusoidal Lissajous
            xData[i] = sinusoidalWave(lissajousFreqX, t, lissajousPhase);
            yData[i] = sinusoidalWave(lissajousFreqY, t, 0.0f);
        }
        else if (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS) {
            // Pattern 2: Triangular Lissajous
            xData[i] = triWave(lissajousFreqX, t, lissajousPhase);
            yData[i] = triWave(lissajousFreqY, t, 0.0f);
        }
        else if (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) {
    // true bidirectional‐Cartesian per MATLAB reference
    bool isSecondHalf = (t > (scanDuration / 2.0f));
    float adjustedTime = isSecondHalf 
                         ? (t - scanDuration / 2.0f) 
                         : t;

    // Compute “f2_bc” = 2*f1/NP (NP == numScanLines)
    float f1 = lissajousFreqX;
    float f2_bc = (2.0f / static_cast<float>(NP)) * lissajousFreqX;

    if (!isSecondHalf) {
        // First half: X uses f1, Y uses f2_bc
        xData[i] = sinf(2.0f * M_PI * f1 * adjustedTime + lissajousPhase);
        yData[i] = sinf(2.0f * M_PI * f2_bc * adjustedTime);
    } else {
        // Second half: X uses f2_bc, Y uses f1
        xData[i] = sinf(2.0f * M_PI * f2_bc * adjustedTime);
        yData[i] = sinf(2.0f * M_PI * f1 * adjustedTime + lissajousPhase);
    }
}

        else if (currentScanMode == SCAN_RADIAL_LISSAJOUS) {
    // correct radial Lissajous per MATLAB:
    float f1 = lissajousFreqX;
    float f2 = lissajousFreqY;
    // X = sin(2π f1 t + φ) * sin(2π f2 t)
    xData[i] = sinf(2.0f * M_PI * f1 * t + lissajousPhase)
               * sinf(2.0f * M_PI * f2 * t);
    // Y = sin(2π f2 t) * cos(2π f1 t)
    yData[i] = sinf(2.0f * M_PI * f2 * t)
               * cosf(2.0f * M_PI * f1 * t);
}


        // Laser ON/OFF cycling (identical for all patterns)
        int posInCycle = i % cycle;
        if (posInCycle < pointsPerTrigger) {
            mData[i] = 0xFF; // Laser ON
        } else {
            mData[i] = 0x00; // Laser OFF
        }
        // Record the trigger time whenever a new ON segment starts
        if (posInCycle == 0) {
            triggerTimes.push_back(t);
        }
    }

    // Scale & offset to camera→MEMS angles
    cv::Point2d ILUTMaxAngles = camera->GetILUTMaxAngles();
    cv::Rect limits(-ILUTMaxAngles.x, -ILUTMaxAngles.y,
                    ILUTMaxAngles.x * 2, ILUTMaxAngles.y * 2);
    ScaleAndOffsetArray(limits, totalPoints);
    CameraAngleToMEMSCommands(totalPoints);

    // Store limited samples for reconstruction (one per cycle)
    int stored = 0;
    for (unsigned int i = 0; i < totalPoints && stored < maxNumLines; ++i) {
        if ((i % cycle) == 0) {
            lissajousMEMSX[stored] = xData[i];   // actual mirror commands
            lissajousMEMSY[stored] = yData[i];
            ++stored;
        }
    }

    // Build the 'angles' array for each trigger (up to maxNumLines)
    int numTriggers = static_cast<int>(triggerTimes.size());
    for (int i = 0; i < numTriggers && i < maxNumLines; i++) {
        float t = triggerTimes[i];
        float xAngle;
        if (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS) {
            xAngle = sinusoidalWave(lissajousFreqX, t, lissajousPhase);
        }
        else if (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS) {
            xAngle = triWave(lissajousFreqX, t, lissajousPhase);
        }
        else if (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) {
            bool isSecondHalf = (t > (scanDuration / 2.0f));
            float adjustedTime = isSecondHalf ? (t - scanDuration/2.0f) : t;
            xAngle = isSecondHalf
                     ? sinusoidalWave(lissajousFreqY * 2.0f, adjustedTime, 0.0f)
                     : sinusoidalWave(lissajousFreqX, adjustedTime, 0.0f);
        }
        else if (currentScanMode == SCAN_RADIAL_LISSAJOUS) {
            xAngle = radialLissajousX(lissajousFreqX, t, lissajousPhase);
        }
        else {
            xAngle = sinusoidalWave(lissajousFreqX, t, lissajousPhase);
        }

        angles[i] = (xAngle * scanAngleMagnitudes.x) + scanAngleOffsets.x;
        angles[i] = std::max((float)-ILUTMaxAngles.x,
                             std::min((float)ILUTMaxAngles.x, angles[i]));
    }

    return totalPoints;
}



unsigned int PrepareScanLineData(unsigned int &sampleRate, int lines, float* angles, bool calibrateOnPlane) {
   // Calculate the raster scan based on variable settings
   int npts = 50000;
   int pixels = 6;
   int ppRaster = 0;
   int angle = 0;
   int bidirF = 1;
   int dRotate = 0;

   // do not allow less than 2 lines to avoid divide by zero below
   lines = std::max(lines, 2);
   // get limits for scan data size
   MTIDeviceParams params;
   mti->GetDeviceParams(&params);
   if (!strstr(params.DeviceName, "MTI-MZ-")) {
    printf("\n\nConnected to incompatible Mirrorcle MEMS Controller (Not MTI-MZ-...).  Press any key to Exit.\n");
    mti->SendSerialReset();
    _getch();
    return -1;
}
   unsigned int maxNumSamples = params.DeviceLimits.SamplesPerFrame_Max;
   int sps_max = params.DeviceLimits.SampleRate_Max;
   int sps_min = params.DeviceLimits.SampleRate_Min;

   // call the function which will return npts and update sampleRate by reference
   // if npts == 0, the function could not satisfy request, either too many total_pixels (lines*pixels) or too fast of a scan rate (sps_max exceeded)
   npts = datagen->LinearRasterPattern(xData, yData, mData, 1, 1, lines, pixels, lineDuration, ppRaster, bidirF, dRotate, float(angle*MTI_DEGTORAD), sampleRate, sps_min, sps_max);
   
   if (npts < 1) {
       printf("\n!! LinearRasterPattern function could not satisfy requested parameters. !!");
       printf("\nLines*Pixels total is too high (especially in point-to-point raster), or \nscan rate exceeds max SampleRate of the Controller.\n");
       printf("\nPress any key to return to the menu...");
       return -1;
   }

   if (npts > maxNumSamples) {
       printf("\nToo many samples to run. Number of points (%d) exceeds Controller buffer size (%d)\n", npts, maxNumSamples);
       _getch();
       return -1;
   }

   // Calculate commands to achieve true MEMS angle using ILUT
   cv::Point2d ILUTMaxAngles = camera->GetILUTMaxAngles();
   cv::Rect limits(-ILUTMaxAngles.x, -ILUTMaxAngles.y, ILUTMaxAngles.x * 2, ILUTMaxAngles.y * 2);
   ScaleAndOffsetArray(limits, npts);
   CameraAngleToMEMSCommands(npts);
   
   // now prepare array of X angles for 3D scanning
   for (int i=0; i<lines; i++) {
       angles[i] = -scanAngleMagnitudes.x + (float)i*(2*scanAngleMagnitudes.x)/((float)(lines-1)) + scanAngleOffsets.x;
       // check limits
       angles[i] = std::max(std::min(angles[i], (float)ILUTMaxAngles.x), (float)-ILUTMaxAngles.x);
       if (calibrateOnPlane)
           calMEMSAnglesNominal[i] = angles[i];
       else {
           float correctedAngle = LookupMEMSAngleCorrection(angles[i]);
           if (correctedAngle > -1000000.f) {
               angles[i] = correctedAngle;
           } else {
               // If lookup failed, just use the uncorrected angle
               if (showDebugInfo) {
                   printf("Warning: Failed to correct MEMS angle %f\n", angles[i]);
               }
           }
       }
   }
   return npts;
}




// 2. ADD THIS SIMPLE FUNCTION:
void SaveScanData() {
    if (points3D.empty()) return;
    
    // Get scan mode name
    const char* modeName = 
        (currentScanMode == SCAN_RASTER) ? "Raster" :
        (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS) ? "Sinusoidal" :
        (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS) ? "Triangular" :
        (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional" :
        (currentScanMode == SCAN_RADIAL_LISSAJOUS) ? "Radial" : "Unknown";
    
    // Calculate basic stats
    float sum = 0, sumSq = 0, minZ = 9999, maxZ = 0;
    for (const auto& pt : points3D) {
        if (pt.z > 20 && pt.z < 5000) {
            sum += pt.z;
            minZ = std::min(minZ, pt.z);
            maxZ = std::max(maxZ, pt.z);
        }
    }
    float mean = sum / points3D.size();
    for (const auto& pt : points3D) {
        if (pt.z > 20 && pt.z < 5000) {
            sumSq += (pt.z - mean) * (pt.z - mean);
        }
    }
    float std = sqrt(sumSq / points3D.size());
    float cv = (std / mean) * 100;
    
    // 1. Save individual scan file (unique name)
    char filename[100];
    sprintf(filename, "%s_NP%d_%.0fHz.csv", modeName, numScanLines, lissajousFreqX);
    
    std::ofstream scanFile(filename);
    scanFile << "x,y,z\n";
    for (const auto& pt : points3D) {
        scanFile << pt.x << "," << pt.y << "," << pt.z << "\n";
    }
    scanFile.close();
    
    // 2. Save to master comparison file
    if (!masterFileOpen) {
        masterFile.open("all_scans_comparison.csv");
        masterFile << "ScanMode,NP,FreqX,FreqY,TotalPoints,MeanDepth,StdDepth,CV,MinDepth,MaxDepth,Filename\n";
        masterFileOpen = true;
    }
    
    masterFile << modeName << ","
               << numScanLines << ","
               << lissajousFreqX << ","
               << lissajousFreqY << ","
               << points3D.size() << ","
               << mean << ","
               << std << ","
               << cv << ","
               << minZ << ","
               << maxZ << ","
               << filename << "\n";
    masterFile.flush();
    
    printf("✓ Data saved: %s (Points: %zu, Mean: %.1fmm, CV: %.1f%%)\n", 
           filename, points3D.size(), mean, cv);
}


void ScanAndDetectLineDemo() {
    system(CLEARSCREEN);
    printf("\nScan And Detect Line Demo\n\n");

    // For the laser scan, it uses the hardware trigger of the camera to precisely time the exposure
    // Try to enable hardware trigger on the camera with SetHardwareTrigger(true)
    if (camera->SetHardwareTrigger(true) != 0) {
        printf("This camera doesn't support hardware triggering. Press any key to continue...\n");
        camera->SetHardwareTrigger(false);
        _getch();
        return;
    }

    printf("\tChoose operating mode:\n");
    printf("\t(S)ingle scan\n");
    printf("\t(C)ontinuous scan\n");
    printf("\t(ESC) to return to main menu\n");

    bool continuousMode = false;
    bool exitFunction = false;
    
    while (int ch = _getch()) {
        if (ch == 27) { // ESC to exit function
            exitFunction = true;
            break;
        }
        if (ch == 'S' || ch == 's') {
            continuousMode = false;
            break;
        }
        if (ch == 'C' || ch == 'c') {
            continuousMode = true;
            break;
        }
    }
    
    if (exitFunction) {
        camera->SetHardwareTrigger(false);
        return;
    }
    
    // Clear the 3D point cloud before starting a new scan
    points3D.clear();
    
    // Initialize variables for auto-positioning
    float sumX = 0, sumY = 0, sumZ = 0;
    int validPointCount = 0;

    if (continuousMode) {
    printf("\nRunning in continuous mode... Press ESC to quit\n");
    printf(
        "Current scan mode: %s\n",
        (currentScanMode == SCAN_RASTER)                 ? "Raster" :
        (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS)   ? "Sinusoidal Lissajous" :
        (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS)   ? "Triangular Lissajous" :
        (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional Cartesian" :
        (currentScanMode == SCAN_RADIAL_LISSAJOUS)       ? "Radial Lissajous" :
                                                          "Unknown"
    );
}
else {
    printf("\nRunning single scan. Press any key to return to the main menu\n");
    printf(
        "Current scan mode: %s\n",
        (currentScanMode == SCAN_RASTER)                 ? "Raster" :
        (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS)   ? "Sinusoidal Lissajous" :
        (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS)   ? "Triangular Lissajous" :
        (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional Cartesian" :
        (currentScanMode == SCAN_RADIAL_LISSAJOUS)       ? "Radial Lissajous" :
                                                          "Unknown"
    );
}

    cv::Size res(camera->GetCameraWidth(), camera->GetCameraHeight());

    // Create control window with sliders for exposure and threshold
    cv::namedWindow(windowControl, cv::WINDOW_NORMAL);
    cv::createTrackbar("Exposure Time", windowControl, &exposure_time, exposure_slider_max, OnTrackbarChange);
    cv::createTrackbar("Laser Threshold", windowControl, &BWThreshold, threshold_slider_max, OnThresholdChange);
    
    // Set up variables for the loop
    std::vector<std::vector<cv::Point>> allLines;
    std::vector<cv::Point> locations;
    std::vector<cv::Point> linePoints;
    std::vector<std::vector<float>> yValues(res.height);
    cv::Mat frame, frameOriginal;
    
    bool shouldExit = false;
    
    do {
        // Reset points3D for each new scan
        points3D.clear();
        sumX = 0;
        sumY = 0;
        sumZ = 0;
        validPointCount = 0;
        allLines.clear();
        
        // Prepare scan data
        scanAngleMagnitudes = camera->GetILUTMaxAngles();
        scanAngleOffsets = cv::Point2f(scanAngleMagnitudes.x * xOffset, scanAngleMagnitudes.y * yOffset);
        scanAngleMagnitudes = cv::Point2f(scanAngleMagnitudes.x * xAmplitude, scanAngleMagnitudes.y * yAmplitude);
        
        int npts = 0;
    int actualScanLines = numScanLines;
    float totalScanDuration = 0;

    if (currentScanMode == SCAN_RASTER) {
        npts = PrepareScanLineData(sps, numScanLines, MEMSAngles, false);
        totalScanDuration = lineDuration * numScanLines;
    }
    else if (
    currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS ||
    currentScanMode == SCAN_TRIANGULAR_LISSAJOUS ||
    currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN ||
    currentScanMode == SCAN_RADIAL_LISSAJOUS
) {
    totalScanDuration = float(NP) / lissajousFreqX;
    npts = PrepareLissajousScanData(sps, totalScanDuration, MEMSAngles);
    actualScanLines = std::min((int)triggerTimes.size(), (int)NP);
}


    if (npts < 1) {
        camera->SetHardwareTrigger(false);
        mti->ResetDevicePosition();
        return;
    }

        // Call StartCamera() so that it will start listening to hardware triggers
        camera->StartCamera();

        // Prepare to run the scan pattern
        mti->SetDeviceParam(MTIParam::SampleRate, sps);
        mti->StopDataStream();
        
            // after you set the sample rate:
        mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);

        // FIXED: For continuous mode, use proper repeat count
        // For continuous mode, we'll handle repetition manually
        int repeatCount = 1; // Always do one shot and manually repeat
        bool oneShot = true;
        mti->SendDataStream(xData, yData, mData, npts, repeatCount, false, oneShot);

        // Start the scan from the MTI device
        mti->StartDataStream(1, false);

        if (currentScanMode == SCAN_RASTER) {
            // Original raster scanning logic
            for (int scanIdx = 0; scanIdx < actualScanLines; scanIdx++) {
                frame = camera->GetFrame();
                if (frame.empty()) continue;

                frameOriginal = frame.clone();
                if (frame.channels() == 3)
                    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

                cv::inRange(frame, cv::Scalar(BWThreshold), cv::Scalar(255), frame);
                if (cv::countNonZero(frame) <= 0) continue;

                cv::findNonZero(frame, locations);

                if (xAveraging) {
                    std::fill(yValues.begin(), yValues.end(), std::vector<float>(0.0));
                    for (size_t j = 0; j < locations.size(); j++) {
                        if (locations[j].y >= 0 && locations[j].y < res.height) {
                            yValues[locations[j].y].push_back((float)locations[j].x);
                        }
                    }
                    linePoints.clear();
                    for (int y = 0; y < res.height; y++) {
                        float x_avg = 0;
                        if (!yValues[y].empty()) {
                            for (size_t j = 0; j < yValues[y].size(); j++) {
                                x_avg += yValues[y][j] / static_cast<float>(yValues[y].size());
                            }
                            linePoints.push_back(cv::Point((int)x_avg, y));
                            
                            cv::Point3f point3D;
                            CalculateTriangulationPoint(cv::Point2f(x_avg, (float)y), MEMSAngles[scanIdx], 0.0f, point3D);
                            if (point3D.z > 20.0f && point3D.z < 5000.0f) {
                                points3D.push_back(point3D);
                                sumX += point3D.x;
                                sumY += point3D.y;
                                sumZ += point3D.z;
                                validPointCount++;
                            }
                        }
                    }
                    allLines.push_back(linePoints);
                }
                else {
                    allLines.push_back(locations);
                    for (const auto& pt : locations) {
                        cv::Point3f point3D;
                        CalculateTriangulationPoint(cv::Point2f((float)pt.x, (float)pt.y), MEMSAngles[scanIdx], 0.0f, point3D);
                        if (point3D.z > 20.0f && point3D.z < 5000.0f) {
                            points3D.push_back(point3D);
                            sumX += point3D.x;
                            sumY += point3D.y;
                            sumZ += point3D.z;
                            validPointCount++;
                        }
                    }
                }
            }
        }
            else {  // LISSAJOUS MODE - Dense Point Cloud Approach
            // Capture many more frames for dense point cloud
            int maxFramesToCapture = std::min((int)triggerTimes.size(), 200);
            
            for (int scanIdx = 0; scanIdx < maxFramesToCapture; scanIdx++) {
                frame = camera->GetFrame();
                if (frame.empty()) continue;

                if (frame.channels() == 3)
                    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

                cv::inRange(frame, cv::Scalar(BWThreshold), cv::Scalar(255), frame);
                if (cv::countNonZero(frame) <= 0) continue;

                cv::findNonZero(frame, locations);

                 // **Compute both mirror angles for this trigger**  
             // Use the _actual_ MEMS mirror angles we recorded
    float x_angle = lissajousMEMSX[scanIdx];
    float y_angle = lissajousMEMSY[scanIdx];
    // Process ALL detected points with correct angles
    for (const auto& pt : locations) {
        cv::Point3f P;
        // FIXED: Pass both X and Y angles to triangulation
        CalculateTriangulationPoint(cv::Point2f((float)pt.x, (float)pt.y), x_angle, y_angle, P);
        if (P.z > 20.0f && P.z < 5000.0f) {
            points3D.push_back(P);
            sumX += P.x; sumY += P.y; sumZ += P.z;
            validPointCount++;
        }
    }

                allLines.push_back(locations);
                
    }
        if (showDebugInfo) {
    printf("Debug: Lissajous scan complete. Points captured: %d\n", validPointCount);
    printf("Debug: Trigger times recorded: %d\n", (int)triggerTimes.size());
    printf("Debug: Total scan points: %d\n", npts);
    printf("Debug: Laser was ON for all points (continuous mode)\n");
}
    }

        // Wait for scan to complete
        while (mti->GetSamplesRemaining() != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Export camera view pattern if requested
if (currentScanMode != SCAN_RASTER && showDebugInfo) {
    ExportCameraViewPattern(allLines, frame);
}


        // Auto-position view based on point cloud statistics
        if (validPointCount > 10) {
        // tiny “face-on” tilt — exactly what you see in the Camera window
        viewAngleX = 0.05f;
        viewAngleY = 0.05f;

        // auto-zoom off the average depth so it fills the window
        float avgZ = sumZ / validPointCount;
        zoomLevel = 1500.0f / avgZ;
        zoomLevel = std::max(0.5f, std::min(zoomLevel, 10.0f));
    }


    // ----------------------------
    // EXPORT RAW POINTS TO DISK
    {
        // --- Write PLY file ---
        std::ofstream plyFile("scan.ply");
        if (plyFile.is_open()) {
            plyFile << "ply\n"
                    << "format ascii 1.0\n"
                    << "element vertex " << points3D.size() << "\n"
                    << "property float x\n"
                    << "property float y\n"
                    << "property float z\n"
                    << "end_header\n";
            for (const auto &p : points3D) {
                plyFile << p.x << " "
                        << p.y << " "
                        << p.z << "\n";
            }
            plyFile.close();
        }

        // --- Write CSV file ---
        std::ofstream csvFile("scan.csv");
        if (csvFile.is_open()) {
            csvFile << "x,y,z\n";
            for (const auto &p : points3D) {
                csvFile << p.x << "," 
                        << p.y << "," 
                        << p.z << "\n";
            }
            csvFile.close();
        }
    }
    // Export 3D visualization (like paper Figure 6d)
    // Export 3D visualization (like paper Figure 6d)
    if (!points3D.empty()) {
        std::string modeName = 
            (currentScanMode == SCAN_RASTER) ? "Raster" :
            (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS) ? "Sinusoidal" :
            (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS) ? "Triangular" :
            (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional" :
            (currentScanMode == SCAN_RADIAL_LISSAJOUS) ? "Radial" : "Unknown";
        
        ExportComprehensive3DProfile(points3D, modeName, numScanLines, lissajousFreqX);
        Export3DVisualization(points3D, frame);
    }
    // ----------------------------
    // ----------------------------


        
        // Display the frame of detected scan lines
        camera->StopCamera();
        camera->SetHardwareTrigger(false);
        camera->StartCamera();
        frame = camera->GetFrame();
        if (frame.empty())
            frame = cv::Mat::zeros(res, CV_8UC3);
        camera->StopCamera();
        camera->SetHardwareTrigger(true);
        camera->StartCamera();

        if (frame.channels() == 1) {
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        }

        // Draw the scanned lines (for each line)
for (size_t i = 0; i < allLines.size(); i++) {
    const auto& line = allLines[i];
    for (size_t j = 1; j < line.size(); j++) {
        cv::line(frame, line[j - 1], line[j], cv::Scalar(50, 150, 200), 2);
    }
    // Optionally, still draw the points for emphasis:
    for (size_t j = 0; j < line.size(); j++) {
        cv::circle(frame, line[j], 2, cv::Scalar(50, 150, 200), -1);
    }
}

        cv::putText(frame, "Detected laser points: " + std::to_string(points3D.size()), 
                   cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 2));
        {
    const char* modeName =
        (currentScanMode == SCAN_RASTER)                 ? "Raster" :
        (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS)   ? "Sinusoidal Lissajous" :
        (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS)   ? "Triangular Lissajous" :
        (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional Cartesian" :
        (currentScanMode == SCAN_RADIAL_LISSAJOUS)       ? "Radial Lissajous" :
                                                          "Unknown";
    cv::putText(frame, std::string("Scan mode: ") + modeName,
                cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 2));
}


        cv::putText(frame, "Valid 3D points: " + std::to_string(validPointCount), 
                   cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 2));
        if (continuousMode) {
            cv::putText(frame, "Press ESC to stop scanning", 
                       cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 2));
        } else {
            cv::putText(frame, "Press any key to close", 
                       cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 2));
        }

        cv::imshow(windowCamera, frame);
        DisplayPointCloud();

        // Process keyboard input
        if (continuousMode) {
            int key = cv::waitKey(30);
            
            if (key == 27) { // ESC key
                shouldExit = true;
            }
            else if (key == 'S' || key == 's') {
                // Adjust scale
                if (key == 'S') pointCloudScale *= 1.1f;
                else pointCloudScale /= 1.1f;
            }
            else if (key == 'R' || key == 'r') {
                // Reset view
                viewAngleX = 0.25f;
                viewAngleY = 0.15f;
                zoomLevel = 0.5f;
            }
            else if (key == 'A' || key == 'a') {
                // Toggle auto-rotation
                autoRotate = !autoRotate;
            }
            else if (key == 'D' || key == 'd') {
                // Toggle debug info
                showDebugInfo = !showDebugInfo;
                printf("Debug info %s\n", showDebugInfo ? "enabled" : "disabled");
            }
            else if (_kbhit()) {
                int ch = _getch();
                if (ch == 27) { // ESC key from console
                    shouldExit = true;
                }
            }
            
            // For continuous mode, stop the stream before restarting
            if (!shouldExit) {
                mti->StopDataStream();
                // Wait a bit to ensure device is ready
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        else {
            // Single scan mode - wait for any key
            int key = cv::waitKey(0);
            if (key == 'D' || key == 'd') {
                showDebugInfo = !showDebugInfo;
                printf("Debug info %s\n", showDebugInfo ? "enabled" : "disabled");
                continue;
            }
            else if (key == 'S' || key == 's') {
                if (key == 'S') pointCloudScale *= 1.1f;
                else pointCloudScale /= 1.1f;
                continue;
            }
            else if (key == 'R' || key == 'r') {
                viewAngleX = 0.25f;
                viewAngleY = 0.15f;
                zoomLevel = 0.5f;
                continue;
            }
            else if (key == 'A' || key == 'a') {
                autoRotate = !autoRotate;
                continue;
            }
            else if (key == 'E' || key == 'e') {
            // Export camera pattern
            ExportCameraViewPattern(allLines, frame);
            continue;
           }
           else if (key == 'X' || key == 'x') {
            // Export 3D visualization
            Export3DVisualization(points3D, frame);
            continue;
           }
            shouldExit = true;
        }

    } while (continuousMode && !shouldExit);


    SaveScanData();

    // Cleanup
    camera->StopCamera();
    camera->SetHardwareTrigger(false);
    mti->SetDeviceParam(MTIParam::SampleRate, spsDefault);
    mti->StopDataStream();
    mti->ResetDevicePosition();
    cv::destroyAllWindows();
}


// Add this complete function before ScanAndDetectLineDemo()
void ExportCameraViewPattern(const std::vector<std::vector<cv::Point>>& allLines, const cv::Mat& currentFrame) {
    printf("\nExporting camera view pattern...\n");
    
    // Create a black image to accumulate all laser points
    cv::Mat patternImage = cv::Mat::zeros(camera->GetCameraHeight(), camera->GetCameraWidth(), CV_8UC3);
    cv::Mat patternMask = cv::Mat::zeros(camera->GetCameraHeight(), camera->GetCameraWidth(), CV_8UC1);
    
    // Accumulate all detected points from allLines
    int totalPoints = 0;
    for (size_t i = 0; i < allLines.size(); i++) {
        const auto& line = allLines[i];
        for (const auto& pt : line) {
            // Draw each point on the pattern
            if (pt.x >= 0 && pt.x < patternImage.cols && pt.y >= 0 && pt.y < patternImage.rows) {
                patternImage.at<cv::Vec3b>(pt.y, pt.x) = cv::Vec3b(255, 255, 255); // White points
                patternMask.at<uchar>(pt.y, pt.x) = 255;
                totalPoints++;
            }
        }
    }
    
    // Apply some morphological operations to connect nearby points
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(patternMask, patternMask, kernel);
    cv::erode(patternMask, patternMask, kernel);
    
    // Create colored version for better visualization
    cv::Mat coloredPattern = cv::Mat::zeros(camera->GetCameraHeight(), camera->GetCameraWidth(), CV_8UC3);
    
    // Apply color based on position (similar to depth coloring but using position)
    for (int y = 0; y < patternMask.rows; y++) {
        for (int x = 0; x < patternMask.cols; x++) {
            if (patternMask.at<uchar>(y, x) > 0) {
                // Create rainbow effect based on position
                float normalizedX = (float)x / patternMask.cols;
                float normalizedY = (float)y / patternMask.rows;
                
                // Use HSV for smooth color transitions
                float hue = normalizedX * 180.0f; // Hue from 0 to 180
                float saturation = 255;
                float value = 255;
                
                cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation, value));
                cv::Mat rgb;
                cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);
                
                coloredPattern.at<cv::Vec3b>(y, x) = rgb.at<cv::Vec3b>(0, 0);
            }
        }
    }
    
    // Save the pattern images
    cv::imwrite("camera_pattern_bw.png", patternMask);
    cv::imwrite("camera_pattern_colored.png", coloredPattern);
    
    // Also save with the original camera frame as background
    cv::Mat overlayImage;
    if (!currentFrame.empty()) {
        overlayImage = currentFrame.clone();
        if (overlayImage.channels() == 1) {
            cv::cvtColor(overlayImage, overlayImage, cv::COLOR_GRAY2BGR);
        }
        
        // Overlay the pattern
        for (int y = 0; y < patternMask.rows; y++) {
            for (int x = 0; x < patternMask.cols; x++) {
                if (patternMask.at<uchar>(y, x) > 0) {
                    overlayImage.at<cv::Vec3b>(y, x) = coloredPattern.at<cv::Vec3b>(y, x);
                }
            }
        }
        cv::imwrite("camera_pattern_overlay.png", overlayImage);
    }
    
    // Export as CSV with 2D coordinates
    std::ofstream csvFile2D("camera_pattern_2d.csv");
    if (csvFile2D.is_open()) {
        csvFile2D << "x,y,line_index\n";
        for (size_t lineIdx = 0; lineIdx < allLines.size(); lineIdx++) {
            const auto& line = allLines[lineIdx];
            for (const auto& pt : line) {
                csvFile2D << pt.x << "," << pt.y << "," << lineIdx << "\n";
            }
        }
        csvFile2D.close();
    }
    
    printf("Camera view pattern exported (%d points):\n", totalPoints);
    printf("  - camera_pattern_bw.png (black and white)\n");
    printf("  - camera_pattern_colored.png (colored by position)\n");
    printf("  - camera_pattern_overlay.png (overlaid on camera frame)\n");
    printf("  - camera_pattern_2d.csv (2D coordinates)\n\n");
}

// Add this function after your ExportCameraViewPattern function

void Export3DVisualization(const std::vector<cv::Point3f>& points3D, const cv::Mat& currentFrame) {
    printf("\nExporting 3D visualization...\n");
    
    if (points3D.empty()) {
        printf("No 3D points to export!\n");
        return;
    }
    
    // Create visualization windows
    int vizWidth = 800;
    int vizHeight = 600;
    
    // Create multiple views
    cv::Mat frontView = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);
    cv::Mat topView = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);
    cv::Mat sideView = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);
    cv::Mat perspectiveView = cv::Mat::zeros(vizHeight, vizWidth, CV_8UC3);
    
    // Find min/max for each dimension for scaling
    float minX = FLT_MAX, maxX = -FLT_MAX;
    float minY = FLT_MAX, maxY = -FLT_MAX;
    float minZ = FLT_MAX, maxZ = -FLT_MAX;
    
    for (const auto& pt : points3D) {
        if (pt.z > 20.0f && pt.z < 5000.0f) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
            minZ = std::min(minZ, pt.z);
            maxZ = std::max(maxZ, pt.z);
        }
    }
    
    // Add margin
    float marginX = (maxX - minX) * 0.1f;
    float marginY = (maxY - minY) * 0.1f;
    float marginZ = (maxZ - minZ) * 0.1f;
    
    minX -= marginX; maxX += marginX;
    minY -= marginY; maxY += marginY;
    minZ -= marginZ; maxZ += marginZ;
    
    // Calculate centers for better visualization
    float centerX = (minX + maxX) / 2.0f;
    float centerY = (minY + maxY) / 2.0f;
    float centerZ = (minZ + maxZ) / 2.0f;
    
    // Create sorted points for depth ordering
    std::vector<std::pair<float, cv::Point3f>> sortedPoints;
    for (const auto& pt : points3D) {
        if (pt.z > 20.0f && pt.z < 5000.0f) {
            sortedPoints.push_back(std::make_pair(pt.z, pt));
        }
    }
    
    // Sort by depth for perspective view
    std::sort(sortedPoints.begin(), sortedPoints.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Draw points in different views
    for (const auto& pair : sortedPoints) {
        const cv::Point3f& pt = pair.second;
        
        // Normalize depth for color
        float normalizedZ = (pt.z - minZ) / (maxZ - minZ);
        cv::Scalar color = GetDepthColor(normalizedZ);
        
        // Front view (X-Y plane, looking along Z)
        int frontX = static_cast<int>((pt.x - minX) / (maxX - minX) * (vizWidth - 40) + 20);
        int frontY = static_cast<int>((1.0f - (pt.y - minY) / (maxY - minY)) * (vizHeight - 40) + 20);
        cv::circle(frontView, cv::Point(frontX, frontY), 2, color, -1);
        
        // Top view (X-Z plane, looking along Y)
        int topX = static_cast<int>((pt.x - minX) / (maxX - minX) * (vizWidth - 40) + 20);
        int topZ = static_cast<int>((1.0f - (pt.z - minZ) / (maxZ - minZ)) * (vizHeight - 40) + 20);
        cv::circle(topView, cv::Point(topX, topZ), 2, color, -1);
        
        // Side view (Y-Z plane, looking along X)
        int sideZ = static_cast<int>((pt.z - minZ) / (maxZ - minZ) * (vizWidth - 40) + 20);
        int sideY = static_cast<int>((1.0f - (pt.y - minY) / (maxY - minY)) * (vizHeight - 40) + 20);
        cv::circle(sideView, cv::Point(sideZ, sideY), 2, color, -1);
        
        // Perspective view (similar to paper)
        float angle = 0.3f; // Viewing angle
        float px = pt.x - centerX;
        float py = pt.y - centerY;
        float pz = pt.z - centerZ;
        
        // Simple rotation for perspective
        float rx = px * cos(angle) - pz * sin(angle);
        float ry = py;
        float rz = px * sin(angle) + pz * cos(angle) + centerZ;
        
        // Project to 2D
        float scale = (vizWidth * 1.5f) / rz;// Perspective scaling 
        int perspX = static_cast<int>(vizWidth/2 + rx * scale);
        int perspY = static_cast<int>(vizHeight/2 - ry * scale);
        
        if (perspX >= 0 && perspX < vizWidth && perspY >= 0 && perspY < vizHeight) {
            cv::circle(perspectiveView, cv::Point(perspX, perspY), 
                      static_cast<int>(6 * scale), color, -1);
        }
    }
    
    // Add labels and color bar
    auto addLabel = [](cv::Mat& img, const std::string& text, int x, int y) {
        cv::putText(img, text, cv::Point(x, y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    };
    
    addLabel(frontView, "Front View (X-Y)", 10, 30);
    addLabel(topView, "Top View (X-Z)", 10, 30);
    addLabel(sideView, "Side View (Y-Z)", 10, 30);
    addLabel(perspectiveView, "3D Perspective View", 10, 30);
    
    // Add color scale bar to perspective view
    int barX = vizWidth - 60;
    int barY = 50;
    int barWidth = 30;
    int barHeight = vizHeight - 100;
    
    for (int y = 0; y < barHeight; y++) {
        float normalizedDepth = 1.0f - static_cast<float>(y) / barHeight;
        cv::Scalar color = GetDepthColor(normalizedDepth);
        cv::line(perspectiveView, 
                cv::Point(barX, barY + y), 
                cv::Point(barX + barWidth, barY + y), 
                color, 1);
    }
    
    cv::rectangle(perspectiveView, 
                 cv::Point(barX - 1, barY - 1), 
                 cv::Point(barX + barWidth + 1, barY + barHeight + 1), 
                 cv::Scalar(255, 255, 255), 1);
    
    char depthText[50];
    sprintf_s(depthText, sizeof(depthText), "%.0f mm", maxZ);
    addLabel(perspectiveView, depthText, barX - 10, barY - 5);
    sprintf_s(depthText, sizeof(depthText), "%.0f mm", minZ);
    addLabel(perspectiveView, depthText, barX - 10, barY + barHeight + 20);
    
    // Create combined view
    cv::Mat combined(vizHeight * 2, vizWidth * 2, CV_8UC3);
    frontView.copyTo(combined(cv::Rect(0, 0, vizWidth, vizHeight)));
    topView.copyTo(combined(cv::Rect(vizWidth, 0, vizWidth, vizHeight)));
    sideView.copyTo(combined(cv::Rect(0, vizHeight, vizWidth, vizHeight)));
    perspectiveView.copyTo(combined(cv::Rect(vizWidth, vizHeight, vizWidth, vizHeight)));
    
    // Save all views
    cv::imwrite("3d_visualization_front.png", frontView);
    cv::imwrite("3d_visualization_top.png", topView);
    cv::imwrite("3d_visualization_side.png", sideView);
    cv::imwrite("3d_visualization_perspective.png", perspectiveView);
    cv::imwrite("3d_visualization_combined.png", combined);
    
    // Create high-quality perspective view with better shading
    cv::Mat hqPerspective = cv::Mat::zeros(1200, 1600, CV_8UC3);
    
    // Draw with larger points and gradient effect
    for (const auto& pair : sortedPoints) {
        const cv::Point3f& pt = pair.second;
        
        float normalizedZ = (pt.z - minZ) / (maxZ - minZ);
        cv::Scalar color = GetDepthColor(normalizedZ);
        
        // Calculate perspective position with better angle
        float angle1 = 0.4f; // Horizontal rotation
        float angle2 = 0.2f; // Vertical tilt
        
        float px = pt.x - centerX;
        float py = pt.y - centerY;
        float pz = pt.z - centerZ;
        
        // Apply rotations
        float rx = px * cos(angle1) - pz * sin(angle1);
        float ry = py * cos(angle2) - (px * sin(angle1) + pz * cos(angle1)) * sin(angle2);
        float rz = py * sin(angle2) + (px * sin(angle1) + pz * cos(angle1)) * cos(angle2) + centerZ;
        
        // Perspective projection
        float scale = 1200.0f / (rz + 400.0f);
        int x = static_cast<int>(800 + rx * scale * 9.5f);
        int y = static_cast<int>(600 - ry * scale * 9.5f);
        
        if (x >= 0 && x < 1600 && y >= 0 && y < 1200) {
            int radius = static_cast<int>(4 * scale);
            radius = std::max(1, std::min(radius, 6));
            
            // Draw with gradient for 3D effect
            for (int r = radius; r > 0; r--) {
                float intensity = static_cast<float>(r) / radius;
                cv::Scalar gradColor(
                    color[0] * intensity,
                    color[1] * intensity,
                    color[2] * intensity
                );
                cv::circle(hqPerspective, cv::Point(x, y), r, gradColor, -1);
            }
        }
    }
    
    // Add title and info
    cv::putText(hqPerspective, "3D Point Cloud Visualization", 
               cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
    
    std::string modeText = "Mode: " + std::string(currentScanMode != SCAN_RASTER ? "Lissajous" : "Raster");
    cv::putText(hqPerspective, modeText, 
               cv::Point(50, 130), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    
    std::string pointsText = "Points: " + std::to_string(sortedPoints.size());
    cv::putText(hqPerspective, pointsText, 
               cv::Point(50, 170), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    
    cv::imwrite("3d_visualization_hq.png", hqPerspective);
    
    printf("3D visualization exported:\n");
    printf("  - 3d_visualization_front.png (Front view)\n");
    printf("  - 3d_visualization_top.png (Top view)\n");
    printf("  - 3d_visualization_side.png (Side view)\n");
    printf("  - 3d_visualization_perspective.png (Perspective view)\n");
    printf("  - 3d_visualization_combined.png (All views combined)\n");
    printf("  - 3d_visualization_hq.png (High quality perspective)\n\n");
    
    // Also show the perspective view window
    cv::namedWindow("3D Visualization", cv::WINDOW_NORMAL);
    cv::imshow("3D Visualization", hqPerspective);
}

// REPLACE THE PREVIOUS FUNCTIONS WITH THESE FIXED VERSIONS

// Function to create detailed colorbar with labels
cv::Mat CreateDetailedColorbar(float minZ, float maxZ, int height, int width) {
    cv::Mat colorbar = cv::Mat::zeros(height + 100, width + 150, CV_8UC3);
    
    // Draw the color gradient
    for (int y = 0; y < height; y++) {
        float normalizedDepth = 1.0f - static_cast<float>(y) / height;
        cv::Scalar color = GetDepthColor(normalizedDepth);
        cv::rectangle(colorbar, 
                     cv::Point(20, y + 50), 
                     cv::Point(20 + width, y + 51), 
                     color, -1);
    }
    
    // Add border
    cv::rectangle(colorbar, 
                 cv::Point(19, 49), 
                 cv::Point(21 + width, 51 + height), 
                 cv::Scalar(255, 255, 255), 2);
    
    // Add labels at multiple points
    std::vector<float> labelPositions;
    labelPositions.push_back(0.0f);
    labelPositions.push_back(0.25f);
    labelPositions.push_back(0.5f);
    labelPositions.push_back(0.75f);
    labelPositions.push_back(1.0f);
    
    for (size_t i = 0; i < labelPositions.size(); i++) {
        float pos = labelPositions[i];
        int y = static_cast<int>(50 + pos * height);
        float depth = minZ + (1.0f - pos) * (maxZ - minZ);
        
        // Draw tick mark
        cv::line(colorbar, cv::Point(20 + width, y), cv::Point(30 + width, y), 
                cv::Scalar(255, 255, 255), 2);
        
        // Add text label
        char depthText[20];
        sprintf_s(depthText, sizeof(depthText), "%.1f mm", depth);
        cv::putText(colorbar, depthText, 
                   cv::Point(35 + width, y + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 255, 255), 1);
    }
    
    // Add title
    cv::putText(colorbar, "Distance", 
               cv::Point(20, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, 
               cv::Scalar(255, 255, 255), 2);
    
    return colorbar;
}

// Function to analyze 3D profile and create depth map
ProfileAnalysis Analyze3DProfile(const std::vector<cv::Point3f>& points3D, 
                                const std::string& scanMode, 
                                int NP, float freq) {
    ProfileAnalysis analysis;
    analysis.scanMode = scanMode;
    analysis.NP_value = NP;
    analysis.frequency = freq;
    
    if (points3D.empty()) {
        return analysis;
    }
    
    // Initialize maps
    int mapWidth = camera->GetCameraWidth();
    int mapHeight = camera->GetCameraHeight();
    analysis.depthMap = cv::Mat::zeros(mapHeight, mapWidth, CV_32F);
    analysis.countMap = cv::Mat::zeros(mapHeight, mapWidth, CV_32S);
    analysis.varianceMap = cv::Mat::zeros(mapHeight, mapWidth, CV_32F);
    
    // Temporary storage for variance calculation
    std::vector<std::vector<std::vector<float>>> depthAtPixel(mapHeight, 
    std::vector<std::vector<float>>(mapWidth));
    
    // Find depth range
    analysis.minDepth = FLT_MAX;
    analysis.maxDepth = -FLT_MAX;
    
    // First pass: collect all valid depths and find range
    for (size_t i = 0; i < points3D.size(); i++) {
        const cv::Point3f& pt = points3D[i];
        if (pt.z > 20.0f && pt.z < 5000.0f) {
            analysis.allDepths.push_back(pt.z);
            analysis.minDepth = std::min(analysis.minDepth, pt.z);
            analysis.maxDepth = std::max(analysis.maxDepth, pt.z);
        }
    }
    
    if (analysis.allDepths.empty()) {
        return analysis;
    }
    
    // Calculate statistics
    float sum = 0;
    for (size_t i = 0; i < analysis.allDepths.size(); i++) {
        float depth = analysis.allDepths[i];
        sum += depth;
    }
    analysis.meanDepth = sum / static_cast<float>(analysis.allDepths.size());
    
    float variance = 0;
    for (size_t i = 0; i < analysis.allDepths.size(); i++) {
        float depth = analysis.allDepths[i];
        variance += (depth - analysis.meanDepth) * (depth - analysis.meanDepth);
    }
    analysis.stdDepth = sqrt(variance / static_cast<float>(analysis.allDepths.size()));
    analysis.totalPoints = static_cast<int>(analysis.allDepths.size());
    
    // Second pass: map depths to 2D coordinates
    for (size_t i = 0; i < points3D.size(); i++) {
        const cv::Point3f& pt = points3D[i];
        if (pt.z > 20.0f && pt.z < 5000.0f) {
            // Convert 3D point back to approximate 2D camera coordinates
            float hfov = camera->GetCameraHFOV() * MTI_DEGTORAD;
            float vfov = camera->GetCameraVFOV() * MTI_DEGTORAD;
            
            int x = static_cast<int>((std::atan(pt.x / pt.z) / hfov + 0.5f) * mapWidth);
            int y = static_cast<int>((0.5f - std::atan(pt.y / pt.z) / vfov) * mapHeight);
            
            if (x >= 0 && x < mapWidth && y >= 0 && y < mapHeight) {
                analysis.depthMap.at<float>(y, x) = pt.z;
                analysis.countMap.at<int>(y, x)++;
                depthAtPixel[y][x].push_back(pt.z);
            }
        }
    }
    
    // Calculate variance map
    for (int y = 0; y < mapHeight; y++) {
        for (int x = 0; x < mapWidth; x++) {
            if (depthAtPixel[y][x].size() > 1) {
                float mean = 0;
                for (size_t i = 0; i < depthAtPixel[y][x].size(); i++) {
                    float d = depthAtPixel[y][x][i];
                    mean += d;
                }
                mean /= static_cast<float>(depthAtPixel[y][x].size());
                
                float var = 0;
                for (size_t i = 0; i < depthAtPixel[y][x].size(); i++) {
                    float d = depthAtPixel[y][x][i];
                    var += (d - mean) * (d - mean);
                }
                analysis.varianceMap.at<float>(y, x) = var / static_cast<float>(depthAtPixel[y][x].size());
            }
        }
    }
    
    return analysis;
}

// Function to export comprehensive 3D profile analysis
void ExportComprehensive3DProfile(const std::vector<cv::Point3f>& points3D, 
                                 const std::string& scanMode, 
                                 int NP, float frequency) {
    printf("Exporting comprehensive 3D profile analysis for %s mode, NP=%d, %.1fHz...\n", 
           scanMode.c_str(), NP, frequency);
    
    // Analyze the 3D profile
    ProfileAnalysis analysis = Analyze3DProfile(points3D, scanMode, NP, frequency);
    
    if (analysis.allDepths.empty()) {
        printf("No valid points for analysis!\n");
        return;
    }
    
    // Create filename prefix
    std::stringstream prefix;
    prefix << scanMode << "_NP" << NP << "_" << static_cast<int>(frequency) << "Hz";
    std::string filenamePrefix = prefix.str();
    
    // 1. Create depth map visualization with colorbar
    // REPLACE THE EXISTING DEPTH VISUALIZATION SECTION (around line 2850) WITH THIS:

    // 1. Create HIGH-QUALITY depth map visualization with proper spacing
    cv::Mat depthViz = cv::Mat::zeros(analysis.depthMap.rows, analysis.depthMap.cols, CV_8UC3);
    cv::Mat validMask = cv::Mat::zeros(analysis.depthMap.rows, analysis.depthMap.cols, CV_8UC1);
    
    for (int y = 0; y < analysis.depthMap.rows; y++) {
        for (int x = 0; x < analysis.depthMap.cols; x++) {
            float depth = analysis.depthMap.at<float>(y, x);
            if (depth > 0) {
                float normalizedDepth = (depth - analysis.minDepth) / (analysis.maxDepth - analysis.minDepth);
                cv::Scalar color = GetDepthColor(normalizedDepth);
                depthViz.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    static_cast<uchar>(color[0]),
                    static_cast<uchar>(color[1]),
                    static_cast<uchar>(color[2])
                );
                validMask.at<uchar>(y, x) = 255;
            }
        }
    }
    
    // Create PROFESSIONAL colorbar with proper spacing
    int colorbarWidth = 120;
    int colorbarHeight = 600;
    cv::Mat colorbar = cv::Mat::zeros(colorbarHeight + 250, colorbarWidth + 350, CV_8UC3);
    
    // Draw high-quality color gradient
    for (int y = 0; y < colorbarHeight; y++) {
        float normalizedDepth = 1.0f - static_cast<float>(y) / colorbarHeight;
        cv::Scalar color = GetDepthColor(normalizedDepth);
        cv::rectangle(colorbar, 
                     cv::Point(50, y + 100), 
                     cv::Point(50 + colorbarWidth, y + 101), 
                     color, -1);
    }
    
    // Add thick border for professional look
    cv::rectangle(colorbar, 
                 cv::Point(49, 99), 
                 cv::Point(51 + colorbarWidth, 101 + colorbarHeight), 
                 cv::Scalar(255, 255, 255), 4);
    
    // Add WELL-SPACED distance labels
    std::vector<float> labelPositions = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
    for (float pos : labelPositions) {
        int y = static_cast<int>(100 + pos * colorbarHeight);
        float depth = analysis.minDepth + (1.0f - pos) * (analysis.maxDepth - analysis.minDepth);
        
        // Draw thick tick marks
        cv::line(colorbar, cv::Point(50 + colorbarWidth, y), cv::Point(80 + colorbarWidth, y), 
                cv::Scalar(255, 255, 255), 5);
        
        // Add large, clear text labels
        char depthText[30];
        sprintf_s(depthText, sizeof(depthText), "%.0f mm", depth);
        cv::putText(colorbar, depthText, 
                   cv::Point(90 + colorbarWidth, y + 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.2, 
                   cv::Scalar(255, 255, 255), 3);
    }
    
    // Add large colorbar title
    cv::putText(colorbar, "Distance", 
           cv::Point(50, 40), 
           cv::FONT_HERSHEY_SIMPLEX, 1.4, 
           cv::Scalar(255, 255, 255), 3);
cv::putText(colorbar, "to Surface/Object", 
           cv::Point(50, 75), 
           cv::FONT_HERSHEY_SIMPLEX, 1.4, 
           cv::Scalar(255, 255, 255), 3);
    
    // Create final combined image with GENEROUS spacing
    int totalWidth = depthViz.cols + colorbar.cols + 100;
    int totalHeight = std::max(depthViz.rows + 350, colorbar.rows);
    cv::Mat combined = cv::Mat::zeros(totalHeight, totalWidth, CV_8UC3);
    
    // Add thick border around depth map
    cv::rectangle(combined, cv::Point(39, 149), 
                 cv::Point(depthViz.cols + 41, depthViz.rows + 151), 
                 cv::Scalar(255, 255, 255), 4);
    
    // Copy depth map with proper positioning
    depthViz.copyTo(combined(cv::Rect(40, 170, depthViz.cols, depthViz.rows)));
    
    // Copy colorbar with generous spacing
    colorbar.copyTo(combined(cv::Rect(depthViz.cols + 100, 0, colorbar.cols, colorbar.rows)));
    
    // Add title with NO overlap
std::string title = "3D Depth Profile: " + scanMode;
cv::putText(combined, title, 
           cv::Point(40, 80), cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(255, 255, 255), 3);

// Add parameters on separate line  
std::string params = "NP=" + std::to_string(NP) + ", " + std::to_string(static_cast<int>(frequency)) + "Hz";
cv::putText(combined, params, 
           cv::Point(40, 120), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2);
    
    // Add statistics BELOW the image with generous spacing
    int statsY = depthViz.rows + 200;
    cv::putText(combined, "Points: " + std::to_string(analysis.totalPoints), 
               cv::Point(40, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    cv::putText(combined, "Range: " + std::to_string(static_cast<int>(analysis.minDepth)) + 
                " - " + std::to_string(static_cast<int>(analysis.maxDepth)) + " mm", 
               cv::Point(350, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    
    statsY += 60;
    cv::putText(combined, "Mean: " + std::to_string(static_cast<int>(analysis.meanDepth)) + " mm", 
               cv::Point(40, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    cv::putText(combined, "Std Dev: " + std::to_string(static_cast<int>(analysis.stdDepth)) + " mm", 
               cv::Point(350, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    
    // Calculate coverage
    int validPixels = cv::countNonZero(validMask);
    float coverage = static_cast<float>(validPixels) / static_cast<float>(analysis.depthMap.rows * analysis.depthMap.cols) * 100;
    
    statsY += 60;
    cv::putText(combined, "Coverage: " + std::to_string(static_cast<int>(coverage)) + "%", 
               cv::Point(40, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    float cv = (analysis.stdDepth / analysis.meanDepth) * 100;
    cv::putText(combined, "CV: " + std::to_string(static_cast<int>(cv)) + "%", 
               cv::Point(350, statsY), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    
    // Save HIGH-QUALITY depth map
    cv::imwrite(filenamePrefix + "_depth_profile.png", combined);
    
    // 2. Create point density map
    // 2. Create HIGH-QUALITY point density map
    cv::Mat densityViz = cv::Mat::zeros(analysis.countMap.rows + 200, analysis.countMap.cols + 400, CV_8UC3);
    
    // Find max count for normalization
    int maxCount = 0;
    for (int y = 0; y < analysis.countMap.rows; y++) {
        for (int x = 0; x < analysis.countMap.cols; x++) {
            maxCount = std::max(maxCount, analysis.countMap.at<int>(y, x));
        }
    }
    
    // Create density visualization with proper scaling
    for (int y = 0; y < analysis.countMap.rows; y++) {
        for (int x = 0; x < analysis.countMap.cols; x++) {
            int count = analysis.countMap.at<int>(y, x);
            if (count > 0) {
                float intensity = static_cast<float>(count) / maxCount;
                // Use heat map colors for better visibility
                cv::Scalar color;
                if (intensity < 0.25f) {
                    color = cv::Scalar(intensity * 4 * 255, 0, 255); // Blue to Purple
                } else if (intensity < 0.5f) {
                    color = cv::Scalar(255, 0, (1.0f - intensity) * 4 * 255); // Purple to Red
                } else if (intensity < 0.75f) {
                    color = cv::Scalar(255, (intensity - 0.5f) * 4 * 255, 0); // Red to Yellow
                } else {
                    color = cv::Scalar(255, 255, (intensity - 0.75f) * 4 * 255); // Yellow to White
                }
                
                densityViz.at<cv::Vec3b>(y + 100, x + 40) = cv::Vec3b(
                    static_cast<uchar>(color[0]),
                    static_cast<uchar>(color[1]),
                    static_cast<uchar>(color[2])
                );
            }
        }
    }
    
    // Add border around density map
    cv::rectangle(densityViz, cv::Point(39, 99), 
                 cv::Point(analysis.countMap.cols + 41, analysis.countMap.rows + 101), 
                 cv::Scalar(255, 255, 255), 3);
    
    // Add title and legend
    cv::putText(densityViz, "Point Density Map: " + scanMode, 
               cv::Point(40, 50), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 255, 255), 3);
    
    cv::putText(densityViz, "Max Points per Pixel: " + std::to_string(maxCount), 
               cv::Point(40, analysis.countMap.rows + 150), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
    
    cv::imwrite(filenamePrefix + "_point_density.png", densityViz);
    
    // 3. Export detailed CSV data
    std::ofstream csvFile((filenamePrefix + "_profile_data.csv").c_str());
    if (csvFile.is_open()) {
        csvFile << "x_pixel,y_pixel,depth_mm,point_count,depth_variance\n";
        for (int y = 0; y < analysis.depthMap.rows; y++) {
            for (int x = 0; x < analysis.depthMap.cols; x++) {
                float depth = analysis.depthMap.at<float>(y, x);
                int count = analysis.countMap.at<int>(y, x);
                float variance = analysis.varianceMap.at<float>(y, x);
                if (depth > 0) {
                    csvFile << x << "," << y << "," << depth << "," 
                           << count << "," << variance << "\n";
                }
            }
        }
        csvFile.close();
    }
    
    // 4. Export summary statistics
    std::ofstream summaryFile((filenamePrefix + "_summary.txt").c_str());
    if (summaryFile.is_open()) {
        summaryFile << "3D Profile Analysis Summary\n";
        summaryFile << "===========================\n";
        summaryFile << "Scan Mode: " << scanMode << "\n";
        summaryFile << "NP Value: " << NP << "\n";
        summaryFile << "Frequency: " << frequency << " Hz\n";
        summaryFile << "Total Points: " << analysis.totalPoints << "\n";
        summaryFile << "Depth Range: " << analysis.minDepth << " - " << analysis.maxDepth << " mm\n";
        summaryFile << "Mean Depth: " << analysis.meanDepth << " mm\n";
        summaryFile << "Standard Deviation: " << analysis.stdDepth << " mm\n";
        summaryFile << "Coefficient of Variation: " << (analysis.stdDepth / analysis.meanDepth * 100) << "%\n";
        
        // Calculate coverage statistics
        int validPixels = cv::countNonZero(validMask);
        float coverage = static_cast<float>(validPixels) / static_cast<float>(analysis.depthMap.rows * analysis.depthMap.cols) * 100;
        summaryFile << "Pixel Coverage: " << coverage << "%\n";
        
        summaryFile.close();
    }
    
    printf("Exported comprehensive analysis:\n");
    printf("  - %s_depth_profile.png (depth map with colorbar)\n", filenamePrefix.c_str());
    printf("  - %s_point_density.png (point density map)\n", filenamePrefix.c_str());
    printf("  - %s_profile_data.csv (detailed x,y,z data)\n", filenamePrefix.c_str());
    printf("  - %s_summary.txt (statistical summary)\n", filenamePrefix.c_str());
}

// Function to run systematic NP comparison study
void RunNPComparisonStudy() {
    printf("\n=== Running NP Comparison Study (250 Hz) ===\n");
    printf("This will systematically test NP values from 20 to 100.\n");
    printf("For each NP value, you need to:\n");
    printf("1. Run a scan (option 1)\n");
    printf("2. Return to menu\n");
    printf("3. Press any key when ready for next NP\n\n");
    
    std::vector<int> NP_values;
    NP_values.push_back(20);
    NP_values.push_back(30);
    NP_values.push_back(40);
    NP_values.push_back(50);
    NP_values.push_back(60);
    NP_values.push_back(70);
    NP_values.push_back(80);
    NP_values.push_back(90);
    NP_values.push_back(100);
    
    float frequency = 250.0f;
    
    // Create comparison CSV file
    std::ofstream comparisonFile("NP_comparison_study.csv");
    if (comparisonFile.is_open()) {
        comparisonFile << "NP,Frequency_Hz,Total_Points,Mean_Depth_mm,Std_Depth_mm,CV_percent,Min_Depth_mm,Max_Depth_mm,Coverage_percent\n";
        comparisonFile.close();
    }
    
    for (size_t idx = 0; idx < NP_values.size(); idx++) {
        int NP = NP_values[idx];
        printf("\n=== Testing NP = %d ===\n", NP);
        printf("Setting parameters:\n");
        printf("  NP (numScanLines) = %d\n", NP);
        printf("  Frequency X = %.1f Hz\n", frequency);
        
        // Set parameters
        numScanLines = NP;
        lissajousFreqX = frequency;
        lissajousFreqY = frequency * (NP - 1) / static_cast<float>(NP);
        
        printf("  Frequency Y = %.1f Hz\n", lissajousFreqY);
        printf("\nNow:\n");
        printf("1. Go back to main menu\n");
        printf("2. Set scan mode to Sinusoidal Lissajous (press 'M' until you see it)\n");
        printf("3. Run option 1 (Start Detect Line Demo)\n");
        printf("4. After scan completes, return to this menu\n");
        printf("5. Choose option 4 again to continue\n");
        printf("\nPress any key when you've completed the scan for NP=%d...", NP);
        _getch();
        
        // Analyze results if points3D has data
        if (!points3D.empty()) {
            std::string modeName = "Sinusoidal_Study";
            
            // Export comprehensive analysis
            ExportComprehensive3DProfile(points3D, modeName, NP, frequency);
            
            // Get analysis for comparison
            ProfileAnalysis analysis = Analyze3DProfile(points3D, modeName, NP, frequency);
            
            // Append to comparison file
            std::ofstream comparisonFile("NP_comparison_study.csv", std::ios::app);
            if (comparisonFile.is_open()) {
                float cv = (analysis.stdDepth / analysis.meanDepth) * 100;
                // Calculate coverage
                cv::Mat validMask = cv::Mat::zeros(analysis.depthMap.rows, analysis.depthMap.cols, CV_8UC1);
                for (int y = 0; y < analysis.depthMap.rows; y++) {
                    for (int x = 0; x < analysis.depthMap.cols; x++) {
                        if (analysis.depthMap.at<float>(y, x) > 0) {
                            validMask.at<uchar>(y, x) = 255;
                        }
                    }
                }
                int validPixels = cv::countNonZero(validMask);
                float coverage = static_cast<float>(validPixels) / static_cast<float>(analysis.depthMap.rows * analysis.depthMap.cols) * 100;
                
                comparisonFile << NP << "," 
                              << frequency << ","
                              << analysis.totalPoints << ","
                              << analysis.meanDepth << ","
                              << analysis.stdDepth << ","
                              << cv << ","
                              << analysis.minDepth << ","
                              << analysis.maxDepth << ","
                              << coverage << "\n";
                comparisonFile.close();
            }
            
            printf("Analysis completed for NP=%d\n", NP);
        } else {
            printf("WARNING: No 3D points found for NP=%d. Make sure you ran a scan.\n", NP);
        }
    }
    
    printf("\n=== NP Comparison Study Completed! ===\n");
    printf("Results saved in:\n");
    printf("- Individual analysis files for each NP value\n");
    printf("- NP_comparison_study.csv (summary comparison)\n");
    printf("\nPress any key to return to main menu...");
    _getch();
}






// Show the boundary of the device laser to calculate the Lookup Table limits
void ViewScanAreaBoundary() {
   system(CLEARSCREEN);
   printf("\nCheck selected scan area.");
   printf("\nThe area is based on the Scan Module's calibrated Field of Regard (CFOR) and user's amplitude and offset settings.\n");
   printf("\nHorizontal extent of the scan area of Scan Module is: %3.2fdeg to %3.2fdeg", -camera->GetILUTMaxAngles().x*(xAmplitude + xOffset), +camera->GetILUTMaxAngles().x*(xAmplitude + xOffset));
   printf("\nVertical extent of the scan area of Scan Module is: %3.2fdeg to %3.2fdeg", -camera->GetILUTMaxAngles().y*(yAmplitude + yOffset), +camera->GetILUTMaxAngles().y*(yAmplitude + yOffset));
   printf("\n\nPress any key to return to the menu...\n");

   // We draw a square with the laser of the previous ILUT values
   float xKey[6], yKey[6];
   unsigned char mKey[6];
   const unsigned int refreshRate = 40;
   const unsigned int spf = spsDefault / refreshRate;
   float xSample[spf * 2], ySample[spf * 2];
   unsigned char mSample[spf * 2];
   cv::Point2d maxAng = camera->GetILUTMaxAngles();
   cv::Point2d laser;

   if (camera->IsILUTCalibrated()) {
       // show a rectangle extension of scan area based on angle of the Scan Module
       // and amplitude and offset settings provided by user
       xKey[0] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
       xKey[1] = maxAng.x * xAmplitude + maxAng.x * xOffset;
       xKey[2] = maxAng.x * xAmplitude + maxAng.x * xOffset;
       xKey[3] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
       xKey[4] = -maxAng.x * xAmplitude + maxAng.x * xOffset;
       yKey[0] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
       yKey[1] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
       yKey[2] = maxAng.y * yAmplitude + maxAng.y * yOffset;
       yKey[3] = maxAng.y * yAmplitude + maxAng.y * yOffset;
       yKey[4] = -maxAng.y * yAmplitude + maxAng.y * yOffset;
       mKey[0] = 0xFF;
       mKey[1] = 0xFF;
       mKey[2] = 0xFF;
       mKey[3] = 0xFF;
       mKey[4] = 0xFF;
       // Close the polygon if necessary
       int nKey = datagen->CloseCurve(xKey, yKey, mKey, 5, 1, false);
       int numSamples = datagen->InterpolateData(xKey, yKey, mKey, xSample, ySample, mSample, nKey, spf);

       // Now convert the angles to device coordinates through binlinear interpolation
       for (int i = 0; i < numSamples; i++) {
           if (!camera->BilinearInterpolateILUTForAngle(cv::Point2d(xSample[i], ySample[i]), laser)) {
               xSample[i] = 0;
               ySample[i] = 0;
           }
           else {
               xSample[i] = laser.x;
               ySample[i] = laser.y;
           }
       }
       mti->SendDataStream(xSample, ySample, mSample, numSamples);
       mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
   }

   // Show a yellow rectangle for iLUT boundary
   cv::Point pixelTL = camera->CFORAngleToCamPixel(cv::Point2d(-1 * maxAng.x, maxAng.y));
   cv::Point pixelBR = camera->CFORAngleToCamPixel(cv::Point2d(maxAng.x, -1 * maxAng.y));
   cv::Rect rectLUT = cv::Rect(pixelTL, pixelBR);

   // Start the camera and frame triggering
   camera->SetHardwareTrigger(false);
   camera->StartCamera();
   exposure_time = 25000;
   camera->SetExposureTime(exposure_time);
   cv::namedWindow(windowControl);
   cv::createTrackbar("Exposure [us]", windowControl, &exposure_time, exposure_slider_max, &OnTrackbarChange);
   cv::createTrackbar("Laser Threshold", windowControl, &BWThreshold, threshold_slider_max, OnThresholdChange);

   // Show the camera view while waiting
   int keyPress = -1;
   cv::Mat frame;
   do {
       frame = camera->GetFrame();
       cv::rectangle(frame, rectLUT, cv::Scalar(100, 255, 255));
       cv::imshow(windowCamera, frame);
       int wKey = cv::waitKey(1);
       if (_kbhit() || (wKey != -1))
           keyPress = (wKey != -1) ? wKey : _getch();
   } while (keyPress == -1);
   cv::destroyAllWindows();

   mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
   mti->ResetDevicePosition();
   return;
}

// Setup triangulation with improved parameter calculation
void SetupTriangulation() {
   system(CLEARSCREEN);
   printf("\nSetup Triangulation - Calibrate camera and MEMS parameters\n\n");
   printf("This utility helps configure the triangulation parameters correctly.\n");
   printf("Position a flat object at a known distance for calibration.\n\n");
   
   // We draw a cross with the laser to help with alignment
   float xKey[6], yKey[6];
   unsigned char mKey[6];
   const unsigned int refreshRate = 40;
   const unsigned int spf = spsDefault / refreshRate;
   float xSample[spf * 2], ySample[spf * 2];
   unsigned char mSample[spf * 2];
   cv::Point2d maxAng = camera->GetILUTMaxAngles();
   cv::Point2d laser;

   if (camera->IsILUTCalibrated()) {
       // show a cross in center based on previously available iLUT
       xKey[0] = 0;
       xKey[1] = 0;
       xKey[2] = maxAng.x/4;
       xKey[3] = -maxAng.x/4;
       yKey[0] = -maxAng.y/4;
       yKey[1] = +maxAng.y/4;
       yKey[2] = 0;
       yKey[3] = 0;
       mKey[0] = 0xFF;
       mKey[1] = 0x00;
       mKey[2] = 0xFF;
       mKey[3] = 0xFF;
       // Close the polygon if necessary
       int nKey = datagen->CloseCurve(xKey, yKey, mKey, 4, 1, false);
       int numSamples = datagen->InterpolateData(xKey, yKey, mKey, xSample, ySample, mSample, nKey, spf);

       // Now convert the angles to device coordinates through binlinear interpolation
       for (int i = 0; i < numSamples; i++) {
           if (!camera->BilinearInterpolateILUTForAngle(cv::Point2d(xSample[i], ySample[i]), laser)) {
               xSample[i] = 0;
               ySample[i] = 0;
           }
           else {
               xSample[i] = laser.x;
               ySample[i] = laser.y;
           }
       }
       mti->SendDataStream(xSample, ySample, mSample, numSamples);
       mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
   }

   camera->LoadCameraSettings(cameraSettingsFile);

   // Start the camera and frame triggering
   camera->SetHardwareTrigger(false);
   camera->StartCamera();
   camera->SetExposureTime(exposure_time);
   cv::namedWindow(windowControl);
   cv::createTrackbar("Exposure [us]", windowControl, &exposure_time, exposure_slider_max, OnTrackbarChange);
   cv::createTrackbar("Laser Threshold", windowControl, &BWThreshold, threshold_slider_max, OnThresholdChange);

   // Show the camera view while waiting
   cv::Mat frame;
   bool looping = true, menuFlag = true;
   float angleCorrectionX = camera->GetCameraAngleCorrectionX();
   float planeDistance = camera->GetPlaneDistance();
   float baseDistance = camera->GetBaseDistance();
   
   // If parameters are not set, use some sensible defaults
   if (baseDistance <= 0.0f) baseDistance = 120.0f; // Default to 120mm
   if (planeDistance <= 0.0f) planeDistance = 500.0f; // Default to 500mm
   
   while(looping) {
       if (menuFlag) {
           // Calculate the angle correction based on the triangle formed by
           // the base distance and plane distance
           angleCorrectionX = MTI_RADTODEG * atan(baseDistance/planeDistance);
           
           system(CLEARSCREEN);
           printf("\nSetup Triangulation initial angle - setup camera center to match MEMS line center\n");
           printf("Camera X-Angle Correction: %3.2f degrees\n", angleCorrectionX);
           printf("Distance to Setup Plane: %3.2f mm\n", planeDistance);
           printf("Distance between Camera and Scan Module: %3.2f mm\n", baseDistance);
           printf(" (B/b) increase/decrease base distance of Camera and Scan Module by 0.5mm\n");
           printf(" (N/n) increase/decrease base distance by 5mm\n");
           printf(" (D/d) increase/decrease distance to setup plane by 0.5mm\n");
           printf(" (M/m) increase/decrease plane distance by 50mm\n");
           printf(" (A/a) increase/decrease angle correction directly by 0.1 degree\n");
           printf(" (T) test configuration with a quick scan\n");
           printf(" (S/s)ave camera angle correction\n");
           printf(" (Q/q)uit\n");
           menuFlag = false;
       }
       
       frame = camera->GetFrame();
       
       // Draw crosshairs to help with alignment
       cv::line(frame, cv::Point(camera->GetCameraWidth()/2, 0), 
                cv::Point(camera->GetCameraWidth()/2, camera->GetCameraHeight()), 
                cv::Scalar(100, 255, 255));
       cv::line(frame, cv::Point(0, camera->GetCameraHeight()/2), 
                cv::Point(camera->GetCameraWidth(), camera->GetCameraHeight()/2), 
                cv::Scalar(100, 255, 255));
       
       // Show current parameters on the image
       cv::putText(frame, "Angle: " + std::to_string(angleCorrectionX) + " deg", 
                  cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
       cv::putText(frame, "Base: " + std::to_string(baseDistance) + " mm", 
                  cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
       cv::putText(frame, "Plane: " + std::to_string(planeDistance) + " mm", 
                  cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
       
       cv::imshow(windowCamera, frame);
       
       int wKey = cv::waitKey(1);
       if (_kbhit() || (wKey != -1)) {
           int keyPress = (wKey != -1) ? wKey : _getch();
           switch (keyPress) {
               case 'q':
               case 'Q':
                   looping = false;
                   break;
               case 'B':
                   baseDistance += 0.5f;
                   menuFlag = true;
                   break;
               case 'b':
                   baseDistance = std::max(10.0f, baseDistance - 0.5f);
                   menuFlag = true;
                   break;
               case 'N':
                   baseDistance += 5.0f;
                   menuFlag = true;
                   break;
               case 'n':
                   baseDistance = std::max(10.0f, baseDistance - 5.0f);
                   menuFlag = true;
                   break;
               case 'D':
                   planeDistance += 0.5f;
                   menuFlag = true;
                   break;
               case 'd':
                   planeDistance = std::max(50.0f, planeDistance - 0.5f);
                   menuFlag = true;
                   break;
               case 'M':
                   planeDistance += 50.0f;
                   menuFlag = true;
                   break;
               case 'm':
                   planeDistance = std::max(50.0f, planeDistance - 50.0f);
                   menuFlag = true;
                   break;
               case 'A':
                   angleCorrectionX += 0.1f;
                   menuFlag = true;
                   break;
               case 'a':
                   angleCorrectionX -= 0.1f;
                   menuFlag = true;
                   break;
               case 'T':
               case 't':
                   // Temporarily save parameters for testing
                   camera->SetCameraAngleCorrectionX(angleCorrectionX);
                   camera->SetBaseDistance(baseDistance);
                   camera->SetPlaneDistance(planeDistance);
                   // Hide calibration UI
                   cv::destroyAllWindows();
                   // Perform a quick scan
                   ScanAndDetectLineDemo();
                   // Create windows again
                   cv::namedWindow(windowCamera);
                   cv::namedWindow(windowControl);
                   cv::createTrackbar("Exposure [us]", windowControl, &exposure_time, exposure_slider_max, OnTrackbarChange);
                   cv::createTrackbar("Laser Threshold", windowControl, &BWThreshold, threshold_slider_max, OnThresholdChange);
                   menuFlag = true;
                   break;
               case 's':
               case 'S':
                   camera->SetCameraAngleCorrectionX(angleCorrectionX);
                   camera->SetBaseDistance(baseDistance);
                   camera->SetPlaneDistance(planeDistance);
                   camera->SaveCameraSettings(cameraSettingsFile);
                   printf("\nSettings saved to %s\n", cameraSettingsFile);
                   _getch(); // Wait for keypress to acknowledge save
                   menuFlag = true;
                   break;
           }
       }
   }
   
   cv::destroyAllWindows();

   mti->SetDeviceParam(MTIParam::DigitalOutputEnable, 1);
   mti->ResetDevicePosition();
   return;
}

int main(int argc, char* argv[]) {
   // Open camera
   camera = new MTIPylonCamera();
   if (int error = camera->InitializeCamera()) {
       printf("Couldn't initialize camera. Exiting...\n");
       _getch();
       return -1;
   }
   bool isColorCamera = camera->IsColorCamera();

   // load device related and camera related ini files or exit if not found
   if (!camera->LoadCameraSettings(cameraSettingsFile))
   {
       printf("\n\nUnable to load settings file %s with important camera parameters.\n", cameraSettingsFile);
       printf("Will try to create a new settings file with default parameters.\n");
       // We'll create defaults later when we initialize the MTI device
   }
   if (!camera->LoadDeviceSettings(deviceSettingsFile))
   {
       printf("\n\nUnable to load settings file %s with important device parameters.  Press any key to Exit.\n", deviceSettingsFile);
       camera->ShutdownCamera();
       _getch();
       return -1;                                       // leave demo if not successfully connected
   }

   // Initialize the MTI Device
   mti = new MTIDevice();

   mti->ConnectDevice();
   MTIError lastError = mti->GetLastError();
   if (lastError != MTIError::MTI_SUCCESS)
   {
       printf("\n\nUnable to connect with any device.  Press any key to Exit.\n");
       mti->SendSerialReset();
       camera->ShutdownCamera();
       _getch();
       return -1;                                       // leave demo if not successfully connected
   }
   MTIDeviceParams params;
   mti->GetDeviceParams(&params);       // get current info and parameters from controller
   if (!strstr(params.DeviceName, "MTI-MZ-"))
   {
       printf("\n\nConnected to incompatible Mirrorcle MEMS Controller (Not MTI-MZ-...).  Press any key to Exit.\n");
       mti->SendSerialReset();
       _getch();
       return -1;                                       // leave demo if not successfully connected
   }

   // Force these settings so that saved LUT is not messed up when ini changes
   mti->SetDeviceParam(MTIParam::HardwareFilterBw, camera->GetDeviceHardwareFilterBw());    // over-ride ini file for EaZy2.0 with faster mirror
   mti->SetDeviceParam(MTIParam::VdifferenceMax, camera->GetDeviceVdifferenceMax());
   mti->SetDeviceParam(MTIParam::Vbias, camera->GetDeviceVbias());
   mti->SetDeviceParam(MTIParam::DataScale, 1.0f);
   mti->SetDeviceParam(MTIParam::DataRotation, 0.f);
   mti->SetDeviceParam(MTIParam::OutputOffsets, 0.f, 0.f);
   mti->SetDeviceParam(MTIParam::SampleRate, spsDefault);
   mti->GetDeviceParams(&params);       // get current info and parameters from controller if debugging

   mti->ResetDevicePosition();                 // send analog outputs (and device) back to origin in 25ms
   mti->SetDeviceParam(MTIParam::MEMSDriverEnable, true);        // turn the MEMS Driver on for all examples below

   // MTIDataGenerator object for text, linear raster, interpolation, etc.
   datagen = new MTIDataGenerator();

   // Initialize point cloud scale
   pointCloudScale = 1.0f;

   // Set the default exposure time when using the software trigger
   // Note that the exposure time during laser scan is set by the MTI controller
   // to match the length of the laser scan, and the software exposure time is ignored
   camera->SetExposureTime(exposure_time);

   // If settings file wasn't found, set some default calibration values
   if (!camera->IsILUTCalibrated() || camera->GetBaseDistance() <= 0.0f) {
       printf("\nInitializing default calibration parameters. You should run Setup Triangulation.\n");
       camera->SetBaseDistance(120.0f);  // Default base distance 120mm
       camera->SetPlaneDistance(500.0f); // Default calibration plane at 500mm
       camera->SetCameraAngleCorrectionX(MTI_RADTODEG * atan(120.0f/500.0f)); // Calculate default angle
       camera->SaveCameraSettings(cameraSettingsFile);
       printf("Default parameters saved to %s\n", cameraSettingsFile);
       _getch(); // Wait for keypress
   }

   bool runFlag = true;
   while (runFlag) {
       system(CLEARSCREEN);
       printf("\n************ MTICamera-3DScan 11.1 - 3D Scanning ***************\n");

       printf("\nLoaded mticamera.ini file successfully with following key values:");
       printf("\nCamera field of view (FoV): %3.2f[deg] x %3.2f[deg]", camera->GetCameraHFOV(), camera->GetCameraVFOV());
       printf("\nScan Module calibrated field of regard (CFoR): %3.2f[deg] x %3.2f[deg]", 2 * camera->GetILUTMaxAngles().x, 2 * camera->GetILUTMaxAngles().y);
       printf("\nCamera-MEMS base distance: %3.2f[mm]", camera->GetBaseDistance());
       printf("\nCalibration plane distance: %3.2f[mm]", camera->GetPlaneDistance());
       printf("\nCamera X-angle correction: %3.2f[deg]\n\n", camera->GetCameraAngleCorrectionX());

       printf("\t1: Start Detect Line Demo\n");
       printf("\t2: Setup Triangulation Angles and Parameters\n");
       printf("\t3: View Scan Area Boundary\n");
       printf("\t4: Run NP Comparison Study (250Hz)\n");
       printf("\n\tL(+/-): Increase/decrease number of scan lines: %d", numScanLines);
       printf("\n\tT(+/-): Increase/decrease line duration: %3.2fms", lineDuration * 1000.0f);
       printf("\n\tV/v: Toggle averaging of points along X: %s", xAveraging ? "true" : "false");
       printf("\n\tD/d: Toggle debug info: %s", showDebugInfo ? "on" : "off");
       printf("\n\tM/m: Toggle scan mode (Current: %s)",
    currentScanMode == SCAN_RASTER               ? "Raster" :
    currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS ? "Sinusoidal Lissajous" :
    currentScanMode == SCAN_TRIANGULAR_LISSAJOUS ? "Triangular Lissajous" :
    currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN ? "Bidirectional Cartesian" :
    currentScanMode == SCAN_RADIAL_LISSAJOUS    ? "Radial Lissajous" :
                                                  "Unknown");


       if (currentScanMode != SCAN_RASTER) {
           printf("\n\tF/f: Adjust Lissajous X frequency: %.1f Hz", lissajousFreqX);
           printf("\n\tG/g: Adjust Lissajous Y frequency: %.1f Hz", lissajousFreqY);
           printf("\n\tP/p: Adjust Lissajous phase: %.2f rad", lissajousPhase);
       }
       printf("\n\tE(X/x)it\n");
       printf("\n");

       // Get keypress for menu selection
       int ch = _getch();
       switch (ch) {
           /* Demos */
           case '1':
    ScanAndDetectLineDemo();
    if (showDebugInfo) {
        // Print mode name
        const char* name =
            (currentScanMode == SCAN_RASTER)                ? "Raster" :
            (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS)  ? "Sinusoidal Lissajous" :
            (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS)  ? "Triangular Lissajous" :
                                                             "Unknown";
        printf("Scan completed. Mode was: %s\n", name);
        if (currentScanMode != SCAN_RASTER) {
            printf("Lissajous parameters: FreqX=%.1f, FreqY=%.1f, Phase=%.2f\n",
                   lissajousFreqX, lissajousFreqY, lissajousPhase);
        }
    }
    break;

           case '2':    SetupTriangulation(); break;
           case '3':    ViewScanAreaBoundary(); break;
           case '4':    RunNPComparisonStudy(); break;  // ADD THIS LINE
           case 'V':
           case 'v':
               xAveraging = !xAveraging;
               break;
           case 'D':
           case 'd':
               showDebugInfo = !showDebugInfo;
               break;
           case 'L':
    if (currentScanMode != SCAN_RASTER) {
        if (numScanLines < 15) { // Hardware limit for Lissajous
            numScanLines = std::min(numScanLines + 1, 15U);
            lissajousFreqY = lissajousFreqX * (numScanLines - 1) / float(numScanLines);
            printf("NP=%d, Frequencies: X=%.1f Hz, Y=%.1f Hz\n", 
                   numScanLines, lissajousFreqX, lissajousFreqY);
        } else {
            printf("Cannot increase NP: hardware limit\n");
        }
    } else {
        numScanLines = std::min(numScanLines + 5, maxNumLines);
    }
    break;
case 'l':
    if (currentScanMode != SCAN_RASTER) {
        if (numScanLines > 4) {
            numScanLines = std::max(numScanLines - 1, 4U);
            lissajousFreqY = lissajousFreqX * (numScanLines - 1) / float(numScanLines);
            printf("NP=%d, Frequencies: X=%.1f Hz, Y=%.1f Hz\n", 
                   numScanLines, lissajousFreqX, lissajousFreqY);
        }
    } else {
        numScanLines = std::max(numScanLines - 5, 5U);
    }
    break;
           case 'T':
               lineDuration = std::min(lineDuration + 0.001f, 0.05f);
               break;
           case 't':
               lineDuration = std::max(lineDuration - 0.001f, 0.001f);
               break;
           case 'M':
case 'm':
    // Cycle through all five modes:
    switch (currentScanMode) {
        case SCAN_RASTER:
            currentScanMode = SCAN_SINUSOIDAL_LISSAJOUS;
            break;
        case SCAN_SINUSOIDAL_LISSAJOUS:
            currentScanMode = SCAN_TRIANGULAR_LISSAJOUS;
            break;
        case SCAN_TRIANGULAR_LISSAJOUS:
            currentScanMode = SCAN_BIDIRECTIONAL_CARTESIAN;
            break;
        case SCAN_BIDIRECTIONAL_CARTESIAN:
            currentScanMode = SCAN_RADIAL_LISSAJOUS;
            break;
        case SCAN_RADIAL_LISSAJOUS:
            currentScanMode = SCAN_RASTER;
            break;
        default:
            currentScanMode = SCAN_RASTER;
            break;
    }

    // If we’re now in any Lissajous (or Bidirectional/Radial) mode, reset frequencies
    if (currentScanMode != SCAN_RASTER) {
        lissajousFreqX = baseFreqX;
        lissajousFreqY = baseFreqX * (numScanLines - 1) / float(numScanLines);
        printf(
            "Mode: %s, NP=%d, fX=%.1f Hz, fY=%.1f Hz\n",
            (currentScanMode == SCAN_SINUSOIDAL_LISSAJOUS) ? "Sinusoidal" :
            (currentScanMode == SCAN_TRIANGULAR_LISSAJOUS) ? "Triangular" :
            (currentScanMode == SCAN_BIDIRECTIONAL_CARTESIAN) ? "Bidirectional" :
            (currentScanMode == SCAN_RADIAL_LISSAJOUS)    ? "Radial" :
                                                            "Unknown",
            numScanLines, lissajousFreqX, lissajousFreqY
        );
    }
    break;


           case 'F':
    if (currentScanMode != SCAN_RASTER) {
        if (lissajousFreqX < 200.0f) { // Hardware safety limit
            lissajousFreqX += 5.0f;
            lissajousFreqY = lissajousFreqX * (numScanLines - 1) / float(numScanLines);
            printf("Frequencies: X=%.1f Hz, Y=%.1f Hz\n", lissajousFreqX, lissajousFreqY);
        } else {
            printf("Cannot increase: hardware limit reached\n");
        }
    }
    break;
case 'f':
    if (currentScanMode != SCAN_RASTER) {
        if (lissajousFreqX > 20.0f) { // Minimum useful frequency
            lissajousFreqX -= 5.0f;
            lissajousFreqY = lissajousFreqX * (numScanLines - 1) / float(numScanLines);
            printf("Frequencies: X=%.1f Hz, Y=%.1f Hz\n", lissajousFreqX, lissajousFreqY);
        }
    }
    break;
           case 'G':
               if (currentScanMode != SCAN_RASTER) {
                   lissajousFreqY = std::min(lissajousFreqY + 1.0f, 100.0f);
               }
               break;
           case 'g':
               if (currentScanMode != SCAN_RASTER) {
                   lissajousFreqY = std::max(lissajousFreqY - 1.0f, 1.0f);
               }
               break;
           case 'P':
               if (currentScanMode != SCAN_RASTER) {
                   lissajousPhase = std::min(lissajousPhase + 0.1f, static_cast<float>(2 * M_PI));
               }
               break;
           case 'p':
               if (currentScanMode != SCAN_RASTER) {
                   lissajousPhase = std::max(lissajousPhase - 0.1f, 0.0f);
               }
               break;
           case 'x':
           case 'X':
           case 'e':
           case 'E':
               runFlag = false;
               break;
           default:    break;
       }
   }

   // 4. ADD TO YOUR MAIN() FUNCTION, at program exit:
if (masterFileOpen) {
    masterFile.close();
    printf("✓ All data saved to all_scans_comparison.csv\n");
}

   // Clean up resources
   delete datagen;
   
   // Shut down the camera and any open windows
   camera->ShutdownCamera();
   delete camera;
   
   cv::destroyAllWindows();

   // End the program by returning the device to origin
   mti->ResetDevicePosition(); // send analog outputs (and device) back to origin in 25ms
   mti->SetDeviceParam(MTIParam::MEMSDriverEnable, false);       // turn off the MEMS driver
   mti->DisconnectDevice();
   delete mti;
   
   return 0;
}