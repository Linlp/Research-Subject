#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
//#define M_PI 3.1415926
float toRadians(float x)
{
    return M_PI * x / 180;
}

struct  output{
    int height;
    int width;
    
};
int main()
{
   // char output_filename[256];
    struct output outputSize={1920,3840};
    // outputSize.height=1920;

    Mat imgSize=imread("1.jpg");
    Mat warpMat(1920,3840,CV_32FC2);
    Mat warp;
    bool isFishEye = true;
    //fovHL、fovHR 不确定
    int fovHL = 180;//
    int fovVL = 180;// 180 or 360
    int fovHR = 180;//
    int fovVR = 180;
    int maxFovH = 220;
    int maxFovV = 220;
    int widthL = 1920;//imgSize.cols/2;//output 一半
    int widthR = 1920;//imgSize.cols/2;
    float degreeX = 0.0; 
    float degreeY = 0.0;
    float offsetX = -25;
    float offsetY = 54;
    float effectiveR =1023;
    int YScale =1;
    float calib[6]={0.0, -0.2554, 0.0, 0.0498, 0.0, -0.0037};
    int cam_id =1;
 
    const float maxFovHRad = toRadians(maxFovH);
    const float maxFovVRad = toRadians(maxFovV);
    const float fovHLRad = toRadians(fovHL);
    const float fovVLRad = toRadians(fovVL);
    const float fovHRRad = toRadians(fovHR);
    const float fovVRRad = toRadians(fovVR);
    const float degreeXRad = toRadians(degreeX);
    const float degreeYRad = toRadians(degreeY);
    
    // Fisheye
    for (int y = 0; y < outputSize.height && isFishEye; ++y) {
        
        // Left part
        for (int x = 0; x < widthL; ++x) {
            //图像坐标转换极坐标
            const float theta = -fovHLRad * float(x - widthL) / widthL + M_PI / 2.0f - degreeXRad;
            const float phi = fovVLRad * float(y) / float(outputSize.height) + (M_PI - fovVLRad) / 2.0f + degreeYRad;
            //极坐标转换3D 坐标系
            const float xSphere = cos(theta) * sin(phi);
            const float ySphere = sin(theta) * sin(phi);
            const float zSphere = cos(phi);
            //重投影
            const float theta2 = atan2(-zSphere, xSphere);
            const float phi2 = acos(ySphere);
            const float r = phi2 / (maxFovHRad / 2.0f);
            //投影在鱼眼像素坐标
            const float dist = 1 + calib[0] * r + calib[1] * pow(r, 2) + calib[2] * pow(r, 3)
            + calib[3] * pow(r, 4) + calib[4] * pow(r, 5) + calib[5] * pow(r, 6);
            const float imgSizeX = imgSize.cols / 2 + effectiveR * r * dist * cos(theta2) + offsetX;
            const float imgSizeY = imgSize.rows / 2 + effectiveR * YScale * r * dist * sin(theta2) + offsetY;
            warpMat.at<Point2f>(y, x) = Point2f(imgSizeX, imgSizeY);
        }
   // Right part
        for (int x = widthL; x < outputSize.width; ++x) {
            const float theta = -fovHRRad * float(x - widthL) / widthR + M_PI / 2.0f - degreeXRad;
            const float phi = fovVRRad * float(y) / float(outputSize.height) + (M_PI - fovVRRad) / 2.0f + degreeYRad;
            const float xSphere = cos(theta) * sin(phi);
            const float ySphere = sin(theta) * sin(phi);
            const float zSphere = cos(phi);
            const float theta2 = atan2(-zSphere, xSphere);
            const float phi2 = acos(ySphere);
            const float r = phi2 / (maxFovHRad / 2.0f);
            const float dist = 1 + calib[0] * r + calib[1] * pow(r, 2) + calib[2] * pow(r, 3)
            + calib[3] * pow(r, 4) + calib[4] * pow(r, 5) + calib[5] * pow(r, 6);
            const float srcX = imgSize.cols / 2 + effectiveR * r * dist * cos(theta2) + offsetX;
            const float srcY = imgSize.rows / 2 + effectiveR * YScale * r * dist * sin(theta2) + offsetY;
            warpMat.at<Point2f>(y, x) = Point2f(srcX, srcY);
        }
    

    // sprintf(output_filename, "base_%d.png", params->output_cnt, cam_id);
    // imwrite(params->output_dir + output_filename, warp);
    }
    remap(imgSize, warp, warpMat, Mat(), CV_INTER_CUBIC, BORDER_CONSTANT, Scalar(0, 0, 0, 0));
    //cvtColor(warp, warp, CV_BGR2BGRA);
    imwrite("output.jpg", warp);
}