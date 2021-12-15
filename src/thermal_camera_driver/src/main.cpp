#include "MagDevice.h"
#include "MagService.h"
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include <thread>
using namespace std;
using namespace cv;

image_transport::Publisher pubThermalImage;
char keyBoard;
cv::Mat result;
string fileSavePath = "/home/raymond/";
int counts = 0;
uchar gamma_0[256];
const UCHAR *pData = NULL;
const BITMAPINFO *pInfo = NULL;
bool needPub = false;

// void GammaCorrection(Mat *src_address, Mat LUT_index)
// {
//     // build look up table
//     Mat src = *src_address;
//     Mat dst;
//     LUT(src, LUT_index, dst);
//     *src_address = dst;
// }

// void gamma_vector_creat(uchar *gamma_i, double gamma)
// {
//     double anti_gamma = 1 / gamma;
//     for (int i = 0; i <= 255; i++)
//     {
//         *(gamma_i + i) = (pow(double((i + 0.5) / 256), anti_gamma) * 256 - 0.5);
//     }
// }

void NewFrame(UINT intChannelIndex, int intCameraTemperature, DWORD dwFFCCounterdown,
              DWORD dwCamState, DWORD dwStreamType, ULONG dwUser)
{
    CMagDevice *pDev = (CMagDevice *)dwUser;
    if (dwStreamType == STREAM_HYBRID || dwStreamType == STREAM_VIDEO)
    {

        BOOL bVideoStream = MAG_GetOutputVideoData(intChannelIndex, &pData, &pInfo);
        if (!bVideoStream)
        {
            cout << "获取视频图像数据失败. " << endl;
            needPub = false;
        }
        needPub = true;
    }
}

void pubCameraThread()
{
    ros::Rate rate(200);
    while (ros::ok())
    {
        if (needPub)
        {
            cv::Mat Image(pInfo->bmiHeader.biHeight, pInfo->bmiHeader.biWidth, CV_8UC3);
            auto it = Image.begin<cv::Vec3b>();
            auto itEnd = Image.end<cv::Vec3b>();
            for (; it != itEnd; it++)
            {
                (*it).val[0] = *pData;
                (*it).val[1] = *(pData + 1);
                (*it).val[2] = *(pData + 2);
                pData += 3;
            }
            cv::flip(Image, result, 0);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
            msg->header.stamp=ros::Time::now();
            pubThermalImage.publish(msg);
            needPub = false;
        }

        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    int countss = 0;
    image_transport::ImageTransport it(nh);
    pubThermalImage = it.advertise("/camera/image_raw", 1);

    ROS_INFO("\033[1;32m---->\033[0m Thermal_camera_driver Started.");

    // if (argc < 2)
    // {
    //     cout << "[ERR] 需要指定一个可用相机的IP." << endl;
    //     return -1;
    // }

    CMagService service;
    if (!service.IsInitialized())
    {
        cout << "[ERR] 相机服务初始化失败." << endl;
        return -1;
    }
    service.EnableAutoReConnect(TRUE); //开启断线重连

    CMagDevice device;
    if (!device.IsInitialized())
    {
        cout << "[ERR] 相机设备初始化失败." << endl;
        return -1;
    }

    char *Ip = "192.168.1.112";
    // if (!device.LinkCamera(argv[1]))
    if (!device.LinkCamera(Ip))
    {
        cout << "[ERR] 连接相机失败." << endl;
        return -1;
    }

    const struct_CamInfo *pCamInfo = device.GetCamInfo();
    int intMaxFPS = int(pCamInfo->intMaxFPS);
    int intCurFPS = pCamInfo->intCurrentFPS;
    cout << "Max FPS: " << intMaxFPS << endl;
    cout << "Cur FPS: " << intCurFPS << endl;

    if (!pCamInfo)
    {
        cout << "[ERR] 获取相机参数失败." << endl;
        return -1;
    }

    OutputPara paraOut;
    paraOut.dwFPAWidth = pCamInfo->intFPAWidth;
    paraOut.dwFPAHeight = pCamInfo->intFPAHeight;
    paraOut.dwBMPWidth = pCamInfo->intVideoWidth;
    paraOut.dwBMPHeight = pCamInfo->intVideoHeight;
    paraOut.dwColorBarWidth = 16;
    // paraOut.dwColorBarWidth = 0;
    paraOut.dwColorBarHeight = pCamInfo->intVideoHeight;

    device.SetColorPalette(RainBow); //设置图像颜色样式 Gray0to255 IronBow
    // device.SetColorPalette(Gray0to255); //设置图像颜色样式 Gray0to255 IronBow


    // if (!device.StartProcessImage(&paraOut, NewFrame, STREAM_VIDEO, (ULONG)&device))
    if (!device.StartProcessImage(&paraOut, NewFrame, STREAM_HYBRID, (ULONG)&device))
    {
        cout << "[ERR] 传输数据失败." << endl;
        return -1;
    }
    cout << "[MSG] ^_^ 相机正常工作中..." << endl;
    std::thread pubCarema(pubCameraThread);
    ros::Rate rate(200);
    cout << "start" << endl;
    while (nh.ok())
    {
        if (result.cols > 100)
        {
        cv::imshow("result", result);

        }
        char key_board = cv::waitKey(10);
        if (key_board == 's')
        {
            stringstream ss;
            ss << "/home/raymond/result/" << countss << ".jpg";
            cout << "save" << endl;
            cv::imwrite(ss.str(), result);
            countss++;
        }
        ros::spinOnce();
        rate.sleep();
    }

    device.StopProcessImage();
    device.DisLinkCamera();
    ros::spin();
    return 0;
}

// enum ColorPalette
// {
//     Gray0to255 = 0,//黑白
//     Gray255to0 = 1,
//     IronBow = 2,
//     RainBow = 3,
//     GlowBow = 4,
//     Autumn = 5,
//     Winter = 6,
//     HotMetal = 7,
//     Jet = 8,
//     RedSaturation = 9,
//     HighContrast = 10,
//     IronBow2 = 11,
//     Blue2Red = 12,
// };
