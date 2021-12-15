#ifndef _THERMOGROUPSDK_H_
#define _THERMOGROUPSDK_H_

#define WINAPI
#define MAG_API

typedef unsigned int DWORD;
typedef unsigned long ULONG;
typedef long LONG;
typedef int BOOL;
typedef unsigned short WORD;
typedef unsigned int UINT;
typedef signed int INT;
typedef unsigned short USHORT;
typedef unsigned char BYTE;
typedef unsigned char UCHAR;
typedef void* LPVOID;
typedef void* HWND;

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif
#define TRUE 1
#define FALSE 0

#define CAMNAME_PROTOCOLLEN	(32)
#define FPANAMELEN		(32)
#define TYPENAMELEN		(8)
#define DACOUTWAYS		(4)

#define STREAM_TEMPERATURE  (2)
#define STREAM_VIDEO        (4)
#define STREAM_HYBRID       (STREAM_TEMPERATURE | STREAM_VIDEO)

#define MAX_DEVICE      (128)

typedef struct {
    DWORD dwFPAWidth;
    DWORD dwFPAHeight;
    DWORD dwBMPWidth;
    DWORD dwBMPHeight;
    DWORD dwColorBarWidth;
    DWORD dwColorBarHeight;
} OutputPara;

struct struct_State
{
    int pad0[6];
    int intMaxTemperature;//内部/mC, 实际温度范围
    int intMinTemperature;//内部/mC
    int intAveTemperature;//内部/mC
    int intSTDTemperature;//内部/mC, LITEVERSION不计算
    UINT intPosMax;//温度极值出现位置
    UINT intPosMin;
    int pad1[3];
    UINT intAveNETDt;//内部/mC 像面时域NETD平均值
    int pad2[5];
    UINT intHistTemperature[256];//温度图像直方图
};

enum ColorPalette
{
    Gray0to255 = 0,//黑白
    Gray255to0 = 1,
    IronBow = 2,
    RainBow = 3,
    GlowBow = 4,
    Autumn = 5,
    Winter = 6,
    HotMetal = 7,
    Jet = 8,
    RedSaturation = 9,
    HighContrast = 10,
    IronBow2 = 11,
    Blue2Red = 12,
};

enum EX
{
    E1X = 0,
    E2X = 1,
    E4X = 2,
    E8X = 3,
    E16X = 4,
};

typedef struct {
    UINT intFPAWidth; //探测器像素
    UINT intFPAHeight;

    int pad[2];

    char charName[CAMNAME_PROTOCOLLEN];
    char charType[TYPENAMELEN];

    UINT intMaxFPS;     //本型号的最高帧率
    UINT intCurrentFPS; //当前实际输出帧率

    UINT intVideoWidth; //(以HDMI, H.264或MPEG输出的)数字视频像素
    UINT intVideoHeight;
} struct_CamInfo;

typedef struct
{
    struct_CamInfo BaseInfo;

    UINT intCameraSN;//热像仪序列号

    int intCamTemperature[4];//外壳温度，探测器温度，保留，保留

    char charLensName[32];//所使用的镜头名
    float fFocalLength;//m
    int	intCaliBlackbodyRange[2];//标定黑体温度范围

    DWORD dwReserved0[10];

    long long timeCurrent;//帧生成时间, 64bit time_t

    double dblLatitude;			//纬度, 度，北纬为正
    double dblLongitude;		//经度, 度，东经为正
    float fAltitude;			//高度, m

    int intPaletteIndex;            //用户设置的调色板序号
    int intTempUnit;				//用户设置的温度单位。0: 摄氏度；1：华氏度
    float fEmissivity;              //用户设置的发射率
    float fEnvTemp;                 //用户设置的环境和反射温度
    float fTaoAtm;                  //用户设置的大气透过率
    float fTaoFilter;               //用户设置的(热像仪外部的)窗口透过率
    float fObjDist;                 //用户设置的目标距离 m
    BOOL bSubSectionEnlarge;		//是否启用分段拉伸
    int intEnlargeX1;               //分段拉伸参数，t1, mC
    int intEnlargeX2;               //分段拉伸参数，t2, mC
    UINT byteEnlargeY1;             //分段拉伸参数，gray1
    UINT byteEnlargeY2;				//分段拉伸参数，gray2
    UINT intAutoEnlargeRange;       //自动拉伸范围，C
    int intBrightOffset;            //自动拉伸亮度微调，-100~100
    int intContrastOffset;          //自动拉伸对比度微调，-100~100

    DWORD dwReserved1[32];
} struct_CamInfoEx;

typedef struct tagRGBQUAD {
    BYTE    rgbBlue;
    BYTE    rgbGreen;
    BYTE    rgbRed;
    BYTE    rgbReserved;
} RGBQUAD;

#ifdef __ANDROID__
enum ImageEnhancementMethod {
	AUTO_ENLARGE = 0,//default
	MANUAL_ENLARGE = 1,
	ISOTHERMAL = 2
};

union ImageEnhancementParam {
	struct {
		int range;
		int brightness;
		int contrast;

	}AutoEnlarge;

	struct {
		int lowerLimitTemp;
		int upperLimitTemp;
		int grayScale1;//0~255
		int grayScale2;//0~255
	}ManualEnalrge;

	struct {
		int lowerLimitTemp;
		int upperLimitTemp;
	}ISOThermal;
};
#endif

typedef struct tagBITMAPINFOHEADER{
    DWORD      biSize;
    INT        biWidth;
    INT        biHeight;
    WORD       biPlanes;
    WORD       biBitCount;
    DWORD      biCompression;
    DWORD      biSizeImage;
    INT        biXPelsPerMeter;
    INT        biYPelsPerMeter;
    DWORD      biClrUsed;
    DWORD      biClrImportant;
} BITMAPINFOHEADER;

#pragma pack(push, 2)
typedef struct tagBITMAPFILEHEADER {
    WORD    bfType;
    DWORD   bfSize;
    WORD    bfReserved1;
    WORD    bfReserved2;
    DWORD   bfOffBits;
} BITMAPFILEHEADER;
#pragma pack(pop)

typedef struct tagBITMAPINFO {
    BITMAPINFOHEADER    bmiHeader;
    RGBQUAD             bmiColors[1];
} BITMAPINFO;

typedef struct tagPOINT
{
    INT  x;
    INT  y;
} POINT;

typedef struct tagRECT
{
    INT    left;
    INT    top;
    INT    right;
    INT    bottom;
} RECT;

struct struct_TerminalList
{
    char	charTerminalName[32];
    UINT	intVersion;
    UINT	intTerminalIp;
    UINT	intControllerIp;
    BYTE	charCameraMAC[6];
    char    pad0[2];
    UINT	pad1[2];
};

enum enumInputIo
{
    IoFFC = 0,
    IoCaptureMGT,
    IoCaptureBMP,
    IoCustom,
};


enum enumAnalogPlot
{
    AnalogPlotNone = 0,
    AnalogPlotCenterCross,
    AnalogPlotMaxTemperature,
    AnalogPlotROI,
};

struct struct_CeRegContent
{
    DWORD pad0;

    char charName[CAMNAME_PROTOCOLLEN];

    BOOL bUseStaticIp;
    DWORD dwStaticIp;
    DWORD dwStaticNetMask;

    BOOL bMulticastImg;
    DWORD dwMulticastIp;

    DWORD pad1[16];

    DWORD dwSN;//序列号
    
    DWORD pad2[2];

    DWORD dwStaticGateWay;//静态网关

    DWORD pad3[8];

    UINT intCurrentLensIndex;

    DWORD pad4[2];

    UINT intFFCFrameTrigger;
    UINT intFFCTemperatureTrigger;//mC

    DWORD pad5;

    UINT intAccResponse;//高频响应平均帧数

    DWORD pad6[3];

    enum enumInputIo InputIoFunction;//输入Io功能

    //Analog Output
    UINT intPaletteIndex;
    BOOL bColorBar;
    BOOL bSubSectionEnlarge;
    int intEnlargeX1;//mC temperature
    int intEnlargeX2;//mC
    UINT byteEnlargeY1;//gray
    UINT byteEnlargeY2;
    UINT intAutoEnlargeRange;//C
    enum enumAnalogPlot AnalogPlot;//模拟视频绘制

    //Alarm
    int	intAlarmTemp;//IO报警全局温度

    UINT intTVStandard;//模拟视频制式

    BOOL bCheckHeartBeat;//心跳检测
    BOOL bAlwaysAnalogOutput;

    DWORD pad7[3];

    UINT intEX;//模拟视频电子倍焦，0-none, 1-2X, 2-4X，历史原因未集中放置

    DWORD dwPartnerVisibleIp;//搭档的可见光相机IP，历史原因未集中放置

    UINT intDetailRatio;//模拟输出DDE强度，历史原因未集中放置

    DWORD pad9;

    DWORD dwSerialBaudRate;//串口波特率
    DWORD dwSerialFeature;//串口参数，格式为 (使能<<24 | 数据位 | 停止位 | 校验位)
};

struct struct_RectROI
{
    char charROIName[32];//ROI名称
    int x0;//以左下角为原点，x0必须小于x1
    int y0;//y0必须小于y1
    int x1;
    int y1;
    int intEmissivity;//发射率*100，例如90代表0.9
    int intAlarmTemp;//报警温度，单位mC，报警方式为IO输出和画面上数字闪烁
    DWORD dwDraw;//显示选项
    DWORD dwReserved[9];
};

struct struct_IrregularROI
{
    char charROIName[32];//ROI名称, UTF8
    int intRoiType;//0-point, 1-line, 2-rect, 3-circle, 4-ellipse, 5-polygon, 6-delta, 7-delta3
    int x0;//外接矩形，以左下角为原点，x0必须小于x1
    int y0;//y0必须小于y1
    int x1;
    int y1;
    int intEmissivity;//发射率*100，例如90代表0.9
    int intAlarmTemp;//报警温度，单位mC，报警方式为IO输出和画面上数字闪烁
    int intTextPos;//0-auto, 1-center, 2-left, 3-right, 4-top, 5-bottom
    UINT intSamplePeriod;//采样周期(帧)
    UINT intPtNumber;//有效控制点数量
    POINT Points[4];//用户输入控制点
    DWORD dwReserved[8];
};

#define MAX_RECT_ROI_NUM	(4)

struct struct_UserROIs
{
    UINT intValidRectROI;//有效rect ROI数量
    struct_RectROI ROI[MAX_RECT_ROI_NUM];
};

#define MAX_RECT_ROI_NUM2	(12)

struct struct_RectROIReport
{
    char charROIName[32];//ROI名称, UTF8
    int x0;//ROI外接矩形，以左下角为原点
    int y0;
    int x1;
    int y1;
    BOOL bAlarm;//是否触发了报警
    int intMinTemp;//单位mC
    int intMaxTemp;//单位mC
    int intAveTemp;//单位mC
    int intMaxPos;
    int intAlarmThreshold[2];//单位mC
    int intTestPos;//0-auto, 1-center, 2-left, 3-right, 4-top, 5-bottom
    int intRoiType;//0-point, 1-line, 2-rect, 3-circle, 4-ellipse, 5-polygon, 6-delta, 7-delta3
    DWORD dwReserved[3];
};

struct struct_FixPara
{
    float fDistance;//目标距离 m
    float fEmissivity;//缺省发射率
    float fTemp;//气温 C
    float fRH;//相对湿度
    float fVisDistance;//能见度 km
    float fRain;//降雨强度 mm/h
    float fSnow;//降雪强度 mm/h
    float fExtrapara1;//补充修正参数1, 用于修正平均视场温度对测温点的影响
    float fExtrapara2;//补充修正参数2
    float fTaoAtm;//大气透过率
    float fTaoFilter;//窗口/滤光片透过率
};


enum PTZProtocol
{
    ProtocolPelcoD = 0,
    ProtocolPelcoP,
};

enum PTZCmd
{
    PTZStop = 0,//方位或预置位停止 参数为0
    PTZRight = 1,//方位 参数为运动速度0~63
    PTZLeft,
    PTZUp,
    PTZDown,
    PTZUpRight = 5,
    PTZUpLeft,
    PTZDownRight,
    PTZDownLeft,
    PTZSetPreset = 9,//预置位 参数为预置位编号0~255
    PTZCallPreset,
    PTZClearPreset,
    PTZSetAuxiliary = 12,//辅助开关 参数为辅助开关编号0~255
    PTZClearAuxiliary,
    PTZZoomStop = 14,//镜头停止变倍 参数为0
    PTZZoomIn,//镜头放大 参数为马达运动时间
    PTZZoomOut,//镜头缩小
    PTZFocusStop = 17,//镜头停止调焦 参数为0
    PTZFocusAuto,//镜头自动对焦 参数为0
    PTZFocusFar,//镜头看远 参数为马达运动时间
    PTZFocusNear,//镜头看近
    PTZFocusGoto,//镜头调焦绝对位置 参数为调焦位置
    PTZZoomGoto,//镜头变倍绝对位置 参数为变倍位置
    PTZPanGoto,//云台绝对位置
    PTZTiltGoto,
    PTZPanGotoRelative,//云台相对位置
    PTZTiltGotoRelative,

    PTZQueryPan2 = 1000,//方便相机端代码编写，客户端不用
    PTZQueryTilt2 = 1001,
};

enum PTZQuery
{
    PTZQueryPan = 0,//查询云台上下角度
    PTZQueryTilt,//查询云台左右角度
    PTZQueryZoomPosition,//查询zoom绝对位置
    PTZQueryZoomState,//查询是否正在zoom
    PTZQueryFocusPosition,//查询focus绝对位置
    PTZQueryFocusState,//查询是否正在执行自动对焦
};

// callback function for new frame arrival
typedef void (*MAG_FRAMECALLBACK)(UINT intChannelIndex,
                                  int intCameraTemperature,
                                  DWORD dwFFCCounterdown, DWORD dwCamState,
                                  DWORD dwStreamType, ULONG lUserData);

//callback function for serial receive
typedef void (*MAG_SERIALCALLBACK)(UINT intChannelInde, void* pData,
                                   UINT intDataLen, void* pUserData);

//callback function for re-connect operation
typedef void (*MAG_RECONNECTCALLBACK)(UINT intChannelIndex,
                                      UINT intRecentHeartBeatTick,
                                      int intState, void * pUserData);

//callback function for ROI report (Set by MAG_SetUserROIs() or MAG_SetUserROIs2() )
typedef void (* MAG_ROICALLBACK)(UINT intChannelIndex, struct struct_RectROIReport * pReports, UINT intROINum, void * pUserData);

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __ANDROID__
MAG_API void WINAPI MAG_SetStorageDir(const char* sDir);
#endif

//----------------------------------------------------
MAG_API BOOL WINAPI MAG_NewChannel(UINT intChannelIndex);

MAG_API void WINAPI MAG_DelChannel(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_IsChannelAvailable(UINT intChannelIndex);

//----------------------------------------------------
MAG_API BOOL WINAPI MAG_IsLanConnected();

MAG_API DWORD WINAPI MAG_GetLocalIp();

MAG_API void WINAPI MAG_SetFilter(UINT intFilter);

MAG_API void WINAPI MAG_EnableAutoReConnect(BOOL bEnable);

MAG_API BOOL WINAPI MAG_EnumCameras();

MAG_API DWORD WINAPI
    MAG_GetTerminalList(struct_TerminalList* pList, DWORD dwBufferSize);

MAG_API BOOL WINAPI
    MAG_GetMulticastState(UINT intTargetIp, UINT* intMulticastIp,
                          UINT* intMulticastPort, UINT intTimeoutMS);

//----------------------------------------------------
MAG_API BOOL WINAPI MAG_IsInitialized(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_Initialize(UINT intChannelIndex, HWND hWndMsg);

MAG_API void WINAPI MAG_Free(UINT intChannelIndex);

//----------------------------------------------------

MAG_API BOOL WINAPI MAG_IsLinked(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_LinkCamera(UINT intChannelIndex, UINT intIP, UINT intTimeoutMS);

MAG_API BOOL WINAPI MAG_LinkCamera_usb(UINT intChannelIndex, UINT intId, UINT intTimeoutMS);

MAG_API BOOL WINAPI MAG_LinkCameraEx(UINT intChannelIndex, UINT IndexOrIP, USHORT shortPort,
                                     const char * charCloudUser, const char * charCloudPwd, UINT intCamSN,
                                     const char * charCamUser, const char * charCamPwd, UINT intTimeoutMS);

MAG_API void WINAPI MAG_DisLinkCamera(UINT intChannelIndex);

MAG_API UINT WINAPI MAG_GetRecentHeartBeat(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_IsListening(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_ListenTo(UINT intChannelIndex, UINT intTargetIp);

MAG_API void WINAPI MAG_StopListen(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SetReConnectCallBack(UINT intChannelIndex,
                                             MAG_RECONNECTCALLBACK pCallBack,
                                             void * pUserData);

MAG_API BOOL WINAPI MAG_ResetCamera(UINT intChannelIndex);

MAG_API void WINAPI MAG_GetCamInfo(UINT intChannelIndex, struct_CamInfo* pInfo, UINT intSize);

MAG_API void WINAPI MAG_GetCamInfoEx(UINT intChannelIndex, struct_CamInfoEx * pInfo, UINT intSize);

MAG_API BOOL WINAPI MAG_GetCameraTemperature(UINT intChannelIndex, int intT[4],
                                             UINT intTimeoutMS);

MAG_API BOOL WINAPI MAG_TriggerFFC(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_ReadCameraRegContent(UINT intChannelIndex,
                                             struct_CeRegContent* pContent,
                                             UINT intTimeoutMS,
                                             BOOL bReadDefaultValue);

MAG_API BOOL WINAPI
    MAG_SetCameraRegContent(UINT intChannelIndex,
                            const struct_CeRegContent* pContent);

MAG_API BOOL WINAPI MAG_SetUserROIs(UINT intChannelIndex, const struct_UserROIs* pROI);
MAG_API BOOL WINAPI MAG_SetUserROIsEx(UINT intChannelIndex, const struct_RectROI * pROIs, UINT intROINum);

MAG_API BOOL WINAPI MAG_SetIrregularROIs(UINT intChannelIndex, const struct_IrregularROI * pROIs, UINT intROINum);

MAG_API BOOL WINAPI MAG_SetROIReportCallBack(UINT intChannelIndex, MAG_ROICALLBACK pCallBack, void * pUserData);

MAG_API BOOL WINAPI MAG_SetIrregularROIReportCallBack(UINT intChannelIndex, MAG_ROICALLBACK pCallBack, void * pUserData);

//----------------------------------------------------

MAG_API BOOL WINAPI MAG_IsProcessingImage(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_StartProcessImage(UINT intChannelIndex,
                                          const OutputPara* paraOut,
                                          MAG_FRAMECALLBACK funcFrame,
                                          DWORD dwStreamType, ULONG lUserData);

MAG_API BOOL WINAPI
    MAG_StartProcessPulseImage(UINT intChannelIndex, const OutputPara* paraOut,
                               MAG_FRAMECALLBACK funcFrame, DWORD dwStreamType,
                               ULONG lUserData);

MAG_API BOOL WINAPI MAG_TransferPulseImage(UINT intChannelIndex);

MAG_API void WINAPI MAG_StopProcessImage(UINT intChannelIndex);

#ifdef __ANDROID__
MAG_API void WINAPI MAG_Pause(UINT intChannelIndex);

MAG_API void WINAPI MAG_Resume(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_IsPause(UINT intChannelIndex);
#endif

MAG_API BOOL WINAPI MAG_SetIoAlarmState(UINT intChannelIndex, BOOL bAlarm);

MAG_API void WINAPI MAG_SetColorPalette(UINT intChannelIndex,
                                        enum ColorPalette ColorPaletteIndex);
										
#ifdef __ANDROID__
MAG_API BOOL WINAPI MAG_SetImageTransform(UINT intChannelIndex, int flip, int rotate);

MAG_API BOOL WINAPI MAG_SetEnhancementMethod(UINT intChannelIndex,
									   enum ImageEnhancementMethod method,
									   union ImageEnhancementParam* param);
#else
MAG_API BOOL WINAPI MAG_SetSubsectionEnlargePara(UINT intChannelIndex,
                                                 int intX1, int intX2,
                                                 UCHAR byteY1, UCHAR byteY2);

MAG_API void WINAPI
    MAG_SetAutoEnlargePara(UINT intChannelIndex, DWORD dwAutoEnlargeRange,
                           int intBrightOffset, int intContrastOffset);

MAG_API void WINAPI MAG_SetIsothermalPara(UINT intChannelIndex,
                                          int intLowerLimit, int intUpperLimit);
#endif

MAG_API void WINAPI
    MAG_GetFixPara(UINT intChannelIndex, struct_FixPara* pBuffer);

MAG_API float WINAPI MAG_SetFixPara(UINT intChannelIndex,
                                    const struct_FixPara* pBuffer,
                                    BOOL bEnableCameraCorrect);

MAG_API const USHORT * WINAPI MAG_GetFilteredRaw(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_GetOutputBMPdata(UINT intChannelIndex,
                                         UCHAR const** pData,
                                         BITMAPINFO const** pInfo);

MAG_API BOOL WINAPI MAG_GetOutputBMPdata_copy(UINT intChannelIndex,
                                              UCHAR * pBmp,
                                              UINT intBufferSize);

MAG_API BOOL WINAPI MAG_GetOutputColorBardata(UINT intChannelIndex,
                                              UCHAR const** pData,
                                              BITMAPINFO const** pInfo);

MAG_API BOOL WINAPI MAG_GetOutputColorBardata_copy(UINT intChannelIndex,
                                                   UCHAR * pColorBar,
                                                   UINT intBufferSize);

MAG_API BOOL WINAPI MAG_GetOutputVideoData(UINT intChannelIndex,
                                           UCHAR const** pData,
                                           BITMAPINFO const** pInfo);

MAG_API BOOL WINAPI MAG_GetOutputVideoData_copy(UINT intChannelIndex,
                                                UCHAR * pBmp,
                                                UINT intBufferSize);

MAG_API const UCHAR* WINAPI MAG_GetOutputVideoYV12(UINT intChannelIndex);

MAG_API UINT WINAPI MAG_GetVideoPPS(UINT intChannelIndex, char * pBuffer,
                                    UINT intBufferLen);

MAG_API UINT WINAPI MAG_GetVideoSPS(UINT intChannelIndex, char * pBuffer,
                                    UINT intBufferLen);

MAG_API UINT WINAPI MAG_GetVideoVCL(UINT intChannelIndex, char * pBuffer,
                                    UINT intBufferLen);

MAG_API BOOL WINAPI MAG_GetTemperatureData(UINT intChannelIndex, int* pData,
                                           UINT intBufferSize,
                                           BOOL bEnableExtCorrect);
MAG_API BOOL WINAPI MAG_GetTemperatureData_Raw(UINT intChannelIndex, int * pData,
                                               UINT intBufferSize,
                                               BOOL bEnableExtCorrect);

MAG_API void WINAPI MAG_SetEXLevel(UINT intChannelIndex, enum EX ExLevel,
                                   int intCenterX, int intCenterY);

MAG_API enum EX WINAPI MAG_GetEXLevel(UINT intChannelIndex);

MAG_API void WINAPI
    MAG_SetDetailEnhancement(UINT intChannelIndex, int intDDE, BOOL bQuickDDE);

MAG_API int WINAPI MAG_FixTemperature(UINT intChannelIndex, int intT,
                                      float fEmissivity, DWORD dwPosX,
                                      DWORD dwPosY);

MAG_API int WINAPI MAG_FixTemperature2(UINT intChannelIndex, int intT,
                                      float fEmissivity, DWORD dwPos);

MAG_API const struct_State* WINAPI
    MAG_GetFrameStatisticalData(UINT intChannelIndex);

MAG_API int WINAPI MAG_GetTemperatureProbe(UINT intChannelIndex, DWORD dwPosX,
                                           DWORD dwPosY, UINT intSize);

MAG_API int WINAPI MAG_GetTemperatureProbe2(UINT intChannelIndex, DWORD dwPos, UINT intSize);

MAG_API int WINAPI MAG_GetLineTemperatureInfo(UINT intChannelIndex, int* buffer,
                                              UINT intBufferSizeByte,
                                              int info[3], UINT x0, UINT y0,
                                              UINT x1, UINT y1);

MAG_API BOOL WINAPI MAG_GetRectTemperatureInfo(UINT intChannelIndex, UINT x0,
                                               UINT y0, UINT x1, UINT y1,
                                               int info[5]);

MAG_API BOOL WINAPI MAG_GetEllipseTemperatureInfo(UINT intChannelIndex, UINT x0,
                                                  UINT y0, UINT x1, UINT y1,
                                                  int info[5]);

MAG_API BOOL WINAPI MAG_GetRgnTemperatureInfo(UINT intChannelIndex,
                                              const UINT* Pos,
                                              UINT intPosNumber, int info[5]);

MAG_API BOOL WINAPI MAG_UseTemperatureMask(UINT intChannelIndex, BOOL bUse);

MAG_API BOOL WINAPI MAG_IsUsingTemperatureMask(UINT intChannelIndex);

MAG_API BOOL WINAPI
    MAG_SaveBMP(UINT intChannelIndex, DWORD dwIndex, const char* charFilename);

MAG_API int WINAPI
    MAG_SaveDDT2Buffer(UINT intChannelIndex, void* pBuffer, UINT intBufferSize);

MAG_API BOOL WINAPI MAG_SaveDDT(UINT intChannelIndex, const char* charFilename);

MAG_API BOOL WINAPI MAG_LoadDDT(UINT intChannelIndex, OutputPara* paraOut,
                                const char* charFilename,
                                MAG_FRAMECALLBACK funcFrame, ULONG lUserData);

MAG_API BOOL WINAPI MAG_LoadBufferedDDT(UINT intChannelIndex,
                                        OutputPara * paraOut,
                                        const void * pBuffer,
                                        UINT intBufferSize,
                                        MAG_FRAMECALLBACK funcFrame,
                                        ULONG lUserData);

MAG_API BOOL WINAPI
    MAG_SetPTZCmd(UINT intChannelIndex, enum PTZCmd cmd, DWORD dwPara);

MAG_API BOOL WINAPI MAG_QueryPTZState(UINT intChannelIndex, enum PTZQuery query,
                                      int* intValue, UINT intTimeoutMS);

MAG_API BOOL WINAPI MAG_SetSerialCmd(UINT intChannelIndex, const BYTE* buffer,
                                     UINT intBufferLen);

MAG_API BOOL WINAPI MAG_SetSerialCallBack(UINT intChannelIndex,
                                          MAG_SERIALCALLBACK pCallBack,
                                          void* pUserData);

MAG_API BOOL WINAPI
    MAG_SetVideoContrast(UINT intChannelIndex, int intContrastOffset);

MAG_API BOOL WINAPI
    MAG_SetVideoBrightness(UINT intChannelIndex, int intBrightnessOffset);

MAG_API BOOL WINAPI MAG_SDStorageMGT(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SDStorageBMP(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SDStorageMGSStart(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SDStorageMGSStop(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SDStorageAviStart(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_SDStorageAviStop(UINT intChannelIndex);

MAG_API BOOL WINAPI MAG_LocalStorageMgsRecord(UINT intChannelIndex, const char * charFileName, UINT intSamplePeriod);

MAG_API int WINAPI MAG_LocalStorageMgsPlay(UINT intChannelIndex, const char * charFileName, MAG_FRAMECALLBACK funcFrame, ULONG lUserData);

MAG_API BOOL WINAPI MAG_LocalStorageMgsSeekFrame(UINT intChannelIndex, UINT intFrameIndex);

MAG_API BOOL WINAPI MAG_LocalStorageMgsPopFrame(UINT intChannelIndex);

MAG_API void WINAPI MAG_LocalStorageMgsStop(UINT intChannelIndex);

MAG_API void WINAPI MAG_LockFrame(UINT intChannelIndex);

MAG_API void WINAPI MAG_UnLockFrame(UINT intChannelIndex);

//----------------------------------------------------

MAG_API BOOL WINAPI MAG_GetCurrentOffset(UINT intChannelIndex,
                                         const char* charReferenceDDT,
                                         int* pOffsetX, int* pOffsetY);

#ifdef __cplusplus
}
#endif

#endif // _THERMOGROUPSDK_H_
