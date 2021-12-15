#ifndef __MAGDEVICE_H__
#define __MAGDEVICE_H__

#include "linux/ThermoGroupSDK.h"
#include <stdlib.h>

#ifndef MAG_DEFAULT_TIMEOUT
#define MAG_DEFAULT_TIMEOUT (500)
#endif

class CMagDevice {
  public:
    CMagDevice();
    ~CMagDevice();

	UINT getChannelIndex() { return m_intChannelIndex; }
    BOOL Initialize();
    BOOL IsInitialized() { return m_bInitialized; }
    const struct_CamInfo *GetCamInfo();
    void GetCamInfoEx(struct_CamInfoEx * pInfo, UINT intSize);
    const struct_CeRegContent *GetRegContent() { return &m_RegContent; }

    void ConvertPos2XY(UINT intPos, UINT *pX, UINT *pY);
    UINT ConvertXY2Pos(UINT X, UINT Y);

    BOOL IsLinked();
    BOOL LinkCamera(const char *charIp,
                    UINT intTimeoutMS = 2 * MAG_DEFAULT_TIMEOUT);
    BOOL LinkCamera(UINT intIP, UINT intTimeoutMS = 2 * MAG_DEFAULT_TIMEOUT);
#ifdef __ANDROID__
	BOOL LinkCamera_usb(UINT intId, UINT intTimeoutMS = 2 * MAG_DEFAULT_TIMEOUT);
#endif
	BOOL LinkCameraEx(const char * charIp, USHORT shortPort = 33596,
                      const char * charCloudUser = "", const char * charCloudPwd = "",
                      UINT intCamSN = 0, const char * charCamUser = "", const char * charCamPwd = "",
                      UINT intTimeoutMS = 2 * MAG_DEFAULT_TIMEOUT);
    BOOL LinkCameraEx(UINT intIP, USHORT shortPort = 33596,
                      const char * charCloudUser = "", const char * charCloudPwd = "",
                      UINT intCamSN = 0, const char * charCamUser = "", const char * charCamPwd = "",
                      UINT intTimeoutMS = 2 * MAG_DEFAULT_TIMEOUT);
    void DisLinkCamera();
    UINT GetRecentHeartBeat();

    BOOL IsListening();
    BOOL ListenTo(UINT intIP);
    void StopListen();
    BOOL SetReConnectCallBack(MAG_RECONNECTCALLBACK pCallBack,
                              void * pUserData);

    BOOL ResetCamera();
    BOOL TriggerFFC();
    BOOL AutoFocus();
    BOOL SetPTZCmd(enum PTZCmd cmd, DWORD dwPara = 0);
    BOOL QueryPTZState(enum PTZQuery query, int *intValue,
                       UINT intTimeoutMS = MAG_DEFAULT_TIMEOUT);
    BOOL SetSerialCmd(const BYTE *buffer, UINT intBufferLen);
    BOOL SetSerialCallBack(MAG_SERIALCALLBACK pCallBack, void* pUserData);

    BOOL GetCameraTemperature(int intT[4],
                              UINT intTimeoutMS = MAG_DEFAULT_TIMEOUT);
    BOOL SetCameraRegContent(const struct_CeRegContent *pContent);

    BOOL SetUserROIs(const struct_UserROIs *pROI);
    BOOL SetUserROIsEx(const struct_RectROI * pROIs, UINT intROINum);
    BOOL SetIrregularROIs(const struct_IrregularROI * pROIs, UINT intROINum);

    BOOL SetROIReportCallBack(MAG_ROICALLBACK pCallBack, void * pUserData);
    BOOL SetIrregularROIReportCallBack(MAG_ROICALLBACK pCallBack, void * pUserData);

    BOOL IsProcessingImage();
    BOOL StartProcessImage(const OutputPara *paraOut,
                           MAG_FRAMECALLBACK funcFrame, DWORD dwStreamType,
                           ULONG lUserData);
    BOOL StartProcessPulseImage(const OutputPara *paraOut,
                                MAG_FRAMECALLBACK funcFrame, DWORD dwStreamType,
                                ULONG lUserData);
    BOOL TransferPulseImage();
    void StopProcessImage();
#ifdef __ANDROID__
    void Pause();
    void Resume();
    BOOL IsPause();
#endif

    void SetColorPalette(enum ColorPalette ColorPaletteIndex);
	
#ifdef __ANDROID__
	BOOL SetEnhancementMethod(enum ImageEnhancementMethod method,
							  union ImageEnhancementParam* param);
	BOOL SetImageTransform(int flip, int rotate);
#else
	BOOL SetSubsectionEnlargePara(int intX1, int intX2, UCHAR byteY1 = 0,
                                  UCHAR byteY2 = 255);
    void SetAutoEnlargePara(DWORD dwAutoEnlargeRange, int intBrightOffset = 0,
                            int intContrastOffset = 0);
    void SetIsothermalPara(int intLowerLimit, int intUpperLimit);
#endif
    void SetEXLevel(enum EX ExLevel, int intCenterX, int intCenterY);
    enum EX GetEXLevel();
    void SetDetailEnhancement(int intDDE, BOOL bQuickDDE = TRUE);
    BOOL SetVideoContrast(int intContrastOffset);
    BOOL SetVideoBrightness(int intBrightnessOffset);

    void GetFixPara(struct_FixPara *pPara);
    float SetFixPara(const struct_FixPara *pPara, BOOL bEnableCameraCorrect);
    
	int FixTemperature(int intT, float fEmissivity, DWORD dwPosX, DWORD dwPosY);
    int FixTemperature(int intT, float fEmissivity, DWORD dwPos);

    const USHORT * GetFilteredRaw();

    BOOL GetOutputBMPdata(UCHAR const **pData, BITMAPINFO const **pInfo);
    BOOL GetOutputBMPdata(UINT* pRGB32Data, int intDataSize);
    BOOL GetOutputColorBardata(UCHAR const **pData, BITMAPINFO const **pInfo);
    BOOL GetOutputColorBardata(UINT* pRGB32Data, int intDataSize);
    BOOL GetOutputVideoData(UCHAR const **pData, BITMAPINFO const **pInfo);
    BOOL GetOutputVideoData(UINT* pRGB32Data, int intDataSize);

    UINT GetBarWidth() const { return m_intBarWidth; }
    UINT GetBarHeight() const { return m_intBarHeight; }

    const UCHAR *GetOutputVideoYV12();

    UINT GetDevIPAddress() const { return m_intCamIPAddr; }
    const struct_State *GetFrameStatisticalData();
    BOOL GetTemperatureData(int *pData, UINT intBufferSize,
                            BOOL bEnableExtCorrect);
    BOOL GetTemperatureData_Raw(int *pData, UINT intBufferSize,
                            BOOL bEnableExtCorrect);
    int GetTemperatureProbe(DWORD dwPosX, DWORD dwPosY, UINT intSize);
    int GetTemperatureProbe(DWORD dwPos, UINT intSize);
    int GetLineTemperatureInfo(int *buffer, UINT intBufferSizeByte, int info[3],
                               UINT x0, UINT y0, UINT x1, UINT y1);
    BOOL GetRectTemperatureInfo(UINT x0, UINT y0, UINT x1, UINT y1,
                                int info[5]);
    BOOL GetEllipseTemperatureInfo(UINT x0, UINT y0, UINT x1, UINT y1,
                                   int info[5]);
    BOOL GetRgnTemperatureInfo(const UINT *Pos, UINT intPosNumber, int info[5]);

    BOOL UseTemperatureMask(BOOL bUse);
    BOOL IsUsingTemperatureMask();
	int SaveDDT2Buffer(void* pBuffer, UINT intBufferSize);
    BOOL LoadBufferedDDT(OutputPara * paraOut, const void * pBuffer,
                         UINT intBufferSize, MAG_FRAMECALLBACK funcFrame,
                         ULONG lUserData);

    BOOL SaveBMP(DWORD dwIndex = 0, const char *charFilename = NULL);
    BOOL SaveDDT(const char *charFilename = NULL);
    BOOL LoadDDT(OutputPara *paraOut, const char *charFilename,
                 MAG_FRAMECALLBACK funcFrame, ULONG lUserData);
    BOOL GetCurrentOffset(const char *charReferenceDDT, int *pOffsetX,
                          int *pOffsetY);

    BOOL SDStorageMGT();
    BOOL SDStorageBMP();
    BOOL SDStorageMGSStart();
    BOOL SDStorageMGSStop();
    BOOL SDStorageAviStart();
    BOOL SDStorageAviStop();

    BOOL LocalStorageMgsRecord(const char * charFilename=NULL, UINT intSamplePeriod=1);
    int LocalStorageMgsPlay(const char * charFilename, MAG_FRAMECALLBACK funcFrame, ULONG lUserData);
    BOOL LocalStorageMgsPopFrame();
    BOOL LocalStorageMgsSeekFrame(UINT intFrameIndex);
    void LocalStorageMgsStop();
    BOOL IsLocalMgsRecording() const {return m_bIsRecordingLocalMgs;}
    BOOL IsLocalMgsPlaying() const {return m_bIsPlayingLocalMgs;}

    void Lock();
    void Unlock();

  private:
    BOOL m_bInitialized;

    UINT m_intChannelIndex;
    UINT m_intCamIPAddr;
    struct_CamInfo m_CamInfo;
    struct_CeRegContent m_RegContent;

    BOOL m_bIsRecordingAvi;
    BOOL m_bIsRecordingMGS;
    BOOL m_bIsRecordingLocalMgs;
    BOOL m_bIsPlayingLocalMgs;

    UINT m_intBarWidth;
    UINT m_intBarHeight;
};

#endif
