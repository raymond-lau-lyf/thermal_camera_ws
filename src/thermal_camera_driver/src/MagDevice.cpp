#include "MagDevice.h"

#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

CMagDevice::CMagDevice()
    : m_bInitialized(FALSE), m_intChannelIndex(-1), m_intCamIPAddr(0),
      m_bIsRecordingAvi(FALSE), m_bIsRecordingMGS(FALSE), m_bIsRecordingLocalMgs(FALSE),
      m_bIsPlayingLocalMgs(FALSE),
      m_intBarWidth(0), m_intBarHeight(0) {
    memset(&m_CamInfo, 0, sizeof(m_CamInfo));
    memset(&m_RegContent, 0, sizeof(m_RegContent));

    Initialize();
}

CMagDevice::~CMagDevice() {
    if (MAG_IsProcessingImage(m_intChannelIndex)) {
        MAG_StopProcessImage(m_intChannelIndex);
    }

    if (MAG_IsListening(m_intChannelIndex)) {
        MAG_StopListen(m_intChannelIndex);
    }

    if (MAG_IsLinked(m_intChannelIndex)) {
        DisLinkCamera(); // include stop sd storage
    }

    if (MAG_IsInitialized(m_intChannelIndex)) {
        MAG_Free(m_intChannelIndex);
        m_bInitialized = FALSE;
    }

    if (MAG_IsChannelAvailable(m_intChannelIndex)) {
        MAG_DelChannel(m_intChannelIndex);
        m_intChannelIndex = -1;
    }
}

BOOL CMagDevice::Initialize() {
    if (m_bInitialized) {
        return TRUE;
    }

    if (m_intChannelIndex <= 0 || m_intChannelIndex > MAX_DEVICE) {
        for (int i = 1; i <= MAX_DEVICE; i++) {
            if (!MAG_IsChannelAvailable(i)) // find an unused channel
            {
                MAG_NewChannel(i);
                m_intChannelIndex = i;

                break;
            }
        }
    }

    if (m_intChannelIndex > 0 && m_intChannelIndex <= MAX_DEVICE) {
        m_bInitialized = MAG_Initialize(m_intChannelIndex, NULL);
    }

    return m_bInitialized;
}

const struct_CamInfo* CMagDevice::GetCamInfo() {
    MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
    return &m_CamInfo;
}

void CMagDevice::GetCamInfoEx(struct_CamInfoEx * pInfo, UINT intSize) {
    return MAG_GetCamInfoEx(m_intChannelIndex, pInfo, intSize);
}

BOOL CMagDevice::IsLinked() { return MAG_IsLinked(m_intChannelIndex); }

BOOL CMagDevice::LinkCamera(const char* charIp, UINT intTimeoutMS) {
    return LinkCamera(inet_addr(charIp), intTimeoutMS);
}

BOOL CMagDevice::LinkCamera(UINT intIP, UINT intTimeoutMS) {
    if (MAG_LinkCamera(m_intChannelIndex, intIP, intTimeoutMS)) {
        if (!MAG_ReadCameraRegContent(m_intChannelIndex, &m_RegContent,
                                 MAG_DEFAULT_TIMEOUT, FALSE)) {
            MAG_DisLinkCamera(m_intChannelIndex);
            return FALSE;
        }
        else {
            m_intCamIPAddr = intIP;
            MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
            return TRUE;
        }
    } else {
        return FALSE;
    }
}

#ifdef __ANDROID__
BOOL CMagDevice::LinkCamera_usb(UINT intIP, UINT intTimeoutMS)
{
    if (!MAG_LinkCamera_usb(m_intChannelIndex, intIP, intTimeoutMS)) {
        return FALSE;
    }

    if (!MAG_ReadCameraRegContent(m_intChannelIndex, &m_RegContent,
                                  MAG_DEFAULT_TIMEOUT, FALSE)) {
        MAG_DisLinkCamera(m_intChannelIndex);
        return FALSE;
    }

    m_intCamIPAddr = intIP;
    MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
    return TRUE;
}
#endif

BOOL CMagDevice::LinkCameraEx(const char * charIp, USHORT shortPort, const char * charCloudUser, const char * charCloudPwd, UINT intCamSN, const char * charCamUser, const char * charCamPwd, UINT intTimeoutMS)
{
    return LinkCameraEx(inet_addr(charIp), shortPort, charCloudUser, charCloudPwd, intCamSN, charCamUser, charCamPwd, intTimeoutMS);
}

BOOL CMagDevice::LinkCameraEx(UINT intIP, USHORT shortPort, const char * charCloudUser, const char * charCloudPwd, UINT intCamSN, const char * charCamUser, const char * charCamPwd, UINT intTimeoutMS)
{
    if (MAG_LinkCameraEx(m_intChannelIndex, intIP, shortPort,
        charCloudUser, charCloudPwd, intCamSN, charCamUser, charCamPwd, intTimeoutMS))
    {
        m_intCamIPAddr = intIP;
        MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
        MAG_ReadCameraRegContent(m_intChannelIndex, &m_RegContent, MAG_DEFAULT_TIMEOUT, FALSE);

        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


void CMagDevice::DisLinkCamera() {
    // remember to stop sd storage before dislink
    if (m_bIsRecordingMGS) {
        SDStorageMGSStop();
    }

    if (m_bIsRecordingAvi) {
        SDStorageAviStop();
    }

    m_intCamIPAddr = 0;
    m_intBarWidth = 0;
    m_intBarHeight = 0;

    MAG_DisLinkCamera(m_intChannelIndex);
}

UINT CMagDevice::GetRecentHeartBeat() {
    return MAG_GetRecentHeartBeat(m_intChannelIndex);
}

BOOL CMagDevice::IsListening() { return MAG_IsListening(m_intChannelIndex); }

BOOL CMagDevice::ListenTo(UINT intIP) {
    if (MAG_ListenTo(m_intChannelIndex, intIP)) {
        MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
        return TRUE;
    } else {
        return FALSE;
    }
}

void CMagDevice::StopListen() { return MAG_StopListen(m_intChannelIndex); }

BOOL CMagDevice::SetReConnectCallBack(MAG_RECONNECTCALLBACK pCallBack, void *pUserData)
{
    return MAG_SetReConnectCallBack(m_intChannelIndex, pCallBack, pUserData);
}

BOOL CMagDevice::ResetCamera() {
    // the user should stop image process before reset
    // if you forget, the sdk will call MAG_StopProcessImage()

    // remember to stop sd storage before reset
    if (m_bIsRecordingMGS) {
        SDStorageMGSStop();
    }

    if (m_bIsRecordingAvi) {
        SDStorageAviStop();
    }

    if (MAG_ResetCamera(m_intChannelIndex)) {
        // MAG_ResetCamera() will call MAG_Free() and MAG_DelChannel()
        // so the channel is invalid now
        m_bInitialized = FALSE;
        m_intChannelIndex = -1;

        // this object is reusable after call Initialize()

        return TRUE;
    } else {
        return FALSE;
    }
}

BOOL CMagDevice::TriggerFFC() { return MAG_TriggerFFC(m_intChannelIndex); }

BOOL CMagDevice::AutoFocus() {
    return MAG_SetPTZCmd(m_intChannelIndex, PTZFocusAuto, 0);
}

BOOL CMagDevice::SetPTZCmd(enum PTZCmd cmd, DWORD dwPara) {
    return MAG_SetPTZCmd(m_intChannelIndex, cmd, dwPara);
}

BOOL CMagDevice::QueryPTZState(enum PTZQuery query, int* intValue,
                               UINT intTimeoutMS) {
    return MAG_QueryPTZState(m_intChannelIndex, query, intValue, intTimeoutMS);
}

BOOL CMagDevice::SetSerialCmd(const BYTE* buffer, UINT intBufferLen) {
    return MAG_SetSerialCmd(m_intChannelIndex, buffer, intBufferLen);
}

BOOL CMagDevice::SetSerialCallBack(MAG_SERIALCALLBACK pCallBack,
                                   void* pUserData) {
    return MAG_SetSerialCallBack(m_intChannelIndex, pCallBack, pUserData);
}

BOOL CMagDevice::GetCameraTemperature(int intT[4], UINT intTimeoutMS) {
    return MAG_GetCameraTemperature(m_intChannelIndex, intT, intTimeoutMS);
}

BOOL CMagDevice::SetCameraRegContent(const struct_CeRegContent* pContent) {
    if (MAG_SetCameraRegContent(m_intChannelIndex, pContent)) {
        MAG_ReadCameraRegContent(m_intChannelIndex, &m_RegContent,
                                 MAG_DEFAULT_TIMEOUT * 4, FALSE);
        return TRUE;
    } else {
        return FALSE;
    }
}

BOOL CMagDevice::SetUserROIs(const struct_UserROIs* pROI) {
    return MAG_SetUserROIs(m_intChannelIndex, pROI);
}

BOOL CMagDevice::SetUserROIsEx(const struct_RectROI * pROIs, UINT intROINum){
    return MAG_SetUserROIsEx(m_intChannelIndex, pROIs, intROINum);
}

BOOL CMagDevice::SetIrregularROIs(const struct_IrregularROI * pROIs, UINT intROINum){
    return MAG_SetIrregularROIs(m_intChannelIndex, pROIs, intROINum);
}

BOOL CMagDevice::SetROIReportCallBack(MAG_ROICALLBACK pCallBack, void * pUserData){
    return MAG_SetROIReportCallBack(m_intChannelIndex, pCallBack, pUserData);
}

BOOL CMagDevice::SetIrregularROIReportCallBack(MAG_ROICALLBACK pCallBack, void * pUserData){
    return MAG_SetIrregularROIReportCallBack(m_intChannelIndex, pCallBack, pUserData);
}

BOOL CMagDevice::IsProcessingImage() {
    return MAG_IsProcessingImage(m_intChannelIndex);
}

BOOL CMagDevice::StartProcessImage(const OutputPara* paraOut,
                                   MAG_FRAMECALLBACK funcFrame,
                                   DWORD dwStreamType, ULONG lUserData) {
    BOOL res = MAG_StartProcessImage(m_intChannelIndex, paraOut, funcFrame,
                                     dwStreamType, lUserData);
    if (res) {
        m_intBarWidth = paraOut->dwColorBarWidth;
        m_intBarHeight = paraOut->dwColorBarHeight;
    }
    return res;
}
 
BOOL CMagDevice::StartProcessPulseImage(const OutputPara* paraOut,
                                        MAG_FRAMECALLBACK funcFrame,
                                        DWORD dwStreamType, ULONG lUserData) {
    BOOL res = MAG_StartProcessPulseImage(m_intChannelIndex, paraOut, funcFrame,
                                          dwStreamType, lUserData);
    if (res) {
        m_intBarWidth = paraOut->dwColorBarWidth;
        m_intBarHeight = paraOut->dwColorBarHeight;
    }
    return res;
}

BOOL CMagDevice::TransferPulseImage() {
    return MAG_TransferPulseImage(m_intChannelIndex);
}

void CMagDevice::StopProcessImage() {    
    if (m_bIsRecordingLocalMgs)
    {
        LocalStorageMgsStop();
    }

    MAG_StopProcessImage(m_intChannelIndex);

    m_intBarWidth = 0;
    m_intBarHeight = 0;
}

#ifdef __ANDROID__
void CMagDevice::Pause() {
    MAG_Pause(m_intChannelIndex);
}

void CMagDevice::Resume() {
    MAG_Resume(m_intChannelIndex);
}

BOOL CMagDevice::IsPause() {
    return MAG_IsPause(m_intChannelIndex);
}
#endif

void CMagDevice::SetColorPalette(enum ColorPalette ColorPaletteIndex) {
    return MAG_SetColorPalette(m_intChannelIndex, ColorPaletteIndex);
}

#ifndef __ANDROID__
BOOL CMagDevice::SetSubsectionEnlargePara(int intX1, int intX2, UCHAR byteY1,
                                          UCHAR byteY2) {
    return MAG_SetSubsectionEnlargePara(m_intChannelIndex, intX1, intX2, byteY1,
                                        byteY2);
}

void CMagDevice::SetIsothermalPara(int intLowerLimit, int intUpperLimit) {
    return MAG_SetIsothermalPara(m_intChannelIndex, intLowerLimit,
                                 intUpperLimit);
}

void CMagDevice::SetAutoEnlargePara(DWORD dwAutoEnlargeRange,
                                    int intBrightOffset,
                                    int intContrastOffset) {
    return MAG_SetAutoEnlargePara(m_intChannelIndex, dwAutoEnlargeRange,
                                  intBrightOffset, intContrastOffset);
}
#else
BOOL CMagDevice::SetEnhancementMethod(enum ImageEnhancementMethod method,
						  union ImageEnhancementParam* param) {
	return MAG_SetEnhancementMethod(m_intChannelIndex, method, param);
}

BOOL CMagDevice::SetImageTransform(int flip, int rotate)
{
    return MAG_SetImageTransform(m_intChannelIndex, flip, rotate);
}
#endif

void CMagDevice::SetEXLevel(enum EX ExLevel, int intCenterX, int intCenterY) {
    return MAG_SetEXLevel(m_intChannelIndex, ExLevel, intCenterX, intCenterY);
}

enum EX CMagDevice::GetEXLevel() { return MAG_GetEXLevel(m_intChannelIndex); }

void CMagDevice::SetDetailEnhancement(int intDDE, BOOL bQuickDDE) {
    return MAG_SetDetailEnhancement(m_intChannelIndex, intDDE, bQuickDDE);
}

BOOL CMagDevice::SetVideoContrast(int intContrastOffset) {
    return MAG_SetVideoContrast(m_intChannelIndex, intContrastOffset);
}

BOOL CMagDevice::SetVideoBrightness(int intBrightnessOffset) {
    return MAG_SetVideoBrightness(m_intChannelIndex, intBrightnessOffset);
}

void CMagDevice::GetFixPara(struct_FixPara* pPara) {
    return MAG_GetFixPara(m_intChannelIndex, pPara);
}

float CMagDevice::SetFixPara(const struct_FixPara* pPara,
                             BOOL bEnableCameraCorrect) {
    return MAG_SetFixPara(m_intChannelIndex, pPara, bEnableCameraCorrect);
}

int CMagDevice::FixTemperature(int intT, float fEmissivity, DWORD dwPosX, DWORD dwPosY) {
    return MAG_FixTemperature(m_intChannelIndex, intT, fEmissivity, dwPosX, dwPosY);
}

int CMagDevice::FixTemperature(int intT, float fEmissivity, DWORD dwPos) {
    return MAG_FixTemperature2(m_intChannelIndex, intT, fEmissivity, dwPos);
}

const USHORT *CMagDevice::GetFilteredRaw() {
    return MAG_GetFilteredRaw(m_intChannelIndex);
}

BOOL CMagDevice::GetOutputBMPdata(UCHAR const** pData,
                                  BITMAPINFO const** pInfo) {
    return MAG_GetOutputBMPdata(m_intChannelIndex, pData, pInfo);
}

BOOL CMagDevice::GetOutputBMPdata(UINT* pRGB32Data, int intDataSize)
{
    if (intDataSize < (int)(m_CamInfo.intVideoWidth * m_CamInfo.intVideoHeight * 4)) {
        return FALSE;
    }

    const UCHAR* pData = NULL;
    const BITMAPINFO* pInfo = NULL;

    if (!MAG_GetOutputBMPdata(m_intChannelIndex, &pData, &pInfo)) {
        return FALSE;
    }


    UINT* pClrMap = (UINT*)((char*)pInfo + sizeof(BITMAPINFOHEADER));
    UINT* pBuf = pRGB32Data + (m_CamInfo.intVideoHeight - 1) * m_CamInfo.intVideoWidth;

    for (int j = m_CamInfo.intVideoHeight - 1; j >= 0; --j) {
        for (UINT i = 0; i < m_CamInfo.intVideoWidth; ++i) {
            *pBuf++ = pClrMap[*pData++] | (0xff << 24);
        }
        pBuf -= m_CamInfo.intVideoWidth * 2;
    }

    return TRUE;
}

BOOL CMagDevice::GetOutputColorBardata(UCHAR const** pData,
                                       BITMAPINFO const** pInfo) {
    return MAG_GetOutputColorBardata(m_intChannelIndex, pData, pInfo);
}

BOOL CMagDevice::GetOutputColorBardata(UINT* pRGB32Data, int intDataSize)
{
    if (intDataSize < (int)(m_intBarWidth * m_intBarHeight * 4)){
        return FALSE;
    }

    const UCHAR* pData = NULL;
    const BITMAPINFO* pInfo = NULL;

    if (!MAG_GetOutputColorBardata(m_intChannelIndex, &pData, &pInfo)) {
        return FALSE;
    }

    UINT* pClrMap = (UINT*)((char*)pInfo + sizeof(BITMAPINFOHEADER));
    UINT* pBuf = pRGB32Data + (m_intBarHeight - 1) * m_intBarWidth;

    for (int j = m_intBarHeight - 1; j >= 0; --j) {
        for (UINT i = 0; i < m_intBarWidth; ++i) {
            *pBuf++ = pClrMap[*pData++] | (0xff << 24);
        }
        pBuf -= m_intBarWidth * 2;
    }

    return TRUE;
}

BOOL CMagDevice::GetOutputVideoData(UCHAR const** pData,
                                    BITMAPINFO const** pInfo) {
    return MAG_GetOutputVideoData(m_intChannelIndex, pData, pInfo);
}

BOOL CMagDevice::GetOutputVideoData(UINT* pRGB32Data, int intDataSize)
{
    if (intDataSize < (int)(m_CamInfo.intVideoWidth * m_CamInfo.intVideoHeight * sizeof(int))){
        return FALSE;
    }

    const UCHAR* pData = NULL;
    const BITMAPINFO* pInfo = NULL;

    if (!MAG_GetOutputVideoData(m_intChannelIndex, &pData, &pInfo)) {
        return FALSE;
    }

    memcpy(pRGB32Data, pData, m_CamInfo.intVideoWidth * m_CamInfo.intVideoHeight * sizeof(int));

    return TRUE;
}

const UCHAR* CMagDevice::GetOutputVideoYV12() {
    return MAG_GetOutputVideoYV12(m_intChannelIndex);
}

const struct_State* CMagDevice::GetFrameStatisticalData() {
    return MAG_GetFrameStatisticalData(m_intChannelIndex);
}

BOOL CMagDevice::GetTemperatureData(int* pData, UINT intBufferSize,
                                    BOOL bEnableExtCorrect) {
    return MAG_GetTemperatureData(m_intChannelIndex, pData, intBufferSize,
                                  bEnableExtCorrect);
}

BOOL CMagDevice::GetTemperatureData_Raw(int* pData, UINT intBufferSize,
                                    BOOL bEnableExtCorrect) {
    return MAG_GetTemperatureData_Raw(m_intChannelIndex, pData, intBufferSize,
                                  bEnableExtCorrect);
}

int CMagDevice::GetTemperatureProbe(DWORD dwPosX, DWORD dwPosY, UINT intSize) {
    return MAG_GetTemperatureProbe(m_intChannelIndex, dwPosX, dwPosY, intSize);
}

int CMagDevice::GetTemperatureProbe(DWORD dwPos, UINT intSize) {
    return MAG_GetTemperatureProbe2(m_intChannelIndex, dwPos, intSize);
}

int CMagDevice::GetLineTemperatureInfo(int* buffer, UINT intBufferSizeByte,
                                       int info[3], UINT x0, UINT y0, UINT x1,
                                       UINT y1) {
    return MAG_GetLineTemperatureInfo(m_intChannelIndex, buffer,
                                      intBufferSizeByte, info, x0, y0, x1, y1);
}

BOOL CMagDevice::GetRectTemperatureInfo(UINT x0, UINT y0, UINT x1, UINT y1,
                                        int info[5]) {
    return MAG_GetRectTemperatureInfo(m_intChannelIndex, x0, y0, x1, y1, info);
}

BOOL CMagDevice::GetEllipseTemperatureInfo(UINT x0, UINT y0, UINT x1, UINT y1,
                                           int info[5]) {
    return MAG_GetEllipseTemperatureInfo(m_intChannelIndex, x0, y0, x1, y1,
                                         info);
}

BOOL CMagDevice::GetRgnTemperatureInfo(const UINT* Pos, UINT intPosNumber,
                                       int info[5]) {
    return MAG_GetRgnTemperatureInfo(m_intChannelIndex, Pos, intPosNumber,
                                     info);
}

BOOL CMagDevice::UseTemperatureMask(BOOL bUse) {
    return MAG_UseTemperatureMask(m_intChannelIndex, bUse);
}

BOOL CMagDevice::IsUsingTemperatureMask() {
    return MAG_IsUsingTemperatureMask(m_intChannelIndex);
}

int CMagDevice::SaveDDT2Buffer(void* pBuffer, UINT intBufferSize) {
    return MAG_SaveDDT2Buffer(m_intChannelIndex, pBuffer, intBufferSize);
}

BOOL CMagDevice::LoadBufferedDDT(OutputPara *paraOut, const void *pBuffer,
                                 UINT intBufferSize, MAG_FRAMECALLBACK funcFrame,
                                 ULONG lUserData)
{
    if (!MAG_IsProcessingImage(m_intChannelIndex)) {
        if (!MAG_LoadBufferedDDT(m_intChannelIndex, paraOut, pBuffer, intBufferSize,
                                 funcFrame, lUserData)) {
            return FALSE;
        }

        MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
        m_intBarWidth = paraOut->dwColorBarWidth;
        m_intBarHeight = paraOut->dwColorBarHeight;
        return TRUE;
    } else {
        return FALSE;
    }
}

BOOL CMagDevice::SaveBMP(DWORD dwIndex, const char* charFilename) {
    return MAG_SaveBMP(m_intChannelIndex, dwIndex, charFilename);
}

BOOL CMagDevice::SaveDDT(const char* charFilename) {
    return MAG_SaveDDT(m_intChannelIndex, charFilename);
}

BOOL CMagDevice::LoadDDT(OutputPara* paraOut, const char* charFilename,
                         MAG_FRAMECALLBACK funcFrame, ULONG lUserData) {
    if (!MAG_IsProcessingImage(m_intChannelIndex)) {
        if (!MAG_LoadDDT(m_intChannelIndex, paraOut, charFilename, funcFrame,
                         lUserData)) {
            return FALSE;
        }

        MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
        m_intBarWidth = paraOut->dwColorBarWidth;
        m_intBarHeight = paraOut->dwColorBarHeight;
        return TRUE;
    } else {
        return FALSE;
    }
}

BOOL CMagDevice::GetCurrentOffset(const char* charReferenceDDT, int* pOffsetX,
                                  int* pOffsetY) {
    return MAG_GetCurrentOffset(m_intChannelIndex, charReferenceDDT, pOffsetX,
                                pOffsetY);
}

BOOL CMagDevice::SDStorageMGT() { return MAG_SDStorageMGT(m_intChannelIndex); }

BOOL CMagDevice::SDStorageBMP() { return MAG_SDStorageBMP(m_intChannelIndex); }

BOOL CMagDevice::SDStorageMGSStart() {
    m_bIsRecordingMGS |= MAG_SDStorageMGSStart(m_intChannelIndex);
    return m_bIsRecordingMGS;
}

BOOL CMagDevice::SDStorageMGSStop() {
    BOOL bReturn = MAG_SDStorageMGSStop(m_intChannelIndex);
    if (bReturn) {
        m_bIsRecordingMGS = FALSE;
    }

    return bReturn;
}

BOOL CMagDevice::SDStorageAviStart() {
    m_bIsRecordingAvi |= MAG_SDStorageAviStart(m_intChannelIndex);
    return m_bIsRecordingAvi;
}

BOOL CMagDevice::SDStorageAviStop() {
    BOOL bReturn = MAG_SDStorageAviStop(m_intChannelIndex);
    if (bReturn) {
        m_bIsRecordingAvi = FALSE;
    }

    return bReturn;
}

BOOL CMagDevice::LocalStorageMgsRecord(const char * charFilename, UINT intSamplePeriod)
{
    m_bIsRecordingLocalMgs |= MAG_LocalStorageMgsRecord(m_intChannelIndex, charFilename, intSamplePeriod);
    return m_bIsRecordingLocalMgs;
}

int CMagDevice::LocalStorageMgsPlay(const char * charFilename, MAG_FRAMECALLBACK funcFrame, ULONG lUserData)
{
    int intTotalFrames = MAG_LocalStorageMgsPlay(m_intChannelIndex, charFilename, funcFrame, lUserData);
    if (intTotalFrames > 0)
    {
        m_bIsPlayingLocalMgs = TRUE;
    }

    if (m_bIsPlayingLocalMgs)
    {
        MAG_GetCamInfo(m_intChannelIndex, &m_CamInfo, sizeof(m_CamInfo));
    }

    return intTotalFrames;
}

BOOL CMagDevice::LocalStorageMgsPopFrame()
{
    return MAG_LocalStorageMgsPopFrame(m_intChannelIndex);
}

BOOL CMagDevice::LocalStorageMgsSeekFrame(UINT intFrameIndex)
{
    return MAG_LocalStorageMgsSeekFrame(m_intChannelIndex, intFrameIndex);
}

void CMagDevice::LocalStorageMgsStop()
{
    MAG_LocalStorageMgsStop(m_intChannelIndex);
    m_bIsRecordingLocalMgs = FALSE;
    m_bIsPlayingLocalMgs = FALSE;
}

void CMagDevice::Lock() { return MAG_LockFrame(m_intChannelIndex); }

void CMagDevice::Unlock() { return MAG_UnLockFrame(m_intChannelIndex); }

void CMagDevice::ConvertPos2XY(UINT intPos, UINT* pX, UINT* pY) {
    UINT W = m_CamInfo.intFPAWidth;
    if (W && pX && pY) {
        *pY = intPos / W;
        *pX = intPos - (*pY) * W;
    }
}

UINT CMagDevice::ConvertXY2Pos(UINT X, UINT Y) {
    return Y * m_CamInfo.intFPAWidth + X;
}
