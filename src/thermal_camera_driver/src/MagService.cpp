#include "MagService.h"

BOOL CMagService::m_bInitialized = FALSE;

#ifdef __ANDROID__
CMagService::CMagService(const char* log) {
    MAG_SetStorageDir(log);
    Initialize();
}
#else
CMagService::CMagService() { Initialize(); }
#endif

CMagService::~CMagService() {
    if (MAG_IsInitialized(0)) {
        MAG_Free(0);
    }

    if (MAG_IsChannelAvailable(0)) {
        MAG_DelChannel(0);
    }

    m_bInitialized = FALSE;
}

BOOL CMagService::Initialize() {
    if (m_bInitialized) {
        // ASSERT(0);//I guess you must created more than one CMagService class
        return TRUE;
    }

    if (!MAG_IsChannelAvailable(0)) {
        if (!MAG_NewChannel(0)) {
            return FALSE;
        }
    }

    m_bInitialized = MAG_Initialize(0, NULL);

    return m_bInitialized;
}

void CMagService::EnableAutoReConnect(BOOL bEnable) {
    MAG_EnableAutoReConnect(bEnable);
}

BOOL CMagService::EnumCameras() {
    if (!m_bInitialized) {
        Initialize();
    }

    return m_bInitialized ? MAG_EnumCameras() : FALSE;
}

UINT CMagService::GetTerminalList(struct_TerminalList *pList,
                                  DWORD dwBufferSize) {
    return m_bInitialized ? MAG_GetTerminalList(pList, dwBufferSize) : 0;
}

UINT CMagService::GetTerminalCount() {
    return m_bInitialized ? MAG_GetTerminalList(NULL, 0) : 0;
}

BOOL CMagService::GetMulticastState(UINT intTargetIp, UINT *intMulticastIp,
                                    UINT *intMulticastPort, UINT intTimeoutMS) {
    if (!m_bInitialized) {
        Initialize();
    }

    return m_bInitialized
               ? MAG_GetMulticastState(intTargetIp, intMulticastIp,
                                       intMulticastPort, intTimeoutMS)
               : 0;
}
