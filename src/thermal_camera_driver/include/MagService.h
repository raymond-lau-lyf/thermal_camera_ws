#ifndef __MAGSERVICE_H__
#define __MAGSERVICE_H__

#include "linux/ThermoGroupSDK.h"
#include <stdlib.h>

#ifndef MAG_DEFAULT_TIMEOUT
#define MAG_DEFAULT_TIMEOUT (500)
#endif

class CMagService {
  public:
#ifdef __ANDROID__
	CMagService(const char* log);
#else
    CMagService();
#endif
    ~CMagService();

    BOOL IsInitialized() { return m_bInitialized; }

    static void EnableAutoReConnect(BOOL bEnable);

    BOOL EnumCameras();
    UINT GetTerminalCount();
    UINT GetTerminalList(struct_TerminalList *pList, DWORD dwBufferSize);

    BOOL GetMulticastState(UINT intTargetIp, UINT *intMulticastIp,
                           UINT *intMulticastPort,
                           UINT intTimeoutMS = MAG_DEFAULT_TIMEOUT);

  private:
    BOOL Initialize();

    static BOOL m_bInitialized;
};

#endif
