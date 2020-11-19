#ifndef OPENTHREAD_SRP_H_
#define OPENTHREAD_SRP_H_

#include <openthread/instance.h>

#ifdef __cplusplus
extern "C" {
#endif

otError otSrpClientStart(otInstance *aInstance);
void otSrpClientStop(otInstance *aInstance);
otError otSrpClientRegister(otInstance *aInstance, const char *aName, const char *aType,
                            const char *aDomain, const char *aHost, uint16_t aPort);
otError otSrpClientDeregister(otInstance *aInstance);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_SRP_H_
