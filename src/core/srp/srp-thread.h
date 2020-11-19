/* srp-thread.c
 *
 * Copyright (c) 2019-2020 Apple Computer, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * srp host API implementation for Posix using ioloop primitives.
 */

#include <openthread/instance.h>

#include <dns_sd.h>
#include <srp-api.h>

#ifdef __cplusplus
extern "C" {
#endif

int srp_thread_init(otInstance* instance);
int srp_thread_shutdown(otInstance* instance);

void* create_timer(otInstance* instance, srp_wakeup_callback_t callback);
void start_timer(otInstance* instance, void *timer, uint32_t milliseconds);
void stop_timer(otInstance* instance, void *timer);

void register_callback(DNSServiceRef sdRef, DNSServiceFlags flags, DNSServiceErrorType errorCode,
                  const char *name, const char *regtype, const char *domain, void *context);

#ifdef __cplusplus
} // extern "C"
#endif

// Local Variables:
// mode: C
// tab-width: 4
// c-file-style: "bsd"
// c-basic-offset: 4
// fill-column: 108
// indent-tabs-mode: nil
// End:
