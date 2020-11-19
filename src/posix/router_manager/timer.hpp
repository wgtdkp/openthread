/*
 *  Copyright (c) 2020, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POSIX_TIMER_HPP_
#define POSIX_TIMER_HPP_

#include "openthread-posix-config.h"

#include <stdint.h>

namespace ot
{

namespace Posix
{

typedef uint32_t MilliSeconds;

class Timer
{
public:
    friend class TimerScheduler;

    /**
     * This type represents the function bound to a Timer object.
     *
     */
    typedef void (*Handler)(Timer &aTimer, void *aContext);

    /**
     * This constructor binds the Handler.
     *
     * @param  aHandler  A function to be called when the timer fires.
     *
     */
    Timer(Handler aHandler, void *aContext);

    /**
     * This method starts the timer with given delay.
     *
     * @param  aDelay  The delay which the timer will fire after.
     *
     */
    void Start(MilliSeconds aDelay);

    /**
     * This method starts the timer with given fire time.
     *
     * @param  aFireTime  The time point the timer will fire at.
     *
     */
    void StartAt(MilliSeconds aFireTime);

    /**
     * This method stops a timer.
     *
     */
    void Stop();

    /**
     * This method devices if the timer is running.
     *
     * @returns A boolean indicates if the timer is running.
     *
     */
    bool IsRunning() const { return mIsRunning; }

    /**
     * this method returns the time the timer will fire at.
     *
     * @returns The fire time.
     *
     */
    MilliSeconds GetFireTime() const { return mFireTime; }

private:
    void Fire();

    Handler      mHandler;
    void *       mContext;
    MilliSeconds mFireTime;
    bool         mIsRunning;
    Timer *      mNext;
};

/**
 * This class implements a Timer Scheduler which accepts registration of
 * timer events and drives them.
 *
 */
class TimerScheduler
{
public:
    /**
     * This method returns the TimerScheduler singleton.
     *
     * @return A single, global Timer Scheduler.
     *
     */
    static TimerScheduler &Get();

    /**
     * This method process all timer events, cleanup dead timers.
     *
     * @param[in]  aNow  The current time.
     *
     * @returns Delay of next earliest timer event in MilliSeconds.
     *
     */
    void Process(MilliSeconds aNow);

    /**
     * This method returns the earliest fire time in all sorted timers.
     *
     */
    MilliSeconds GetEarliestFireTime() const;

    /**
     * This method adds a new timer into the scheduler.
     *
     * @param[in]  aTimer  The timer to be added.
     *
     */
    void Add(Timer *aTimer);

    /**
     * This method removes a timer.
     *
     */
    void Remove(Timer *aTimer);

    /**
     * This method clears all timers scheduled.
     *
     */
    void Clear();

private:
    TimerScheduler()
        : mSortedTimerList(nullptr)
    {
    }

    // The timer list sorted by their Fire Time. Earlier timer comes first.
    Timer *mSortedTimerList;
};


} // namespace Posix

} // namespace ot

#endif // POSIX_TIMER_HPP_
