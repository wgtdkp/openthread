
#ifndef BTP_FRAME_HPP_
#define BTP_FRAME_HPP_

#include "openthread-core-config.h"

#include <openthread/platform/toolchain.h>

#include "common/encoding.hpp"
#include "utils/wrap_stdint.h"

using ot::Encoding::LittleEndian::HostSwap16;
using ot::Encoding::LittleEndian::HostSwap32;

namespace ot {
namespace Toble {

#if OPENTHREAD_CONFIG_ENABLE_TOBLE

OT_TOOL_PACKED_BEGIN
class Frame
{
public:
    enum
    {
        kHandshakeFlag  = 1 << 6,
        kManagementFlag = 1 << 5,
        kAckFlag        = 1 << 3,
        kEndFlag        = 1 << 2,
        kContinueFlag   = 1 << 1,
        kBeginFlag      = 1 << 0,
    };

    void Init(void) { mFlags = 0; }

    bool IsHandshake(void) const { return (mFlags & kHandshakeFlag) != 0; }

    bool IsAck(void) const { return (mFlags & kAckFlag) != 0; }

    bool IsBegin(void) const { return (mFlags & kBeginFlag) != 0; }

    bool IsEnd(void) const { return (mFlags & kEndFlag) != 0; }

protected:
    void SetFlags(uint8_t aFlags) { mFlags = aFlags; }

    uint8_t mFlags;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class HandshakeRequest : public Frame
{
public:
    void Init(uint16_t aMtu, uint8_t aWindowSize)
    {
        SetFlags(kFlags);
        mOpcode     = kOpcode;
        mVersions   = HostSwap32(kVersions);
        mMtu        = HostSwap16(aMtu);
        mWindowSize = aWindowSize;
    }

    uint16_t GetMtu(void) const { return HostSwap16(mMtu); }

    uint8_t GetWindowSize(void) const { return mWindowSize; }

private:
    enum
    {
        kFlags    = 0x6e,
        kOpcode   = 0x6c,
        kVersions = 0x3 << 28,
    };

    uint8_t  mOpcode;
    uint32_t mVersions;
    uint16_t mMtu;
    uint8_t  mWindowSize;
} OT_TOOL_PACKED_END;

OT_TOOL_PACKED_BEGIN
class HandshakeResponse : public Frame
{
public:
    void Init(uint16_t aSegmentSize, uint8_t aWindowSize)
    {
        SetFlags(kFlags);
        mOpcode      = kOpcode;
        mVersion     = kVersion;
        mSegmentSize = HostSwap16(aSegmentSize);
        mWindowSize  = aWindowSize;
    }

    uint16_t GetSegmentSize(void) const { return HostSwap16(mSegmentSize); }

    uint8_t GetWindowSize(void) const { return mWindowSize; }

private:
    enum
    {
        kFlags   = 0x6e,
        kOpcode  = 0x6c,
        kVersion = 0x4,
    };

    uint8_t  mOpcode;
    uint8_t  mVersion;
    uint16_t mSegmentSize;
    uint8_t  mWindowSize;
} OT_TOOL_PACKED_END;

#endif // #if OPENTHREAD_CONFIG_ENABLE_TOBLE

} // namespace Toble
} // namespace ot

#endif // BTP_FRAME_HPP_
