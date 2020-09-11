/*
 *    Copyright (c) 2020, The OpenThread Authors.
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the copyright holder nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *    POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file defines the SWIG interface of the commissioner.
 *
 * See http://www.swig.org for more information about SWIG.
 *
 */

%module(directors="1") TobleModule

%{
#include "toble.hpp"
#include "toble_driver.hpp"
%}

%include <arrays_java.i>
%include <carrays.i>
#include <std_string.i>
%include <std_vector.i>
%include <stdint.i>
%include <stl.i>
%include <typemaps.i>

// Remove the 'm' prefix of all members.
%rename("%(regex:/^(m)(.*)/\\2/)s") "";

// Convert first character of function names to lowercase.
%rename("%(firstlowercase)s", %$isfunction) "";

// Insert the code of loading native shared library into generated Java class.
%pragma(java) jniclasscode=%{
    static {
        try {
            System.loadLibrary("toble-java");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("failed to load native ToBLE library!\n" + e);
            System.exit(1);
        }
    }
%}

%feature("director") ot::Toble::TobleCallbacks;
%feature("director") ot::Toble::TobleDriver;
//%typemap(jstype) void* "java.lang.Object";
//%typemap(jstype) unsigned char "short";
//%typemap(jstype) unsigned char* "java.lang.Short";

%rename("$ignore", regexmatch$name="^otPlatToble") "";

%array_class(unsigned char, ByteArray);
%template(ByteVector) std::vector<uint8_t>;

namespace ot {
namespace Toble {
    %ignore Toble::GetOtInstance();
}
}

%include <openthread/error.h>
%include <openthread/platform/toble.h>
%include "toble_driver.hpp"
%include "toble.hpp"
