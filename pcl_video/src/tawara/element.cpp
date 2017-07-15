/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, 2012, Geoffrey Biggs, geoffrey.biggs@aist.go.jp
 *     RT-Synthesis Research Group
 *     Intelligent Systems Research Institute,
 *     National Institute of Advanced Industrial Science and Technology (AIST),
 *     Japan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Geoffrey Biggs nor AIST, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <tawara/element.h>

#include <limits>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Element::Element(uint32_t id)
    : id_(id), offset_(std::numeric_limits<std::streampos>::max())
{
    if (id_ == 0 ||
            id_ == 0xFF ||
            id_ == 0xFFFF ||
            id_ == 0xFFFFFF ||
            id_ == 0xFFFFFFFF)
    {
        throw InvalidElementID() << err_id(id_);
    }
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

std::streamsize Element::size() const
{
    std::streamsize body(body_size());
    return tawara::ids::size(id_) + tawara::vint::size(body) +
        body;
}


///////////////////////////////////////////////////////////////////////////////
// I/O
///////////////////////////////////////////////////////////////////////////////

std::streamsize Element::write(std::ostream& output)
{
    // Fill in the offset of this element in the byte stream.
    offset_ = output.tellp();

    return write_id(output) + write_size(output) + write_body(output);
}


std::streamsize Element::write_id(std::ostream& output)
{
    return tawara::ids::write(id_, output);
}


std::streamsize Element::write_size(std::ostream& output)
{
    return tawara::vint::write(body_size(), output);
}


std::streamsize Element::read(std::istream& input)
{
    // Fill in the offset of this element in the byte stream.
    // The input stream will be at the start of the size value, so:
    //
    //  offset = current position - size of ID
    //
    // The cast here makes Apple's LLVM compiler happy
    offset_ = static_cast<std::streamsize>(input.tellg()) -
        ids::size(id_);
    // Get the element's body size
    vint::ReadResult result = tawara::vint::read(input);
    std::streamsize body_size(result.first);
    std::streamsize read_bytes(result.second);
    // The rest of the read is implemented by child classes
    return read_bytes + read_body(input, body_size);
}


///////////////////////////////////////////////////////////////////////////////
// Other functions in element.h
///////////////////////////////////////////////////////////////////////////////

std::streamsize tawara::skip_read(std::istream& input, bool and_id)
{
    std::streamsize skipped_bytes(0);
    if (and_id)
    {
        skipped_bytes += ids::read(input).second;
    }
    vint::ReadResult size_res(vint::read(input));
    skipped_bytes += size_res.second;
    input.seekg(size_res.first, std::ios::cur);
    skipped_bytes += size_res.first;
    return skipped_bytes;
}


std::streamsize tawara::skip_write(std::iostream& stream, bool and_id)
{
    std::streampos cur_read(stream.tellg());
    stream.seekg(stream.tellp());
    std::streamsize skipped_bytes = tawara::skip_read(stream, and_id);
    stream.seekp(stream.tellg());
    stream.seekg(cur_read);
    return skipped_bytes;
}

