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

#include <tawara/simple_block.h>

#include <tawara/el_ids.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

SimpleBlock::SimpleBlock(uint64_t track_number, int16_t timecode,
        LacingType lacing)
    : BlockElement(tawara::ids::SimpleBlock, track_number, timecode, lacing),
    keyframe_(false), discardable_(false),
    block_(track_number, timecode, lacing)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void SimpleBlock::swap(SimpleBlock& other)
{
    std::swap(keyframe_, other.keyframe_);
    std::swap(discardable_, other.discardable_);
    block_.swap(other.block_);
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(SimpleBlock const& lhs, SimpleBlock const& rhs)
{
    return lhs.keyframe_ == rhs.keyframe_ &&
        lhs.discardable_ == rhs.discardable_ &&
        lhs.block_ == rhs.block_;
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize SimpleBlock::body_size() const
{
    return block_.size();
}


std::streamsize SimpleBlock::write_body(std::ostream& output)
{
    uint8_t extra_flags(0);

    if (keyframe_)
    {
        extra_flags |= 0x01;
    }
    if (discardable_)
    {
        extra_flags |= 0x80;
    }
    return block_.write(output, extra_flags);
}


std::streamsize SimpleBlock::read_body(std::istream& input,
        std::streamsize size)
{
    BlockImpl::ReadResult res(block_.read(input, size));
    if (res.second & 0x01)
    {
        keyframe_ = true;
    }
    else
    {
        keyframe_ = false;
    }
    if (res.second & 0x80)
    {
        discardable_ = true;
    }
    else
    {
        discardable_ = false;
    }
    return res.first;
}

