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

#include <tawara/block_impl.h>

#include <algorithm>
#include <boost/foreach.hpp>
#include <numeric>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

BlockImpl::BlockImpl(uint64_t track_number, int16_t timecode,
        LacingType lacing)
    : Block(track_number, timecode, lacing),
    track_num_(track_number), timecode_(timecode), invisible_(false),
    lacing_(lacing)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

BlockImpl& BlockImpl::operator=(BlockImpl const& other)
{
    track_num_ = other.track_num_;
    timecode_ = other.timecode_;
    invisible_ = other.invisible_;
    lacing_ = other.lacing_;
    frames_ = other.frames_;
    return *this;
}


BlockImpl::size_type BlockImpl::max_count() const
{
    if (lacing_ == LACING_NONE)
    {
        return 1;
    }
    return frames_.max_size();
}


void BlockImpl::push_back(BlockImpl::value_type const& value)
{
    if (!value)
    {
        // Empty pointer
        throw EmptyFrame();
    }
    if (value->empty())
    {
        // Pointer has a vector, but it is empty
        throw EmptyFrame();
    }
    if (frames_.size() >= 1 && lacing_ == LACING_NONE)
    {
        throw MaxLaceSizeExceeded() << err_max_lace(1) <<
            err_req_lace(frames_.size() + 1);
    }
    if (frames_.size() > 0 && lacing_ == LACING_FIXED &&
            value->size() != frames_[0]->size())
    {
        throw BadLacedFrameSize() << err_frame_size(value->size());
    }
    frames_.push_back(value);
}


void BlockImpl::resize(BlockImpl::size_type count)
{
    if (count > 1 && lacing_ == LACING_NONE)
    {
        throw MaxLaceSizeExceeded() << err_max_lace(1) << err_req_lace(count);
    }
    frames_.resize(count);
}


void BlockImpl::swap(BlockImpl& other)
{
    std::swap(track_num_, other.track_num_);
    std::swap(timecode_, other.timecode_);
    std::swap(invisible_, other.invisible_);
    std::swap(lacing_, other.lacing_);
    frames_.swap(other.frames_);
}


std::streamsize add_size(std::streamsize x, Block::value_type frame)
{
    if (!frame)
    {
        return x;
    }
    return x + frame->size();
}

std::streamsize BlockImpl::size() const
{
    // Timecode (2) + flags (1)
    std::streamsize hdr_size(3);

    hdr_size += tawara::vint::size(track_num_);

    switch(lacing_)
    {
        case LACING_EBML:
            hdr_size += 1; // Number of frames
            if (!frames_.empty())
            {
                std::streamsize prev_size(frames_[0]->size());
                hdr_size += vint::size(prev_size);
                // Add the size of each of the remaining frames except the last
                BOOST_FOREACH(value_type frame,
                        std::make_pair(frames_.begin() + 1, frames_.end() - 1))
                {
                    std::streamsize size_diff(frame->size() - prev_size);
                    prev_size = frame->size();
                    hdr_size += vint::s_to_u(size_diff).second;
                }
            }
            break;
        case LACING_FIXED:
            // Only the number of frames is stored
            hdr_size += 1;
            break;
        case LACING_NONE:
            // No lacing header
            break;
    }

    return hdr_size + std::accumulate(frames_.begin(), frames_.end(), 0,
            std::cref(add_size));
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(BlockImpl const& lhs, BlockImpl const& rhs)
{
    bool frames_equal(false);
    if (lhs.frames_.size() == rhs.frames_.size())
    {
        // Because the frames are pointers, the vectors cannot be compared
        // directly. Instead, each frame pointer must be dereferenced and
        // compared.
        frames_equal = true;
        for(std::vector<BlockImpl::value_type>::const_iterator
                lf(lhs.frames_.begin()), rf(rhs.frames_.begin());
            lf != lhs.frames_.end() && rf != rhs.frames_.end();
            ++lf, ++rf)
        {
            if (**lf != **rf)
            {
                frames_equal = false;
                break;
            }
        }
    }
    return lhs.track_num_ == rhs.track_num_ &&
        lhs.timecode_ == rhs.timecode_ &&
        lhs.invisible_ == rhs.invisible_ &&
        lhs.lacing_ == rhs.lacing_ &&
        frames_equal;
}


///////////////////////////////////////////////////////////////////////////////
// I/O
///////////////////////////////////////////////////////////////////////////////

std::streamsize BlockImpl::write(std::ostream& output, uint8_t extra_flags)
{
    validate();

    std::streamsize written(0);

    // Write the track number
    written += vint::write(track_num_, output);
    // Write the time code (2 bytes) as big-endian (as per EBML)
    output.put(timecode_ >> 8);
    output.put(timecode_ & 0x00FF);
    if (!output)
    {
        throw tawara::WriteError() << tawara::err_pos(output.tellp());
    }
    written += 2;
    // Prepare and write the flags
    uint8_t flags(extra_flags);
    if (invisible_)
    {
        flags |= 0x10;
    }
    switch (lacing_)
    {
        case Block::LACING_EBML:
            flags |= 0x60;
            break;
        case Block::LACING_FIXED:
            flags |= 0x40;
            break;
        case LACING_NONE:
            // Nothing to do for no lacing
            break;
    }
    output.put(flags);
    if (!output)
    {
        throw tawara::WriteError() << tawara::err_pos(output.tellp());
    }
    written += 1;
    // Write the lacing header
    uint8_t num_frames(frames_.size());
    std::streamsize prev_size(0);
    switch (lacing_)
    {
        case Block::LACING_EBML:
            output.put(num_frames);
            if (!output)
            {
                throw tawara::WriteError() << tawara::err_pos(output.tellp());
            }
            written += 1;
            // Write the first frame size as an unsigned integer
            prev_size = frames_[0]->size();
            written += vint::write(prev_size, output);
            // Loop over the remaining frames
            BOOST_FOREACH(value_type frame,
                    std::make_pair(frames_.begin() + 1, frames_.end() - 1))
            {
                std::streamsize size_diff(frame->size() - prev_size);
                prev_size = frame->size();
                // Write the frame size as an offset signed integer
                vint::OffsetInt o_size(vint::s_to_u(size_diff));
                written += vint::write(o_size.first, output, o_size.second);
            }
            break;
        case Block::LACING_FIXED:
            output.put(num_frames);
            if (!output)
            {
                throw tawara::WriteError() << tawara::err_pos(output.tellp());
            }
            written += 1;
            break;
        case LACING_NONE:
            // Nothing to do for no lacing
            break;
    }
    // Write the frames
    BOOST_FOREACH(value_type frame, frames_)
    {
        output.write(&(*frame)[0], frame->size());
        if (!output)
        {
            throw tawara::WriteError() << tawara::err_pos(output.tellp());
        }
        written += frame->size();
    }
    return written;
}


BlockImpl::ReadResult BlockImpl::read(std::istream& input,
        std::streamsize size)
{
    std::streamsize read(0);
    std::streampos start_pos(input.tellg());

    reset();

    if (size < 4)
    {
        // The block data must be at least 4 bytes, which is the size of the
        // smallest possible header.
        throw BadBodySize() << err_el_size(size) << err_pos(start_pos);
    }

    // Read the track number and timecode
    vint::ReadResult res = vint::read(input);
    track_num_ = res.first;
    read += res.second;
    int16_t high_byte(0);
    char tmp(0);
    input.get(tmp);
    high_byte = tmp;
    input.get(tmp);
    timecode_ = (high_byte << 8) | static_cast<unsigned char>(tmp);
    read += 2;
    if (input.fail())
    {
        throw tawara::ReadError() << tawara::err_pos(input.tellg());
    }
    // Read and intepret the flags
    char flags;
    input.get(flags);
    if (input.fail())
    {
        throw tawara::ReadError() << tawara::err_pos(input.tellg());
    }
    read += 1;
    if (flags & 0x10)
    {
        invisible_ = true;
    }
    else
    {
        invisible_ = false;
    }
    if ((flags & 0x60) == 0)
    {
        lacing_ = Block::LACING_NONE;
    }
    else if ((flags & 0x60) == 0x60)
    {
        lacing_ = Block::LACING_EBML;
    }
    else if ((flags & 0x60) == 0x40)
    {
        lacing_ = Block::LACING_FIXED;
    }
    // Prepare the remaining flags to be returned
    flags &= 0x8F;
    // Check there is still data left for the frames
    if (read >= size)
    {
        throw BadBodySize() << err_el_size(size) << err_pos(start_pos);
    }
    // Read the frames according to the lace style used
    char frame_count(0);
    switch (lacing_)
    {
        case Block::LACING_EBML:
            read += read_ebml_laced_frames(input, size - read);
            break;
        case Block::LACING_FIXED:
            // Get the number of frames
            input.get(frame_count);
            if (input.fail())
            {
                throw tawara::ReadError() << tawara::err_pos(input.tellg());
            }
            read += 1;
            read += read_fixed_frames(input, size - read, frame_count);
            break;
        case Block::LACING_NONE:
            // Read the remaining data as a single "fixed-lace" frame
            read += read_fixed_frames(input, size - read, 1);
            break;
    }

    if (read != size)
    {
        throw BadBodySize() << err_el_size(size) << err_pos(start_pos);
    }

    return std::make_pair(read, flags);
}


///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

void BlockImpl::validate() const
{
    if (frames_.empty())
    {
        throw EmptyBlock();
    }

    assert((lacing_ == LACING_NONE && frames_.size() == 1) ||
            lacing_ != LACING_NONE);

    BOOST_FOREACH(value_type f, frames_)
    {
        if (!f)
        {
            // Empty pointer
            throw EmptyFrame();
        }
        if (f->empty())
        {
            // Pointer has a vector, but it is empty
            throw EmptyFrame();
        }
        if (f->size() != frames_[0]->size() && lacing_ == Block::LACING_FIXED)
        {
            // Fixed lacing requires that all frames are the same size
            throw BadLacedFrameSize() << err_frame_size(f->size());
        }
    }
}


void BlockImpl::reset()
{
    track_num_ = 0;
    timecode_ = 0;
    invisible_ = false;
    lacing_ = Block::LACING_NONE;
    frames_.clear();
}


std::streamsize BlockImpl::read_ebml_laced_frames(std::istream& input,
        std::streamsize size)
{
    std::streamsize read(0);

    // Read the frame counts
    char frame_count;
    input.get(frame_count);
    if (input.fail())
    {
        throw tawara::ReadError() << tawara::err_pos(input.tellg());
    }
    read += 1;

    // Read the frame sizes
    std::vector<std::streamsize> sizes;
    // First frame size is just a normal vint
    vint::ReadResult res = vint::read(input);
    if (res.first == 0)
    {
        throw EmptyFrame() << err_pos(input.tellg());
    }
    sizes.push_back(res.first);
    read += res.second;
    // Get each remaining frame size except the last
    std::streamsize leftover(size - read - sizes[0]);
    for (int ii(0); ii < frame_count - 2; ++ii)
    {
        res = vint::read(input);
        // The stored value is the difference from the previous frame size
        int64_t frame_size = sizes[ii] + vint::u_to_s(res);
        if (frame_size == 0)
        {
            throw EmptyFrame() << err_pos(input.tellg());
        }
        else if (frame_size < 0)
        {
            throw BadLacedFrameSize() << err_pos(input.tellg()) <<
                err_frame_size(frame_size);
        }
        sizes.push_back(frame_size);
        leftover -= res.second;
        leftover -= frame_size;
        read += res.second;
    }
    // The last frame size is the left-over data
    sizes.push_back(leftover);
    if (leftover == 0)
    {
        throw EmptyFrame() << err_pos(input.tellg());
    }
    else if (leftover < 0)
    {
        throw BadLacedFrameSize() << err_pos(input.tellg()) <<
            err_frame_size(leftover);
    }

    size -= read;
    BOOST_FOREACH(std::streamsize frame_size, sizes)
    {
        if (read >= size)
        {
            throw EmptyFrame() << err_pos(input.tellg());
        }
        Block::value_type new_frame(new std::vector<char>(frame_size));
        input.read(&(*new_frame)[0], frame_size);
        if (!input)
        {
            throw ReadError() << err_pos(input.tellg()) <<
                err_reqsize(frame_size);
        }
        frames_.push_back(new_frame);
        read += frame_size;
    }

    return read;
}


std::streamsize BlockImpl::read_fixed_frames(std::istream& input,
        std::streamsize size, unsigned int count)
{
    if ((size % count) != 0)
    {
        // Frame sizes are not equal
        throw BadLacedFrameSize() << err_frame_size(size / count);
    }
    std::streamsize frame_size(size / count);
    assert((frame_size * count) == size);

    std::streamsize read(0);
    for(unsigned int ii(0); ii < count; ++ii)
    {
        if (read >= size)
        {
            throw EmptyFrame() << err_pos(input.tellg());
        }
        Block::value_type new_frame(new std::vector<char>(frame_size));
        input.read(&(*new_frame)[0], frame_size);
        if (!input)
        {
            throw ReadError() << err_pos(input.tellg()) <<
                err_reqsize(frame_size);
        }
        frames_.push_back(new_frame);
        read += frame_size;
    }

    return read;
}

