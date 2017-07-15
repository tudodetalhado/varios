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

#include <tawara/cues.h>

#include <boost/foreach.hpp>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// CueTrackPosition Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

CueTrackPosition::CueTrackPosition()
    : MasterElement(ids::CueTrackPosition), track_(ids::CueTrack, 1),
    cluster_pos_(ids::CueClusterPosition, 0),
    block_num_(ids::CueBlockNumber, 1, 1),
    codec_state_(ids::CueCodecState, 0, 0)
{
}

CueTrackPosition::CueTrackPosition(uint64_t track, uint64_t cluster_pos)
    : MasterElement(ids::CueTrackPosition), track_(ids::CueTrack, track),
    cluster_pos_(ids::CueClusterPosition, cluster_pos),
    block_num_(ids::CueBlockNumber, 1, 1),
    codec_state_(ids::CueCodecState, 0, 0)
{
    if (track == 0)
    {
        throw ValueOutOfRange() << err_id(ids::CueTrack) <<
            err_par_id(ids::CueTrackPosition);
    }
}


///////////////////////////////////////////////////////////////////////////////
// CueTrackPosition accessors
///////////////////////////////////////////////////////////////////////////////

void CueTrackPosition::track(uint64_t track)
{
    if (track == 0)
    {
        throw ValueOutOfRange() << err_id(ids::CueTrack) <<
            err_par_id(ids::CueTrackPosition);
    }
    track_ = track;
}


void CueTrackPosition::block_num(uint64_t block_num)
{
    if (block_num == 0)
    {
        throw ValueOutOfRange() << err_id(ids::CueTrack) <<
            err_par_id(ids::CueTrackPosition);
    }
    block_num_ = block_num;
}


///////////////////////////////////////////////////////////////////////////////
// CueTrackPosition operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(CueTrackPosition const& lhs, CueTrackPosition const& rhs)
{
    return lhs.track_ == rhs.track_ &&
        lhs.cluster_pos_ == rhs.cluster_pos_ &&
        lhs.block_num_ == rhs.block_num_ &&
        lhs.codec_state_ == rhs.codec_state_ &&
        lhs.ref_blocks_ == rhs.ref_blocks_;
}


///////////////////////////////////////////////////////////////////////////////
// CueTrackPosition Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize CueTrackPosition::body_size() const
{
    std::streamsize size = track_.size() + cluster_pos_.size();
    if (!block_num_.is_default())
    {
        size += block_num_.size();
    }
    if (!codec_state_.is_default())
    {
        size += codec_state_.size();
    }
    BOOST_FOREACH(uint64_t r, ref_blocks_)
    {
        UIntElement reftime(ids::CueRefTime, r);
        size += ids::size(ids::CueReference) + vint::size(reftime.size()) +
            reftime.size();
    }
    return size;
}


std::streamsize CueTrackPosition::write_body(std::ostream& output)
{
    assert(track_ != 0);
    assert(block_num_ != 0);

    std::streamsize written = track_.write(output) + cluster_pos_.write(output);
    if (!block_num_.is_default())
    {
        written += block_num_.write(output);
    }
    if (!codec_state_.is_default())
    {
        written += codec_state_.write(output);
    }
    BOOST_FOREACH(uint64_t r, ref_blocks_)
    {
        UIntElement reftime(ids::CueRefTime, r);
        written += ids::write(ids::CueReference, output);
        written += vint::write(reftime.size(), output);
        written += reftime.write(output);
    }
    return written;
}


std::streamsize CueTrackPosition::read_body(std::istream& input,
        std::streamsize size)
{
    // Clear the data
    reset();

    std::streamsize read_bytes(0);
    vint::ReadResult read_res;
    bool have_track(false), have_cluster_pos(false);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::CueTrack:
                read_bytes += track_.read(input);
                if (track_ == 0)
                {
                    throw ValueOutOfRange() << err_id(ids::CueTrack) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                have_track = true;
                break;
            case ids::CueClusterPosition:
                read_bytes += cluster_pos_.read(input);
                have_cluster_pos = true;
                break;
            case ids::CueBlockNumber:
                read_bytes += block_num_.read(input);
                if (block_num_ == 0)
                {
                    throw ValueOutOfRange() << err_id(ids::CueBlockNumber) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::CueCodecState:
                read_bytes += codec_state_.read(input);
                break;
            case ids::CueReference:
                read_res = vint::read(input);
                read_bytes += read_res.second;
                read_bytes += read_cue_reference(input, read_res.first);
                break;
            default:
                throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                    err_pos(input.tellg());
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (!have_track)
    {
        throw MissingChild() << err_id(ids::CueTrack) <<
            err_par_id(ids::CueTrackPosition) << err_pos(offset_);
    }
    if (!have_cluster_pos)
    {
        throw MissingChild() << err_id(ids::CueClusterPosition) <<
            err_par_id(ids::CueTrackPosition) << err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// CueTrackPosition private functions
///////////////////////////////////////////////////////////////////////////////

std::streamsize CueTrackPosition::read_cue_reference(std::istream& input,
        std::streamsize size)
{
    ids::ReadResult id_res = ids::read(input);
    if (id_res.first != ids::CueRefTime)
    {
        throw InvalidChildID() << err_id(id_res.first) << err_par_id(id_) <<
            err_pos(input.tellg());
    }
    UIntElement rt(ids::CueRefTime, 0);
    std::streamsize read_bytes = rt.read(input);
    ref_blocks_.push_back(rt.value());
    return read_bytes += id_res.second;
}


void CueTrackPosition::reset()
{
    track_ = 1;
    cluster_pos_ = 0;
    block_num_ = block_num_.get_default();
    codec_state_ = codec_state_.get_default();
    ref_blocks_.clear();
}


///////////////////////////////////////////////////////////////////////////////
// CuePoint Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

CuePoint::CuePoint()
    : MasterElement(ids::CuePoint), timecode_(ids::CueTime, 0)
{
}


CuePoint::CuePoint(uint64_t timecode)
    : MasterElement(ids::CuePoint), timecode_(ids::CueTime, timecode)
{
}


///////////////////////////////////////////////////////////////////////////////
// CuePoint operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(CuePoint const& lhs, CuePoint const& rhs)
{
    return lhs.timecode_ == rhs.timecode_ &&
        lhs.positions_ == rhs.positions_;
}


///////////////////////////////////////////////////////////////////////////////
// CuePoint Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize CuePoint::body_size() const
{
    std::streamsize size(0);
    size += timecode_.size();
    BOOST_FOREACH(value_type p, positions_)
    {
        size += p.size();
    }
    return size;
}


std::streamsize CuePoint::write_body(std::ostream& output)
{
    // There must be at least one CueTrackPosition child
    if (positions_.empty())
    {
        throw EmptyCuePointElement();
    }

    std::streamsize written = timecode_.write(output);
    BOOST_FOREACH(value_type p, positions_)
    {
        written += p.write(output);
    }
    return written;
}


std::streamsize CuePoint::read_body(std::istream& input, std::streamsize size)
{
    // Clear the data
    timecode_ = 0;
    positions_.clear();

    std::streamsize read_bytes(0);
    bool have_timecode(false);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id == ids::CueTime)
        {
            read_bytes += timecode_.read(input);
            have_timecode = true;
        }
        else if (id == ids::CueTrackPosition)
        {
            CueTrackPosition p(1, 1);
            read_bytes += p.read(input);
            positions_.push_back(p);
        }
        else
        {
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (!have_timecode)
    {
        throw MissingChild() << err_id(ids::CueTime) <<
            err_par_id(ids::CuePoint) << err_pos(offset_);
    }
    if (positions_.empty())
    {
        throw EmptyCuePointElement() << err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Cues Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Cues::Cues()
    : MasterElement(ids::Cues)
{
}


///////////////////////////////////////////////////////////////////////////////
// Cues Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(Cues const& lhs, Cues const& rhs)
{
    return lhs.cues_ == rhs.cues_;
}


///////////////////////////////////////////////////////////////////////////////
// Cues Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize Cues::body_size() const
{
    std::streamsize size(0);
    BOOST_FOREACH(value_type c, cues_)
    {
        size += c.second.size();
    }
    return size;
}


std::streamsize Cues::write_body(std::ostream& output)
{
    // There must be at least one CuePoint
    if (cues_.empty())
    {
        throw EmptyCuesElement();
    }

    std::streamsize written(0);
    BOOST_FOREACH(value_type c, cues_)
    {
        written += c.second.write(output);
    }
    return written;
}


std::streamsize Cues::read_body(std::istream& input, std::streamsize size)
{
    // Clear the cue points
    cues_.clear();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::CuePoint)
        {
            // Only CuePoint elements may be in the Cues element
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
        // Read the body
        CuePoint cuepoint(0);
        read_bytes += cuepoint.read(input);
        std::pair<iterator, bool> res = insert(cuepoint);
        if (!res.second)
        {
            throw DuplicateTimecode() << err_pos(input.tellg());
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (cues_.empty())
    {
        // No CuePoints is bad.
        throw EmptyCuesElement() << err_pos(offset_);
    }

    return read_bytes;
}

