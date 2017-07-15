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

#include <tawara/cluster.h>

#include <boost/foreach.hpp>
#include <numeric>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Cluster::Cluster(uint64_t timecode)
    : MasterElement(ids::Cluster),
    timecode_(ids::Timecode, timecode), position_(ids::Position, 0),
    prev_size_(ids::PrevSize, 0), writing_(false)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

uint64_t Cluster::position() const
{
    throw NotImplemented();
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize Cluster::size() const
{
    // The size of a cluster is always written using 8 bytes
    return tawara::ids::size(id_) + 8 + body_size();
}


std::streamsize add_size(std::streamsize x, SilentTrackNumber stn)
{
    return x + stn.size();
}

std::streamsize Cluster::meta_size() const
{
    std::streamsize result(timecode_.size());

    if (!silent_tracks_.empty())
    {
        result += ids::size(ids::SilentTracks);
        std::streamsize st_size(std::accumulate(silent_tracks_.begin(),
                    silent_tracks_.end(), 0, std::cref(add_size)));
        result += vint::size(st_size) + st_size;
    }
    if (position_ != 0)
    {
        result += position_.size();
    }
    if (prev_size_ != 0)
    {
        result += prev_size_.size();
    }

    return result;
}


std::streamsize Cluster::write_size(std::ostream& output)
{
    return vint::write(body_size(), output, 8);
}


std::streamsize Cluster::write_body(std::ostream& output)
{
    std::streamsize written(0);
    writing_ = true;

    written += timecode_.write(output);
    if (!silent_tracks_.empty())
    {
        written += ids::write(ids::SilentTracks, output);
        std::streamsize st_size(std::accumulate(silent_tracks_.begin(),
                    silent_tracks_.end(), 0, std::cref(add_size)));
        written += tawara::vint::write(st_size, output);
        BOOST_FOREACH(SilentTrackNumber& stn, silent_tracks_)
        {
            written += stn.write(output);
        }
    }
    if (position_ != 0)
    {
        written += position_.write(output);
    }
    if (prev_size_ != 0)
    {
        written += prev_size_.write(output);
    }

    return written;
}


std::streamsize Cluster::read_body(std::istream& input,
        std::streamsize size)
{
    // Reset to defaults
    reset();
    // Cannot write a cluster being read
    writing_ = false;

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    bool have_timecode(false);
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch (id)
        {
            case ids::Timecode:
                read_bytes += timecode_.read(input);
                have_timecode = true;
                break;
            case ids::SilentTracks:
                read_bytes += read_silent_tracks(input);
                break;
            case ids::Position:
                read_bytes += position_.read(input);
                break;
            case ids::PrevSize:
                read_bytes += prev_size_.read(input);
                break;
            case ids::SimpleBlock:
            case ids::BlockGroup:
                // Rewind to the element ID value
                input.seekg(-id_res.second, std::ios::cur);
                read_bytes -= id_res.second;
                // Read all the blocks - this will use up the rest of the block
                read_bytes += read_blocks(input, size - read_bytes);
                break;
            default:
                throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                    // The cast here makes Apple's LLVM compiler happy
                    err_pos(static_cast<std::streamsize>(input.tellg()) -
                            id_res.second);
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
        throw MissingChild() << err_id(ids::Timecode) << err_par_id(id_) <<
            err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

std::streamsize Cluster::read_silent_tracks(std::istream& input)
{
    std::streampos el_start(input.tellg());
    // Get the element's body size
    vint::ReadResult result = tawara::vint::read(input);
    std::streamsize body_size(result.first);
    std::streamsize size_size(result.second);
    std::streamsize read_bytes(result.second);
    // Read elements until the body is exhausted
    while (read_bytes < size_size + body_size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::SilentTrackNumber)
        {
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
        SilentTrackNumber stn(0);
        read_bytes += stn.read(input);
        silent_tracks_.push_back(stn);
    }
    if (read_bytes != size_size + body_size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(body_size) <<
            err_pos(el_start);
    }

    return read_bytes;
}


void Cluster::reset()
{
    timecode_ = 0;
    silent_tracks_.clear();
    position_ = 0;
    prev_size_ = 0;
}

