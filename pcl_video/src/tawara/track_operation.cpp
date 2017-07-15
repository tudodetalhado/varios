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

#include <tawara/track_operation.h>

#include <boost/foreach.hpp>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

TrackJoinBlocks::TrackJoinBlocks()
    : TrackOperationBase(ids::TrackJoinBlocks)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void TrackJoinBlocks::append(uint64_t uid)
{
    if (uid == 0)
    {
        // Zero-value UIDs are illegal
        throw ValueOutOfRange() << err_id(ids::TrackJoinUID) <<
            err_par_id(id_);
    }
    uids_.push_back(UIntElement(ids::TrackJoinUID, uid));
}


uint64_t TrackJoinBlocks::remove(unsigned int pos)
{
    UIntElement uid = uids_[pos];
    uids_.erase(uids_.begin() + pos);
    return uid.value();
}


uint64_t TrackJoinBlocks::operator[](unsigned int pos) const
{
    return uids_[pos].value();
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize TrackJoinBlocks::body_size() const
{
    std::streamsize size(0);
    BOOST_FOREACH(UIntElement el, uids_)
    {
        size += el.size();
    }
    return size;
}


std::streamsize TrackJoinBlocks::write_body(std::ostream& output)
{
    assert(!uids_.empty());

    std::streamsize written(0);
    BOOST_FOREACH(UIntElement uid, uids_)
    {
        if (uid.value() == 0)
        {
            // Zero-value UIDs are illegal
            throw ValueOutOfRange() << err_id(ids::TrackJoinUID) <<
                err_par_id(id_);
        }
        written += uid.write(output);
    }
    return written;
}


std::streamsize TrackJoinBlocks::read_body(std::istream& input,
        std::streamsize size)
{
    // Clear the UIDs list
    uids_.clear();
    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = tawara::ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::TrackJoinUID)
        {
            // Only TrackJoinUID elements may be in the TrackJoinBlocks element
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                // The cast here makes Apple's LLVM compiler happy
                err_pos(static_cast<std::streamsize>(input.tellg()) -
                        id_res.second);
        }
        // Read the body
        UIntElement uid(ids::Null, 0);
        read_bytes += uid.read(input);
        if (uid.value() == 0)
        {
            // Zero-value UIDs are illegal
            throw ValueOutOfRange() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
        uids_.push_back(uid);
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (uids_.empty())
    {
        // Must have read at least one TrackJoinUID
        throw MissingChild() << err_id(ids::TrackJoinUID) << err_par_id(id_) <<
            err_pos(offset_);
    }

    return read_bytes;
}

