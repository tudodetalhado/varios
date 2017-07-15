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

#include <tawara/tracks.h>

#include <algorithm>
#include <boost/foreach.hpp>
#include <functional>
#include <stdexcept>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;
using namespace std::placeholders;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Tracks::Tracks()
    : MasterElement(ids::Tracks)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

Tracks::mapped_type& Tracks::operator[](Tracks::key_type const& key)
{
    if (entries_.find(key) == entries_.end())
    {
        std::stringstream str;
        str << key;
        throw std::out_of_range(str.str());
    }
    return entries_[key];
}


Tracks::mapped_type const& Tracks::operator[](Tracks::key_type const& key) const
{
    if (entries_.find(key) == entries_.end())
    {
        std::stringstream str;
        str << key;
        throw std::out_of_range(str.str());
    }
    // Cannot use entries_[key] because it is not a const function.
    return entries_.find(key)->second;
}


std::pair<Tracks::iterator, bool> Tracks::insert(
        Tracks::mapped_type const& value)
{
    verify_not_duplicate(value);
    value_type new_val(value->number(), value);
    return entries_.insert(new_val);
}


void Tracks::insert(const_iterator first, const_iterator last)
{
    const_iterator ii(first);
    while(ii != last)
    {
        verify_not_duplicate(ii->second);
        ++ii;
    }
    entries_.insert(first, last);
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(Tracks const& lhs, Tracks const& rhs)
{
    return lhs.entries_ == rhs.entries_;
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize Tracks::body_size() const
{
    std::streamsize size(0);
    BOOST_FOREACH(value_type te, entries_)
    {
        size += te.second->size();
    }
    return size;
}


std::streamsize Tracks::write_body(std::ostream& output)
{
    // There must be at least one TrackEntry
    if (entries_.empty())
    {
        throw EmptyTracksElement();
    }

    // Check there are no duplicate track numbers or UIDs
    validate_entries();

    std::streamsize written(0);
    BOOST_FOREACH(value_type te, entries_)
    {
        written += te.second->write(output);
    }
    return written;
}


std::streamsize Tracks::read_body(std::istream& input, std::streamsize size)
{
    // Clear the entries
    entries_.clear();
    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::TrackEntry)
        {
            // Only TrackEntry elements may be in the Tracks element
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
        // Read the body
        TrackEntry::Ptr entry(new TrackEntry(1, 1, "Empty"));
        read_bytes += entry->read(input);
        insert(entry);
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (entries_.empty())
    {
        // No TrackEntries is bad.
        throw EmptyTracksElement() << err_pos(offset_);
    }
    // Check there are no duplicate track numbers or UIDs
    validate_entries();

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

void Tracks::validate_entries() const
{
    std::vector<uint64_t> seen_numbers;
    std::vector<uint64_t> seen_uids;
    BOOST_FOREACH(value_type te, entries_)
    {
        if (std::find(seen_numbers.begin(), seen_numbers.end(),
                    te.second->number()) != seen_numbers.end())
        {
            throw DuplicateTrackNumber() << err_track_num(te.second->number());
        }
        if (std::find(seen_uids.begin(), seen_uids.end(),
                    te.second->uid()) != seen_uids.end())
        {
            throw DuplicateUID() << err_int_uid(te.second->uid());
        }
        seen_numbers.push_back(te.second->number());
        seen_uids.push_back(te.second->uid());
    }
}


bool comp_uid(Tracks::value_type entry, uint64_t uid)
{
    return entry.second->uid() == uid;
}

void Tracks::verify_not_duplicate(TrackEntry::Ptr entry) const
{
    if (entries_.find(entry->number()) != entries_.end())
    {
        throw DuplicateTrackNumber() << err_track_num(entry->number());
    }
    if (std::find_if(entries_.begin(), entries_.end(),
                std::bind2nd(std::cref(comp_uid), entry->uid())) != entries_.end())
    {
        throw DuplicateUID() << err_int_uid(entry->uid());
    }
}

