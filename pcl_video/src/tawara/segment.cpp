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

#include <tawara/segment.h>

#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/seek_element.h>
#include <tawara/vint.h>
#include <tawara/void_element.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Segment::Segment(std::streamsize pad_size)
    : MasterElement(ids::Segment), pad_size_(pad_size), size_(pad_size),
    writing_(false)
{
}


///////////////////////////////////////////////////////////////////////////////
// Cluster and block access
///////////////////////////////////////////////////////////////////////////////

Segment::MemClusterIterator Segment::clusters_begin_mem(std::istream& stream)
{
    return Segment::MemClusterIterator(this, stream);
}


Segment::MemClusterIterator Segment::clusters_end_mem(std::istream& stream)
{
    Segment::MemClusterIterator result(this, stream);
    result.cluster_.reset();
    return result;
}


Segment::MemBlockIterator Segment::blocks_begin_mem(std::istream& stream)
{
    return Segment::MemBlockIterator(this,
            Segment::clusters_begin_mem(stream));
}


Segment::MemBlockIterator Segment::blocks_end_mem(std::istream& stream)
{
    return Segment::MemBlockIterator(this,
            Segment::clusters_end_mem(stream));
}


Segment::FileClusterIterator Segment::clusters_begin_file(std::istream& stream)
{
    return Segment::FileClusterIterator(this, stream);
}


Segment::FileClusterIterator Segment::clusters_end_file(std::istream& stream)
{
    Segment::FileClusterIterator result(this, stream);
    result.cluster_.reset();
    return result;
}


Segment::FileBlockIterator Segment::blocks_begin_file(std::istream& stream)
{
    return Segment::FileBlockIterator(this,
            Segment::clusters_begin_file(stream));
}


Segment::FileBlockIterator Segment::blocks_end_file(std::istream& stream)
{
    return Segment::FileBlockIterator(this,
            Segment::clusters_end_file(stream));
}


///////////////////////////////////////////////////////////////////////////////
// Miscellaneous member functions
///////////////////////////////////////////////////////////////////////////////

std::streamsize Segment::to_segment_offset(std::streamsize stream_offset) const
{
    return stream_offset - offset_ - ids::size(ids::Segment) - 8;
}


std::streamsize Segment::to_stream_offset(std::streamsize seg_offset) const
{
    return seg_offset + offset_ + ids::size(ids::Segment) + 8;
}


///////////////////////////////////////////////////////////////////////////////
// I/O
///////////////////////////////////////////////////////////////////////////////

std::streamsize Segment::size() const
{
    // The size of a segment is always written using 8 bytes
    return tawara::ids::size(id_) + 8 + body_size();
}


std::streamsize Segment::finalise(std::iostream& stream)
{
    if (!writing_)
    {
        throw NotWriting();
    }

    // Store the current end of the file
    std::streamoff end_pos(stream.tellp());
    // Store the current read point
    std::streamoff cur_read(stream.tellg());

    // Move to the beginning of the segment, skipping the dummy size value
    stream.seekg(static_cast<std::streamsize>(offset_) +
            ids::size(ids::Segment) + 8, std::ios::beg);
    // Get the size of the void element that is providing padding
    std::streamoff pad_start(stream.tellg());
    std::streamsize pad_size(0);
    ids::ReadResult id_res = ids::read(stream);
    if (id_res.first != ids::Void)
    {
        pad_size = 0;
    }
    else
    {
        VoidElement ve(2);
        ve.read(stream);
        pad_size = ve.size();
    }
    stream.seekp(pad_start);

    // TODO: Add the SegmentInfo to the SeekHead (not having it is not a
    // disaster if it's placed immediately afterwards).
    std::streamsize written(0);
    bool wrote_seekhead(false), wrote_seginfo(false);
    while(written < pad_size && (!wrote_seekhead || !wrote_seginfo))
    {
        if (!wrote_seekhead && index.size() < pad_size - written)
        {
            written += index.write(stream);
            wrote_seekhead = true;
        }
        else if (!wrote_seginfo && info.size() < pad_size - written)
        {
            written += info.write(stream);
            wrote_seginfo = true;
        }
    }
    // Re-do the padding
    if (pad_size - written != 0)
    {
        VoidElement ve(pad_size - written, false);
        ve.write(stream);
    }
    // Move to the end of the file
    stream.seekp(end_pos);
    if (!wrote_seekhead)
    {
        // Write the index at the end
        index.write(stream);
        end_pos = stream.tellp();
    }
    if (!wrote_seginfo)
    {
        // Write the segment info at the end
        info.write(stream);
        end_pos = stream.tellp();
    }

    // Calculate the size of the segment
    size_ = stream.tellp() - offset_ - ids::size(ids::Segment) - 8;
    // Write the size way back at the beginning of the segment
    stream.seekp(static_cast<std::streamsize>(offset_) +
            ids::size(ids::Segment), std::ios::beg);
    write_size(stream);
    // Reset pointers
    stream.seekp(end_pos);
    stream.seekg(cur_read);

    writing_ = false;
    return size();
}


std::streamsize Segment::write_size(std::ostream& output)
{
    return vint::write(body_size(), output, 8);
}


std::streamsize Segment::write_body(std::ostream& output)
{
    writing_ = true;
    // Write some padding
    VoidElement ve(pad_size_, true);
    return ve.write(output);
}


std::streamsize Segment::read_body(std::istream& input, std::streamsize size)
{
    index.clear();
    // +2 for the size values (which must be at least 1 byte each)
    if (size < ids::size(ids::Tracks) + ids::size(ids::Cluster) + 2)
    {
        // 0 is a bad size for the body because there must be at least enough
        // room for the tracks and at least one cluster
        throw BadBodySize() << err_id(id_) << err_pos(offset_);
    }

    // Segments being read cannot be written.
    writing_ = false;

    // Store the segment's size
    size_ = size;

    bool have_seekhead(false);
    bool have_segmentinfo(false);
    bool have_tracks(false);
    bool have_clusters(false);
    std::streamsize read_bytes(0);
    std::streamoff last_read_end(0); // Tracks where the read pointer should be
                                     // placed when this method returns.

    // Check if the first child is the meta-seek
    ids::ReadResult id_res = ids::read(input);
    read_bytes += id_res.second;
    if (id_res.first == ids::SeekHead)
    {
        have_seekhead = true;
        // Read the SeekHead element
        read_bytes += index.read(input);
        if (index.find(ids::Info) != index.end())
        {
            have_segmentinfo = true;
        }
        if (index.find(ids::Tracks) != index.end())
        {
            have_tracks = true;
        }
        if (index.find(ids::Cluster) != index.end())
        {
            have_clusters = true;
        }
    }
    else
    {
        // Rewind back to the ID for the search
        input.seekg(-id_res.second, std::ios::cur);
        read_bytes -= id_res.second;
    }
    last_read_end = input.tellg();

    // Search for the other necessary elements
    while (read_bytes < size &&
        (!have_seekhead || !have_segmentinfo || !have_tracks || !have_clusters))
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::SeekHead:
                if (have_seekhead)
                {
                    throw MultipleSeekHeads() << err_pos(offset_);
                }
                // Read the SeekHead element
                read_bytes += index.read(input);
                last_read_end = input.tellg();
                if (index.find(ids::Info) != index.end())
                {
                    have_segmentinfo = true;
                }
                if (index.find(ids::Tracks) != index.end())
                {
                    have_tracks = true;
                }
                if (index.find(ids::Cluster) != index.end())
                {
                    have_clusters = true;
                }
                break;
            case ids::Info:
                have_segmentinfo = true;
                index.insert(std::make_pair(ids::Info,
                    to_segment_offset(
                        static_cast<std::streamsize>(input.tellg()) -
                        id_res.second)));
                read_bytes += skip_read(input, false);
                break;
            case ids::Tracks:
                have_tracks = true;
                index.insert(std::make_pair(ids::Tracks,
                    to_segment_offset(
                        static_cast<std::streamsize>(input.tellg()) -
                        id_res.second)));
                read_bytes += skip_read(input, false);
                break;
            case ids::Cluster:
                if (!have_clusters)
                {
                    // Only store the first cluster in the index
                    have_clusters = true;
                    index.insert(std::make_pair(ids::Cluster,
                        to_segment_offset(
                            static_cast<std::streamsize>(input.tellg()) -
                            id_res.second)));
                }
                read_bytes += skip_read(input, false);
                break;
            case ids::Cues:
            case ids::Attachments:
            case ids::Chapters:
            case ids::Tags:
                index.insert(std::make_pair(id,
                    to_segment_offset(
                        static_cast<std::streamsize>(input.tellg()) -
                        id_res.second)));
                read_bytes += skip_read(input, false);
                break;
            case ids::Void:
                read_bytes += skip_read(input, false);
                break;
            default:
                throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                    // The cast here makes Apple's LLVM compiler happy
                    err_pos(static_cast<std::streamsize>(input.tellg()) -
                            id_res.second);
        }
    }

    if (!have_segmentinfo)
    {
        throw NoSegmentInfo() << err_pos(offset_);
    }
    else
    {
        input.seekg(to_stream_offset(index.find(ids::Info)->second));
        // Check the ID is correct
        ids::ReadResult id_res = ids::read(input);
        if (id_res.first != ids::Info)
        {
            throw NoSegmentInfo() << err_pos(index.find(ids::Info)->second);
        }
        info.read(input);
        if (input.tellg() > last_read_end)
        {
            last_read_end = input.tellg();
        }
    }
    if (!have_tracks)
    {
        throw NoTracks() << err_pos(offset_);
    }
    if (!have_clusters)
    {
        throw NoClusters() << err_pos(offset_);
    }

    input.seekg(last_read_end);
    return last_read_end - offset_ - ids::size(ids::Segment) - 8;
}

