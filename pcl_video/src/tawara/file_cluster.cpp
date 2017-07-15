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

#include <tawara/file_cluster.h>

#include <tawara/block_group.h>
#include <tawara/exceptions.h>
#include <tawara/simple_block.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

FileCluster::FileCluster(uint64_t timecode)
    : Cluster(timecode), ostream_(0), istream_(0), blocks_start_pos_(0),
    blocks_end_pos_(0)
{
}

///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

FileCluster::Iterator FileCluster::begin()
{
    return Iterator(this, *istream_, blocks_start_pos_);
}


FileCluster::Iterator FileCluster::end()
{
    return Iterator(this, *istream_, blocks_end_pos_);
}


///////////////////////////////////////////////////////////////////////////////
// I/O (Cluster interface)
///////////////////////////////////////////////////////////////////////////////

bool FileCluster::empty() const
{
    return blocks_size() == 0;
}


FileCluster::size_type FileCluster::count() const
{
    assert(istream_ && "istream_ has not been initialised");

    FileCluster::size_type result(0);
    // Remember the current read position
    std::streampos cur_read(istream_->tellg());
    // Jump to the beginning of the blocks
    istream_->seekg(blocks_start_pos_);
    // Start reading through the blocks, skipping the body of each
    std::streamsize read_bytes(0);
    std::streamsize size(blocks_end_pos_ - blocks_start_pos_);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(*istream_);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id == ids::SimpleBlock || id == ids::BlockGroup)
        {
            ++result;
            // Skip the body
            read_bytes += skip_read(*istream_, false);
        }
        else
        {
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                // The cast here makes Apple's LLVM compiler happy
                err_pos(static_cast<std::streamsize>(istream_->tellg()) -
                        id_res.second);
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    // Return to the original read position
    istream_->seekg(cur_read);
    // Return the count
    return result;
}


void FileCluster::clear()
{
    assert(false && "Not implemented");
    // Making this function work for clusters that are read and written will
    // require that clusters are read using an iostream, not an istream.
    // Aspect-oriented design using type traits might make this easier.
}


void FileCluster::erase(FileCluster::Iterator position)
{
    assert(false && "Not implemented");
    // Making this function work for clusters that are read and written will
    // require that clusters are read using an iostream, not an istream.
    // Aspect-oriented design using type traits might make this easier.
}


void FileCluster::erase(FileCluster::Iterator first, FileCluster::Iterator last)
{
    assert(false && "Not implemented");
    // Making this function work for clusters that are read and written will
    // require that clusters are read using an iostream, not an istream.
    // Aspect-oriented design using type traits might make this easier.
}


void FileCluster::push_back(FileCluster::value_type const& value)
{
    // TODO: Make this a compile-time error somehow (type traits?)
    if (!writing_)
    {
        throw NotWriting();
    }
    assert(ostream_ != 0 && "ostream_ was not initialised");

    // Preserve the current write position
    //std::streampos cur_pos(ostream_->tellp());
    // Jump to the cluster's current write position
    ostream_->seekp(blocks_end_pos_);
    // Write the block
    value->write(*ostream_);
    // Update the cluster's current write position
    blocks_end_pos_ = ostream_->tellp();
    // Return to the original write position
    //ostream_->seekp(cur_pos);
    // TODO: update the block size continuously so that it can be written
    // during finalise() without needing to be calculated from the file write
    // pointer position at that time.
    // TODO: benchmark the impact constantly seeking back to the previous
    // position has on performance to determine if it is worth doing or not.
}


std::streamsize FileCluster::finalise(std::ostream& output)
{
    // TODO: Make this a compile-time error somehow (type traits?)
    if (!writing_)
    {
        throw NotWriting();
    }

    // Preserve the current write position
    std::streampos cur_pos(output.tellp());

    // Go back and write the cluster's actual size in the element header
    // actual size = current write position (i.e. end of the
    // cluster) - cluster's start position - ID - 8-byte size.
    std::streamsize size(blocks_end_pos_ - offset_ - ids::size(id_) - 8);
    output.seekp(static_cast<std::streamsize>(offset_) +
            ids::size(ids::Cluster));
    write_size(output);

    // Return to the original write position
    output.seekp(cur_pos);

    writing_ = false;
    return ids::size(id_) + 8 + size;
}


std::streamsize FileCluster::blocks_size() const
{
    return blocks_end_pos_ - blocks_start_pos_;
}


std::streamsize FileCluster::write(std::ostream& output)
{
    // TODO: Make this a compile-time error somehow (type traits?)
    assert(!writing_ && "Already writing");
    // Store a pointer to the stream for push_back() to use.
    ostream_ = &output;
    std::streamsize result = Element::write(output);
    // Make a note of where to write the first block.
    blocks_start_pos_ = blocks_end_pos_ = output.tellp();
    return result;
}


std::streamsize FileCluster::read_blocks(std::istream& input,
        std::streamsize size)
{
    // Remember the stream for use in other functions
    istream_ = &input;
    // Record the start position of the blocks
    blocks_start_pos_ = input.tellg();
    // Jump to the end of the blocks
    input.seekg(size, std::ios::cur);
    // Record the end position of the blocks
    blocks_end_pos_ = input.tellg();
    // Return the total size of the block elements to pretend they've been read
    return size;
}

