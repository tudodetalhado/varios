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

#include <tawara/memory_cluster.h>

#include <boost/foreach.hpp>
#include <numeric>
#include <tawara/block_group.h>
#include <tawara/exceptions.h>
#include <tawara/simple_block.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

MemoryCluster::MemoryCluster(uint64_t timecode)
    : Cluster(timecode)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

MemoryCluster::Iterator MemoryCluster::begin()
{
    return Iterator(blocks_.begin());
}


MemoryCluster::ConstIterator MemoryCluster::begin() const
{
    return ConstIterator(blocks_.begin());
}


MemoryCluster::Iterator MemoryCluster::end()
{
    return Iterator(blocks_.end());
}


MemoryCluster::ConstIterator MemoryCluster::end() const
{
    return ConstIterator(blocks_.end());
}


///////////////////////////////////////////////////////////////////////////////
// I/O (Cluster interface)
///////////////////////////////////////////////////////////////////////////////

std::streamsize MemoryCluster::finalise(std::ostream& output)
{
    if (!writing_)
    {
        throw NotWriting();
    }

    std::streamsize written(0);

    // Write the blocks to the file
    BOOST_FOREACH(BlockElement::Ptr& block, blocks_)
    {
        written += block->write(output);
    }

    // Go back and write the cluster's actual size in the element header
    std::streampos cluster_end(output.tellp());
    std::streamsize size = cluster_end - offset_ - 8 - ids::size(id_);
    output.seekp(static_cast<std::streamsize>(offset_) +
            ids::size(ids::Cluster));
    write_size(output);
    // And return back to the end of the cluster again
    output.seekp(cluster_end);

    writing_ = false;
    return ids::size(id_) + 8 + meta_size() + written;
}


std::streamsize add_size(std::streamsize x, BlockElement::Ptr b)
{
    return x + b->size();
}

std::streamsize MemoryCluster::blocks_size() const
{
    return std::accumulate(blocks_.begin(), blocks_.end(), 0,
            std::cref(add_size));
}


std::streamsize MemoryCluster::read_blocks(std::istream& input,
        std::streamsize size)
{
    // Clear any existing blocks
    blocks_.clear();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        BlockElement::Ptr new_block;
        if (id == ids::SimpleBlock)
        {
            BlockElement::Ptr new_block(new SimpleBlock(0, 0));
            read_bytes += new_block->read(input);
            blocks_.push_back(new_block);
        }
        else if (id == ids::BlockGroup)
        {
            BlockElement::Ptr new_block(new BlockGroup(0, 0));
            read_bytes += new_block->read(input);
            blocks_.push_back(new_block);
        }
        else
        {
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

    return read_bytes;
}

