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

#include <tawara/seek_element.h>

#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

SeekElement::SeekElement(ids::ID id, std::streampos offset)
    : MasterElement(ids::Seek),
    indexed_id_(ids::SeekID, tawara::ids::encode(id)),
    offset_(ids::SeekPosition, offset)
{
    assert(offset >= 0);
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

ids::ID SeekElement::indexed_id() const
{
    std::vector<char> bin(indexed_id_.value());
    ids::DecodeResult r(ids::decode(bin));
    return r.first;
}


void SeekElement::indexed_id(ids::ID id)
{
    indexed_id_.value(ids::encode(id));
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize SeekElement::body_size() const
{
    return indexed_id_.size() + offset_.size();
}


std::streamsize SeekElement::write_body(std::ostream& output)
{
    std::streamsize written(0);
    written += indexed_id_.write(output);
    written += offset_.write(output);
    return written;
}


std::streamsize SeekElement::read_body(std::istream& input,
        std::streamsize size)
{
    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    bool have_id(false), have_offset(false);
    while (read_bytes < size)
    {
        if (have_id && have_offset)
        {
            // If both children have been read, why is there still body left?
            throw BadBodySize() << err_id(id_) << err_el_size(size) <<
                err_pos(offset_);
        }
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::SeekID:
                read_bytes += indexed_id_.read(input);
                have_id = true;
                break;
            case ids::SeekPosition:
                read_bytes += offset_.read(input);
                have_offset = true;
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

    if (!have_id)
    {
        throw MissingChild() << err_id(ids::SeekID) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (!have_offset)
    {
        throw MissingChild() << err_id(ids::SeekPosition) <<
            err_par_id(id_) << err_pos(offset_);
    }

    return read_bytes;
}

