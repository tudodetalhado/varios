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

#include <tawara/block_additions.h>

#include <boost/foreach.hpp>
#include <tawara/binary_element.h>
#include <tawara/el_ids.h>
#include <tawara/ebml_int.h>
#include <tawara/exceptions.h>
#include <tawara/uint_element.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

BlockAdditions::BlockAdditions()
    : MasterElement(ids::BlockAdditions)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void BlockAdditions::push_back(value_type const& value)
{
    if (value->first ==  0)
    {
        throw ValueOutOfRange() << err_id(ids::BlockAddID) <<
            err_par_id(ids::BlockMore);
    }
    additions_.push_back(value);
}

///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(BlockAdditions const& lhs, BlockAdditions const& rhs)
{
    return lhs.additions_ == rhs.additions_;
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize BlockAdditions::body_size() const
{
    std::streamsize result(0);
    BOOST_FOREACH(value_type add, additions_)
    {
        std::streamsize this_size(0);
        // BlockAddID defaults to 1
        if (add->first != 1)
        {
            this_size += ids::size(ids::BlockAddID) +
                vint::size(ebml_int::size_u(add->first)) +
                ebml_int::size_u(add->first);
        }
        this_size += ids::size(ids::BlockAdditional) +
            vint::size(add->second.size()) + add->second.size();
        result += ids::size(ids::BlockMore) + vint::size(this_size) + this_size;
    }
    return result;
}


std::streamsize BlockAdditions::write_body(std::ostream& output)
{
    std::streamsize written(0);

    if (additions_.empty())
    {
        throw EmptyBlockAdditionsElement();
    }

    BOOST_FOREACH(value_type add, additions_)
    {
        if (add->first ==  0)
        {
            throw ValueOutOfRange() << err_id(ids::BlockAddID) <<
                err_par_id(ids::BlockMore);
        }
        UIntElement add_id(ids::BlockAddID, add->first, 1);
        BinaryElement additional(ids::BlockAdditional, add->second);

        written += ids::write(ids::BlockMore, output);
        std::streamsize this_size(0);
        // BlockAddID defaults to 1
        if (!add_id.is_default())
        {
            this_size += add_id.size();
        }
        this_size += additional.size();
        written += vint::write(this_size, output);
        if (!add_id.is_default())
        {
            written += add_id.write(output);
        }
        written += additional.write(output);
    }

    return written;
}


std::streamsize BlockAdditions::read_body(std::istream& input,
        std::streamsize size)
{
    // Clear the additions
    additions_.clear();
    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::BlockMore)
        {
            // Only BlockMore elements may be in the BlockAdditions element
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                err_pos(input.tellg());
        }
        // Read the size
        vint::ReadResult size_res = vint::read(input);
        read_bytes += size_res.second;
        read_bytes += read_addition(input, size_res.first);
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (additions_.empty())
    {
        // No BlockMores is bad.
        throw EmptyBlockAdditionsElement() << err_pos(offset_);
    }

    return read_bytes;
}


std::streamsize BlockAdditions::read_addition(std::istream& input,
        std::streamsize size)
{
    std::streampos el_start(input.tellg());
    UIntElement addid(ids::BlockAddID, 1, 1);
    BinaryElement additional(ids::BlockAdditional, std::vector<char>());
    bool have_addid(false), have_additional(false);
    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        if (have_addid && have_additional)
        {
            // If both children have been read, why is there still body left?
            throw BadBodySize() << err_id(id_) << err_el_size(size) <<
                err_pos(el_start);
        }
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch (id)
        {
            case ids::BlockAddID:
                read_bytes += addid.read(input);
                have_addid = true;
                if (addid == 0)
                {
                    throw ValueOutOfRange() << err_id(ids::BlockAddID) <<
                        err_par_id(ids::BlockMore) << err_pos(input.tellg());
                }
                break;
            case ids::BlockAdditional:
                read_bytes += additional.read(input);
                have_additional = true;
                break;
            default:
                throw InvalidChildID() << err_id(ids::BlockMore) <<
                    err_par_id(id_) <<
                    // The cast here makes Apple's LLVM compiler happy
                    err_pos(static_cast<std::streamsize>(input.tellg()) -
                            id_res.second);
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(ids::BlockMore) << err_el_size(size) <<
            err_pos(el_start);
    }
    if (!have_additional)
    {
        throw MissingChild() << err_id(ids::BlockAdditional) <<
            err_par_id(ids::BlockMore) << err_pos(el_start);
    }
    value_type new_addition(new Addition(addid.value(), additional.value()));
    additions_.push_back(new_addition);

    return read_bytes;
}

