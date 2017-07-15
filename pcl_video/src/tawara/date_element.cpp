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

#include <tawara/date_element.h>

#include <tawara/ebml_int.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

DateElement::DateElement(uint32_t id, int64_t value)
    : PrimitiveElement<int64_t>(id, value)
{
}


DateElement::DateElement(uint32_t id, int64_t value, int64_t default_value)
    : PrimitiveElement<int64_t>(id, value, default_value)
{
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

DateElement& DateElement::operator=(int64_t const& rhs)
{
    value_ = rhs;
    return *this;
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize DateElement::body_size() const
{
    // Dates are always stored as the full 8 bytes
    return 8;
}


std::streamsize DateElement::write_body(std::ostream& output)
{
    output.write(reinterpret_cast<char*>(&value_), 8);
    if (!output)
    {
        throw WriteError() << err_pos(output.tellp());
    }
    return 8;
}


std::streamsize DateElement::read_body(std::istream& input,
        std::streamsize size)
{
    if (size != 8)
    {
        throw BadElementLength() << err_pos(offset_) << err_id(id_) <<
            err_valid_sizes(std::vector<std::streamsize>(1, 8)) <<
            err_el_size(size);
    }
    input.read(reinterpret_cast<char*>(&value_), 8);
    if (!input)
    {
        throw ReadError() << err_pos(input.tellg());
    }
    return 8;
}

