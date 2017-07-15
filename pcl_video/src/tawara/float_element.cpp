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

#include <tawara/float_element.h>

#include <tawara/exceptions.h>
#include <tawara/vint.h>
#include <vector>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

FloatElement::FloatElement(uint32_t id, double value, EBMLFloatPrec precision)
    : PrimitiveElement<double>(id, value), prec_(precision)
{
}


FloatElement::FloatElement(uint32_t id, double value, double default_value,
        EBMLFloatPrec precision)
    : PrimitiveElement<double>(id, value, default_value),
    prec_(precision)
{
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

FloatElement& FloatElement::operator=(double const& rhs)
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

std::streamsize FloatElement::body_size() const
{
    switch(prec_)
    {
        case EBML_FLOAT_PREC_SINGLE:
            return 4;
        case EBML_FLOAT_PREC_DOUBLE:
            return 8;
    };
}


std::streamsize FloatElement::write_body(std::ostream& output)
{
    float tmp(0);
    std::streamsize result(0);
    switch(prec_)
    {
        case EBML_FLOAT_PREC_SINGLE:
            tmp = value_;
            output.write(reinterpret_cast<char*>(&tmp), 4);
            if (!output)
            {
                throw WriteError() << err_pos(output.tellp());
            }
            result = 4;
            break;
        case EBML_FLOAT_PREC_DOUBLE:
            output.write(reinterpret_cast<char*>(&value_), 8);
            if (!output)
            {
                throw WriteError() << err_pos(output.tellp());
            }
            result = 8;
            break;
    };
    return result;
}


std::streamsize FloatElement::read_body(std::istream& input,
        std::streamsize size)
{
    if (size == 4)
    {
        float tmp(0);
        input.read(reinterpret_cast<char*>(&tmp), 4);
        if (!input)
        {
            throw ReadError() << err_pos(input.tellg());
        }
        value_ = tmp;
        prec_ = EBML_FLOAT_PREC_SINGLE;
        return 4;
    }
    else if (size == 8)
    {
        double tmp(0);
        input.read(reinterpret_cast<char*>(&tmp), 8);
        if (!input)
        {
            throw ReadError() << err_pos(input.tellg());
        }
        value_ = tmp;
        prec_ = EBML_FLOAT_PREC_DOUBLE;
        return 8;
    }
    else
    {
        std::vector<std::streamsize> valid_sizes;
        valid_sizes.push_back(4); valid_sizes.push_back(8);
        throw BadElementLength() << err_pos(offset_) << err_id(id_) <<
            err_valid_sizes(valid_sizes) << err_el_size(size);
    }
}

