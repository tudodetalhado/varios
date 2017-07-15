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

#include <tawara/ebml_element.h>

#include <tawara/exceptions.h>
#include <tawara/tawara_config.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

EBMLElement::EBMLElement(std::string const& doc_type)
    : MasterElement(ids::EBML),
    ver_(ids::EBMLVersion, TawaraEBMLVersion, TawaraEBMLVersion),
    read_ver_(ids::EBMLReadVersion, TawaraEBMLVersion, TawaraEBMLVersion),
    max_id_length_(ids::EBMLMaxIDLength, 4, 4),
    max_size_length_(ids::EBMLMaxSizeLength, 8, 8),
    doc_type_(ids::DocType, doc_type, TawaraDocType),
    doc_type_ver_(ids::DocTypeVersion, TawaraVersionMajor, TawaraVersionMajor),
    doc_type_read_ver_(ids::DocTypeReadVersion, TawaraVersionMajor,
            TawaraVersionMajor)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize EBMLElement::body_size() const
{
    return ver_.size() +
        read_ver_.size() +
        max_id_length_.size() +
        max_size_length_.size() +
        doc_type_.size() +
        doc_type_ver_.size() +
        doc_type_read_ver_.size();
}


std::streamsize EBMLElement::write_body(std::ostream& output)
{
    std::streamsize written(0);
    // The EBML header element always writes every value, regardless of if it
    // is the default or not. If it did not, other implementations may use
    // different defaults and things would go very wrong, very quickly.
    written += ver_.write(output);
    written += read_ver_.write(output);
    written += max_id_length_.write(output);
    written += max_size_length_.write(output);
    written += doc_type_.write(output);
    written += doc_type_ver_.write(output);
    written += doc_type_read_ver_.write(output);
    return written;
}


std::streamsize EBMLElement::read_body(std::istream& input,
        std::streamsize size)
{
    // Start by resetting everything to the defaults
    set_defaults_();
    std::streamsize read_bytes(0);
    // Read IDs until the body is exhausted
    while (read_bytes < size)
    {
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::EBMLVersion:
                read_bytes += ver_.read(input);
                break;
            case ids::EBMLReadVersion:
                read_bytes += read_ver_.read(input);
                break;
            case ids::EBMLMaxIDLength:
                read_bytes += max_id_length_.read(input);
                break;
            case ids::EBMLMaxSizeLength:
                read_bytes += max_size_length_.read(input);
                break;
            case ids::DocType:
                read_bytes += doc_type_.read(input);
                break;
            case ids::DocTypeVersion:
                read_bytes += doc_type_ver_.read(input);
                break;
            case ids::DocTypeReadVersion:
                read_bytes += doc_type_read_ver_.read(input);
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

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Internal methods
///////////////////////////////////////////////////////////////////////////////

void EBMLElement::set_defaults_()
{
    ver_.value(TawaraEBMLVersion);
    read_ver_.value(TawaraEBMLVersion);
    max_id_length_.value(4);
    max_size_length_.value(8);
    doc_type_.value(TawaraDocType);
    doc_type_ver_.value(TawaraVersionMajor);
    doc_type_read_ver_.value(TawaraVersionMajor);
}

