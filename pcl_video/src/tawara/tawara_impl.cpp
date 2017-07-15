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

#include <tawara/tawara_impl.h>

#include <tawara/ebml_element.h>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

TawaraImpl::TawaraImpl(std::iostream& stream)
    : Tawara(stream),
    stream_(stream)
{
    prepare_stream();
}


///////////////////////////////////////////////////////////////////////////////
// Private methods
///////////////////////////////////////////////////////////////////////////////

void TawaraImpl::prepare_stream()
{
    // Preserve the current read position
    std::streamsize cur(stream_.tellg());
    // Check the file size
    stream_.seekg(0, std::ios::end);
    std::streamsize size(stream_.tellg());
    stream_.seekg(cur, std::ios::beg);
    // If the file is empty, write an EBML header
    if (size <= 0)
    {
        stream_.seekg(0, std::ios::beg);
        EBMLElement e;
        e.write(stream_);
    }
    // If the file is not empty, search for an EBML header
    else
    {
        bool found_header(false);
        while (!stream_.eof())
        {
            char c(stream_.get());
            if (c == 0x1A)
            {
                stream_.seekg(-1, std::ios::cur);
                // Read the Header ID
                ids::ReadResult hdr = ids::read(stream_);
                if (hdr.first != ids::EBML)
                {
                    // Not an EBML header; throw
                    throw NotEBML();
                }
                found_header = true;
                break;
            }
        }
        if (!found_header)
        {
            // Header not found: throw
            throw NotEBML();
        }
        // Read the header and check the DocType
        EBMLElement e;
        e.read(stream_);
        if (e.doc_type() != TawaraDocType)
        {
            throw NotTawara();
        }
        if (e.read_version() > TawaraVersionMajor)
        {
            throw BadReadVersion();
        }
        if (e.doc_read_version() > TawaraVersionMajor)
        {
            throw BadDocReadVersion();
        }
    }
}

