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

#include <tawara/void_element.h>

#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

VoidElement::VoidElement(std::streamsize tgt_size, bool fill)
    : Element(ids::Void), fill_(fill), extra_size_(0)
{
    if (tgt_size < 2)
    {
        // Void elements must be at least 2 bytes
        throw VoidTooSmall();
    }
    // Set this element's size from the total size required.  We need to
    // calculate an appropriate body size that, when combined with the data
    // size and the ID size, will give the same size as size_. Start by
    // estimating the bytes required for the body size.
    size_ = tgt_size - 1;
    size_ -= tawara::vint::size(size_);
    // Check if enough space is used
    if (size() != tgt_size)
    {
        // Need to use more space (typically 1 more byte), but if we increase
        // the body size, it might push the data size value over the line to
        // requiring another byte, meaning that the void element's total size
        // would go from 1 byte under to 1 byte over. Instead, we require that
        // the body size is stored with an extra byte.
        extra_size_ = 1;
    }
    assert(size() == tgt_size);
}


VoidElement::VoidElement(Element const& element, bool fill)
    : Element(ids::Void), fill_(fill), extra_size_(0)
{
    // Set this element's size from the total size of the element to replace.
    // We need to calculate an appropriate body size that, when combined with
    // the data size and the ID size, will give the same size as
    // element.size(). Start by estimating the bytes required for the body
    // size.
    size_ = element.size() - 1;
    size_ -= tawara::vint::size(size_);
    // Check if enough space is used
    if (size() != element.size())
    {
        // Need to use more space (typically 1 more byte), but if we increase
        // the body size, it might push the data size value over the line to
        // requiring another byte, meaning that the void element's total size
        // would go from 1 byte under to 1 byte over. Instead, we require that
        // the body size is stored with an extra byte.
        extra_size_ = 1;
    }
    assert(size() == element.size());
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void VoidElement::set_size(std::streamsize tgt_size)
{
    if (tgt_size < 2)
    {
        // Void elements must be at least 2 bytes
        throw VoidTooSmall();
    }
    // Set this element's size from the total size required.  We need to
    // calculate an appropriate body size that, when combined with the data
    // size and the ID size, will give the same size as size_. Start by
    // estimating the bytes required for the body size.
    size_ = tgt_size - 1;
    size_ -= tawara::vint::size(size_);
    // Check if enough space is used
    if (size() != tgt_size)
    {
        // Need to use more space (typically 1 more byte), but if we increase
        // the body size, it might push the data size value over the line to
        // requiring another byte, meaning that the void element's total size
        // would go from 1 byte under to 1 byte over. Instead, we require that
        // the body size is stored with an extra byte.
        extra_size_ = 1;
    }
    assert(size() == tgt_size);
}


std::streamsize VoidElement::size() const
{
    // ID is always 1 byte
    return 1 + tawara::vint::size(size_) + size_ + extra_size_;
}


///////////////////////////////////////////////////////////////////////////////
// I/O
///////////////////////////////////////////////////////////////////////////////

std::streamsize VoidElement::write(std::ostream& output)
{
    // Fill in the offset of this element in the byte stream.
    offset_ = output.tellp();

    return write_id(output) + write_body(output);
}


std::streamsize VoidElement::write_body(std::ostream& output)
{
    std::streamsize result(0);

    // Write the body size value padded with extra bytes if necessary
    result += tawara::vint::write(size_, output,
            tawara::vint::size(size_) + extra_size_);
    if (fill_)
    {
        std::vector<char> zeros(size_, 0);
        output.write(&zeros[0], zeros.size());
        if (!output)
        {
            throw WriteError() << err_pos(output.tellp());
        }
    }
    else
    {
        // Skip ahead in the file to the end of the body
        output.seekp(size_, std::ios::cur);
    }
    return result + size_;
}


std::streamsize VoidElement::read(std::istream& input)
{
    // Fill in the offset of this element in the byte stream.
    // The input stream will be at the start of the size value, so:
    //
    //  offset = current position - size of ID
    //
    // The cast here makes Apple's LLVM compiler happy
    offset_ = static_cast<std::streamsize>(input.tellg()) -
        ids::size(id_);
    // Get the element's body size
    vint::ReadResult result = tawara::vint::read(input);
    size_ = result.first;
    std::streamsize read_bytes(result.second);
    // Record the extra body size byte count for future writing
    extra_size_ = result.second - tawara::vint::size(size_);
    return read_bytes + read_body(input, size_);
}

std::streamsize VoidElement::read_body(std::istream& input,
        std::streamsize size)
{
    // Skip the body
    input.seekg(size_, std::ios::cur);
    if (!input)
    {
        throw ReadError() << err_pos(input.tellg());
    }
    return size_;
}

