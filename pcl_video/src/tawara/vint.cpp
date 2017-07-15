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

#include <tawara/vint.h>
#include <tawara/exceptions.h>


std::streamsize tawara::vint::size(uint64_t integer)
{
    if (integer < 0x80)
    {
        return 1;
    }
    else if (integer < 0x4000)
    {
        return 2;
    }
    else if (integer < 0x200000)
    {
        return 3;
    }
    else if (integer < 0x10000000)
    {
        return 4;
    }
    else if (integer < 0x800000000)
    {
        return 5;
    }
    else if (integer < 0x40000000000)
    {
        return 6;
    }
    else if (integer < 0x2000000000000)
    {
        return 7;
    }
    else if (integer < 0x100000000000000)
    {
        return 8;
    }
    else
    {
        throw tawara::VarIntTooBig() << tawara::err_varint(integer);
    }
}


tawara::vint::OffsetInt tawara::vint::s_to_u(int64_t integer)
{
    if (integer >= -0x3F && integer <= 0x3F)
    {
        return std::make_pair(integer + 0x3F, 1);
    }
    else if (integer >= -0x1FFF && integer <= 0x1FFF)
    {
        return std::make_pair(integer + 0x1FFF, 2);
    }
    else if (integer >= -0x0FFFFF && integer <= 0x0FFFFF)
    {
        return std::make_pair(integer + 0x0FFFFF, 3);
    }
    else if (integer >= -0x07FFFFFF && integer <= 0x07FFFFFF)
    {
        return std::make_pair(integer + 0x07FFFFFF, 4);
    }
    else if (integer >= -0x03FFFFFFFF && integer <= 0x03FFFFFFFF)
    {
        return std::make_pair(integer + 0x03FFFFFFFF, 5);
    }
    else if (integer >= -0x01FFFFFFFFFF && integer <= 0x01FFFFFFFFFF)
    {
        return std::make_pair(integer + 0x01FFFFFFFFFF, 6);
    }
    else if (integer >= -0xFFFFFFFFFFFF && integer <= 0xFFFFFFFFFFFF)
    {
        return std::make_pair(integer + 0xFFFFFFFFFFFF, 7);
    }
    else
    {
        throw tawara::VarIntTooBig() << tawara::err_varint(integer);
    }
}


int64_t tawara::vint::u_to_s(tawara::vint::OffsetInt integer)
{
    switch (integer.second)
    {
        case 1:
            return integer.first - 0x3F;
        case 2:
            return integer.first - 0x1FFF;
        case 3:
            return integer.first - 0x0FFFFF;
        case 4:
            return integer.first - 0x07FFFFFF;
        case 5:
            return integer.first - 0x03FFFFFFFF;
        case 6:
            return integer.first - 0x01FFFFFFFFFF;
        case 7:
            return integer.first - 0xFFFFFFFFFFFF;
        default:
            throw tawara::VarIntTooBig() << tawara::err_varint(integer.first);
    }
}


std::vector<char> tawara::vint::encode(uint64_t integer, std::streamsize req_size)
{
    assert(req_size <= 8);

    unsigned int shifts(0);
    uint8_t mask(0);
    std::vector<char> buffer;

    std::streamsize c_size(size(integer));
    if (req_size > 0)
    {
        if (req_size < c_size)
        {
            throw tawara::SpecSizeTooSmall() << tawara::err_varint(integer) <<
                tawara::err_reqsize(req_size);
        }
        c_size = req_size;
    }
    switch (c_size)
    {
        case 1:
            // Skip the byte-copying code
            buffer.push_back(integer | 0x80);
            return buffer;
        case 2:
            shifts = 1;
            mask = 0x40;
            break;
        case 3:
            shifts = 2;
            mask = 0x20;
            break;
        case 4:
            shifts = 3;
            mask = 0x10;
            break;
        case 5:
            shifts = 4;
            mask = 0x08;
            break;
        case 6:
            shifts = 5;
            mask = 0x04;
            break;
        case 7:
            shifts = 6;
            mask = 0x02;
            break;
        case 8:
            shifts = 7;
            mask = 0x01;
            break;
    }

    buffer.assign(c_size, 0);
    for(int ii(shifts); ii > 0; --ii)
    {
        buffer[ii] = (integer >> (shifts - ii) * 8) & 0xFF;
    }
    buffer[0] = ((integer >> shifts * 8) & 0xFF) | mask;

    return buffer;
}


tawara::vint::DecodeResult tawara::vint::decode(std::vector<char> const& buffer)
{
    assert(buffer.size() > 0);

    uint64_t result(0);
    unsigned int to_copy(0);
    if (static_cast<unsigned char>(buffer[0]) >= 0x80) // 1 byte
    {
        // There will be no extra bytes to copy.
        return std::make_pair(buffer[0] & 0x7F, buffer.begin() + 1);
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x40) // 2 bytes
    {
        result = buffer[0] & 0x3F;
        to_copy = 1;
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x20) // 3 bytes
    {
        result = buffer[0] & 0x1F;
        to_copy = 2;
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x10) // 4 bytes
    {
        result = buffer[0] & 0x0F;
        to_copy = 3;
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x08) // 5 bytes
    {
        result = buffer[0] & 0x07;
        to_copy = 4;
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x04) // 6 bytes
    {
        result = buffer[0] & 0x03;
        to_copy = 5;
    }
    else if (static_cast<unsigned char>(buffer[0]) >= 0x02) // 7 bytes
    {
        result = buffer[0] & 0x01;
        to_copy = 6;
    }
    else if (static_cast<unsigned char>(buffer[0]) == 0x01) // 8 bytes
    {
        // The first byte is not part of the result in this case
        to_copy = 7;
    }
    else
    {
        // All bits zero is invalid
        throw tawara::InvalidVarInt();
    }

    if (buffer.size() < to_copy + 1)
    {
        throw tawara::BufferTooSmall() << tawara::err_bufsize(buffer.size()) <<
            tawara::err_reqsize(to_copy);
    }

    // Copy the remaining bytes
    for (std::streamsize ii(1); ii < to_copy + 1; ++ii)
    {
        result <<= 8;
        result += static_cast<unsigned char>(buffer[ii]);
    }
    return std::make_pair(result, buffer.begin() + to_copy + 1);
}


std::streamsize tawara::vint::write(uint64_t integer, std::ostream& output,
        std::streamsize req_size)
{
    assert(req_size <= 8);

    unsigned int shifts(0);
    uint8_t mask(0);

    std::streamsize c_size(size(integer));
    if (req_size > 0)
    {
        if (req_size < c_size)
        {
        throw tawara::SpecSizeTooSmall() << tawara::err_varint(integer) <<
            tawara::err_reqsize(req_size);
        }
        c_size = req_size;
    }
    switch (c_size)
    {
        case 1:
            // Skip the byte-copying code
            output.put(integer | 0x80);
            if (!output)
            {
                throw tawara::WriteError() << tawara::err_pos(output.tellp());
            }
            return 1;
            break;
        case 2:
            shifts = 1;
            mask = 0x40;
            break;
        case 3:
            shifts = 2;
            mask = 0x20;
            break;
        case 4:
            shifts = 3;
            mask = 0x10;
            break;
        case 5:
            shifts = 4;
            mask = 0x08;
            break;
        case 6:
            shifts = 5;
            mask = 0x04;
            break;
        case 7:
            shifts = 6;
            mask = 0x02;
            break;
        case 8:
            shifts = 7;
            mask = 0x01;
            break;
    }

    // Write the first byte with its length indicator
    output.put(((integer >> shifts * 8) & 0xFF) | mask);
    // Write the remaining bytes
    for (unsigned int ii(1); ii <= shifts; ++ii)
    {
        output.put((integer >> (shifts - ii) * 8) & 0xFF);
    }
    if (!output)
    {
        throw tawara::WriteError() << tawara::err_pos(output.tellp());
    }

    return c_size;
}


tawara::vint::ReadResult tawara::vint::read(std::istream& input)
{
    uint64_t result(0);
    std::streamsize to_copy(0);
    uint8_t buffer[8];

    // Read the first byte
    input.read(reinterpret_cast<char*>(buffer), 1);
    if (input.fail())
    {
        throw tawara::ReadError() << tawara::err_pos(input.tellg());
    }
    // Check the size
    if (buffer[0] >= 0x80) // 1 byte
    {
        return std::make_pair(buffer[0] & 0x7F, 1);
    }
    else if (buffer[0] >= 0x40) // 2 bytes
    {
        result = buffer[0] & 0x3F;
        to_copy = 1;
    }
    else if (buffer[0] >= 0x20) // 3 bytes
    {
        result = buffer[0] & 0x1F;
        to_copy = 2;
    }
    else if (buffer[0] >= 0x10) // 4 bytes
    {
        result = buffer[0] & 0x0F;
        to_copy = 3;
    }
    else if (buffer[0] >= 0x08) // 5 bytes
    {
        result = buffer[0] & 0x07;
        to_copy = 4;
    }
    else if (buffer[0] >= 0x04) // 6 bytes
    {
        result = buffer[0] & 0x03;
        to_copy = 5;
    }
    else if (buffer[0] >= 0x02) // 7 bytes
    {
        result = buffer[0] & 0x01;
        to_copy = 6;
    }
    else if (buffer[0] == 0x01) // 8 bytes
    {
        // The first byte is not part of the result in this case
        to_copy = 7;
    }
    else
    {
        // All bits zero is invalid
        throw tawara::InvalidVarInt();
    }

    // Copy the remaining bytes
    input.read(reinterpret_cast<char*>(&buffer[1]), to_copy);
    if (input.fail())
    {
        throw tawara::ReadError() << tawara::err_pos(input.tellg());
    }

    for (std::streamsize ii(1); ii < to_copy + 1; ++ii)
    {
        result <<= 8;
        result += buffer[ii];
    }
    return std::make_pair(result, to_copy + 1);
}

