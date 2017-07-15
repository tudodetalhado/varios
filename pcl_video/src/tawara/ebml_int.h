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

#if !defined(TAWARA_EBML_INT_H_)
#define TAWARA_EBML_INT_H_

#include <cstddef>
#include <istream>
#include <ostream>
#include <stdint.h>
#include <vector>

namespace tawara
{
    /** \brief Functions for managing integers coded for EBML.
     *
     * This namespace contains the functions used to mange the way integers,
     * both signed and unsigned, are stored in EBML files. Rather than writing
     * a constant number of bytes regardless of the value being stored, EBML
     * specifies that leading 0x00 bytes and, for negative signed integers,
     * leading 0xFF bytes, be trimmed. For example, a value of 2 will be stored
     * as 0x02, even if the variable that holds it is 32-bit (i.e. 0x00000002).
     * Similarly, a value of -30000, or 0xFFFFF530 in 32 bits, will be stored
     * as 0xF530.
     *
     * Note that this is distinct from the coding used on EBML Element IDs and
     * data sizes, which relies on leading zero bits to indicate the stored
     * size.
     */
    namespace ebml_int
    {
        /** \brief Get the size of an unsigned integer after encoding.
         *
         * The size required by an encoded integer depends on the value of that
         * integer, and will range from 1 to 8 bytes.
         *
         * \param[in] integer The integer to get the size of.
         * \return The size, in bytes, that the integer will require when
         * coded.
         */
        std::streamsize size_u(uint64_t integer);

        /** \brief Get the size of a signed integer after encoding.
         *
         * The size required by an encoded integer depends on the value of that
         * integer, and will range from 1 to 8 bytes.
         *
         * \param[in] integer The integer to get the size of.
         * \return The size, in bytes, that the integer will require when
         * coded.
         */
        std::streamsize size_s(int64_t integer);

        /** \brief Encode an unsigned integer into a buffer.
         *
         * Encodes an unsigned integer according to the EBML specification for
         * unsigned integers. Leading zero bytes are trimmed.
         *
         * \param[in] integer The integer to encode.
         * \return A vector containing the encoded data.
         */
        std::vector<char> encode_u(uint64_t integer);

        /** \brief Encode a signed integer into a buffer.
         *
         * Encodes an unsigned integer according to the EBML specification for
         * signed integers. Leading zero or 0xFF bytes are trimmed.
         *
         * \param[in] integer The integer to encode.
         * \return A vector containing the encoded data.
         */
        std::vector<char> encode_s(int64_t integer);

        /** \brief Encode and write an unsigned integer into a byte stream.
         *
         * This function performs the same task as tawara::ebml_int::encode_u(),
         * but instead of writing to a basic buffer, it writes to a
         * std::ostream object.
         *
         * \param[in] integer The integer to encode.
         * \param[in] output The std::ostream object to write to.
         * \return The number of bytes written.
         * \exception WriteError if there is an error writing the output stream.
         */
        std::streamsize write_u(uint64_t integer, std::ostream& output);

        /** \brief Encode and write a signed integer into a byte stream.
         *
         * This function performs the same task as tawara::ebml_int::encode_s(),
         * but instead of writing to a basic buffer, it writes to a
         * std::ostream object.
         *
         * \param[in] integer The integer to encode.
         * \param[in] output The std::ostream object to write to.
         * \return The number of bytes written.
         * \exception WriteError if there is an error writing the output stream.
         */
        std::streamsize write_s(int64_t integer, std::ostream& output);

        /** \brief Decode an unsigned integer from a buffer.
         *
         * Decodes the unsigned integer stored in the buffer according to the
         * EBML specification for unsigned integers.
         *
         * \param[in] buffer The buffer holding the raw data. The size of the
         * buffer defines the number of bytes to use for the integer; it must
         * be 8 or less.
         * \return The decoded unsigned integer.
         */
        uint64_t decode_u(std::vector<char> const& buffer);

        /** \brief Decode a signed integer from a buffer.
         *
         * Decodes the unsigned integer stored in the buffer according to the
         * EBML specification for unsigned integers.
         *
         * \param[in] buffer The buffer holding the raw data. The size of the
         * buffer defines the number of bytes to use for the integer; it must
         * be 8 or less.
         * \return The decoded unsigned integer.
         */
        int64_t decode_s(std::vector<char> const& buffer);

        /** \brief Read and decode an unsigned integer from a byte stream.
         *
         * This function performs the same task as tawara::ebml_int::decode_u(),
         * but instead of reading from a basic buffer, it reads from a
         * std::istream object.
         *
         * \param[in] input The std::istream object to read from.
         * \param[in] n The number of bytes from the buffer to read.
         * \return The decoded unsigned integer.
         * \exception ReadError if there is an error reading the input stream.
         */
        uint64_t read_u(std::istream& input, std::streamsize n);

        /** \brief Read and decode a signed integer from a byte stream.
         *
         * This function performs the same task as tawara::ebml_int::decode_s(),
         * but instead of reading from a basic buffer, it reads from a
         * std::istream object.
         *
         * \param[in] input The std::istream object to read from.
         * \param[in] n The number of bytes from the buffer to read.
         * \return The decoded signed integer.
         * \exception ReadError if there is an error reading the input stream.
         */
        int64_t read_s(std::istream& input, std::streamsize n);
    }; // namespace ebml_int
}; // namespace tawara

#endif // TAWARA_EBML_INT_H_


