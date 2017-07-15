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
#if !defined(TAWARA_ELEMENT_H_)
#define TAWARA_ELEMENT_H_

#include <tawara/el_ids.h>
#include <tawara/win_dll.h>

#include <ios>
#include <iostream>
#include <stdint.h>


/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The Element interface, a basic interface to an element object.
     *
     * Tawara objects store their data in elements. Like in XML, the elements
     * form a tree of information. Each element contains a single value and
     * zero or more sub-elements. Each element has a unique ID within the
     * format.
     *
     * This interface provides the most basic element facilities. It provides
     * the element's ID and an abstract interface to read and write elements to
     * a byte stream.
     */
    class TAWARA_EXPORT Element
    {
        public:
            /** \brief Create a new Element.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \exception InvalidEBMLID if the provided ID is invalid.
             */
            Element(tawara::ids::ID id);

            /// \brief Destructor.
            virtual ~Element() {};

            /** Get the element's ID.
             *
             * The element's ID is an unsigned integer with a maximum size of
             * 28 bits. Some IDs are invalid:
             * - 0
             * - 0xFF
             * - 0xFFFF
             * - 0xFFFFFF
             * - 0xFFFFFFFF
             * - 0x100000000 or greater
             *
             * IDs are divided into four classes:
             * - Class A (0 to 2^7 - 1)
             * - Class B (2^7 to 2^14 - 1)
             * - Class C (2^14 to 2^21 - 1)
             * - Class D (2^21 to 2^28 - 1)
             *
             * In some storage types, such as EBML file storage, the higher
             * classes use more storage space and so are more likely to be
             * unique within the file. This allows them to be used for
             * synchronisation in the event of file corruption. To take
             * advantage of this, you should use higher IDs for elements that
             * occur less frequently, such as the top-level elements.
             */
            uint32_t id() const { return id_; }

            /** Get the element's offset in the byte stream.
             *
             * If the element has been written, or was read from a byte stream,
             * this value will contain its position in that stream. It is
             * updated every time the element is read or written, so reading
             * from one offset and then writing to another will change the
             * stored offset.
             *
             * If the offset is std::numeric_limits<std::streampos>::max(),
             * then the element has not yet been read or written.
             */
            std::streampos offset() const { return offset_; }

            /** \brief Get the total size of the element.
             *
             * Returns the size, in bytes, required to store this entire
             * element, including its ID, data size value and body.
             *
             * \return The size of the entire element, in bytes.
             */
            virtual std::streamsize size() const;

            /** \brief Element writing.
             *
             * Writes the entire element, including its ID, body size and body
             * data, to a byte stream providing a std::ostream interface.
             *
             * \param[in] output The destination byte stream to write to.
             * \return The number of bytes written.
             * \exception WriteError if an error occurs writing data.
             */
            virtual std::streamsize write(std::ostream& output);

            /** \brief Element reading.
             *
             * Reads the element from a byte stream providing a std::istream
             * interface.
             *
             * This method assumes that the Element ID has already been read
             * (and thus used to construct the Element instance doing the
             * reading), which means that the file's read pointer should be
             * positioned at the first byte of the element's size.
             *
             * \return The number of bytes read.
             * \exception ReadError if an error occurs reading data.
             * \exception BadBodySize if the size read from the element's
             * header doesn't match its actual size. Only occurs with master
             * elements.
             * \exception InvalidChildID if a child element is found in the
             * body of a master element to which it doesn't belong.
             * \exception MissingChild if a child element that must be present
             * in a master element is not found.
             * \throw ValueOutOfRange if a child element is read with a value
             * that is out of range.
             * \throw ValueSizeOutOfRange if a child element is read with a
             * size that is not in the allowable range of sizes.
             */
            virtual std::streamsize read(std::istream& input);

        protected:
            tawara::ids::ID id_;
            std::streampos offset_;

            /** \brief Get the size of the body of this element.
             *
             * Returns the size, in bytes, required to store this element's
             * body. This does not include the space required by the ID or the
             * data size value.
             *
             * See also size().
             *
             * \return The size of the element's body, in bytes.
             */
            virtual std::streamsize body_size() const = 0;

            /** \brief Element ID writing.
             *
             * Writes the element's EBML ID to a byte stream providing a
             * std::ostream interface. Up to 4 bytes may be written.
             *
             * \param[in] output The destination byte stream to write to.
             * \return The number of bytes written.
             * \exception WriteError if an error occurs writing data.
             */
            std::streamsize write_id(std::ostream& output);

            /** \brief Element size writing.
             *
             * Writes the element's size to a byte stream providing a
             * std::ostream interface.
             *
             * \return The number of bytes written.
             * \exception WriteError if an error occurs writing data.
             */
            virtual std::streamsize write_size(std::ostream& output);

            /** \brief Element body writing.
             *
             * Writes the element's body to a byte stream providing a
             * std::ostream interface.
             *
             * \return The number of bytes written.
             * \exception WriteError if an error occurs writing data.
             */
            virtual std::streamsize write_body(std::ostream& output) = 0;

            /** \brief Element body reading implementation.
             *
             * Implementations of the Element interface should implement this
             * function to read the body of the element. When this function is
             * called, the read pointer in the byte stream will be positioned
             * at the first byte of the element's body (i.e. immediately after
             * the element's size value).
             *
             * \param[in] input The input byte stream containing the element's
             * body data.
             * \param[in] size The size of the body data. The stream \e must \e
             * not be read beyond this number of bytes.
             * \return The number of bytes read.
             * \exception ReadError if an error occurs reading data.
             * \exception BadBodySize if the size read from the element's
             * header doesn't match its actual size. Only occurs with master
             * elements.
             * \exception InvalidChildID if a child element is found in the
             * body of a master element to which it doesn't belong.
             * \exception MissingChild if a child element that must be present
             * in a master element is not found.
             * \throw ValueOutOfRange if a child element is read with a value
             * that is out of range.
             * \throw ValueSizeOutOfRange if a child element is read with a
             * size that is not in the allowable range of sizes.
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size) = 0;

    }; // class Element


    /** \brief Skip an element in an input stream.
     *
     * This function skips past an element, placing the read pointer at the ID
     * of the next element.
     *
     * \param[in] input The input stream to seek the read pointer in.
     * \param[in] and_id If true, the read pointer is expected to be at the ID
     * of the element to skip when the function is called, and the ID will also
     * be skipped. If false, the read pointer is expected to be placed before
     * the element's size value.
     * \return The number of bytes skipped.
     */
    std::streamsize skip_read(std::istream& input, bool and_id);


    /** \brief Skip an element in an input/output stream.
     *
     * This function skips past an element, placing the write pointer after the
     * end of the element's body.
     *
     * \param[in] stream The input/ouput stream to seek the write pointer in.
     * \param[in] and_id If true, the write pointer is expected to be at the ID
     * of the element to skip when the function is called, and the ID will also
     * be skipped. If false, the write pointer is expected to be placed before
     * the element's size value.
     * \return The number of bytes skipped.
     */
    std::streamsize skip_write(std::iostream& stream, bool and_id);
}; // namespace tawara

/// @}
/// group interfaces

#endif // TAWARA_ELEMENT_H_

