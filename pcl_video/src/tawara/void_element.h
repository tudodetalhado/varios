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

#if !defined(TAWARA_INT_ELEMENT_H_)
#define TAWARA_INT_ELEMENT_H_

#include <tawara/el_ids.h>
#include <tawara/prim_element.h>
#include <tawara/win_dll.h>

#include <stdint.h>

/// \addtogroup implementations Implementations
/// @{

namespace tawara
{
    /** Void primitive element.
     *
     * This is a void element, specified with the EBML ID 0xEC. Void elements
     * are ignored by the parser. They are typically used to reserve some space
     * in a byte stream for later writing, or to blank out an element that is
     * no longer used without re-writing the entire file.
     */
    class TAWARA_EXPORT VoidElement : public Element
    {
        public:
            /** \brief Create a new void element.
             *
             * \param[in] tgt_size The size of the element, in bytes. This much
             * space will be reserved in the file.
             * \param[in] fill If true, when writing the element, the element's
             * body will be filled with 0x00.
             * \throw VoidTooSmall if tgt_size is less than 2 bytes.
             */
            VoidElement(std::streamsize tgt_size, bool fill=false);

            /** \brief Create a new VoidElement that replaces another element.
             *
             * This constructor creates a VoidElement with its fill set to the
             * necessary size to completely and exactly cover the element
             * passed to it.
             *
             * \param[in] element The element to replace.
             * \param[in] fill Whether to fill the space in the file with
             * zeros.
             */
            VoidElement(Element const& element, bool fill=false);

            /** \brief Set the size of this element.
             *
             * A void element has a size value, given in bytes, which
             * determines how much space it reserves in the byte stream.
             *
             * \throw VoidTooSmall if tgt_size is less than 2 bytes.
             */
            void set_size(std::streamsize tgt_size);

            /// \brief Get the total size of the element.
            std::streamsize size() const;

            /// \brief Get the fill setting.
            bool fill() const { return fill_; }
            /** \brief Set the fill setting.
             *
             * A void element may fill its body when written. This is typically
             * used when writing it at the end of a byte stream, such as
             * reserving space in a file to be filled in later with other data.
             * Set fill to true to do so.
             *
             * On the other hand, void elements are also used for blanking out
             * existing elements that are no longer used, such as when removing
             * a tag, without needing to rewrite the entire file. In this case,
             * only the element ID and size need to be written, with the
             * remainder of the element's body being left as-is. Set fill to
             * false for this style of writing.
             */
            void fill(bool fill) { fill_ = fill; }

            /// \brief Element writing.
            virtual std::streamsize write(std::ostream& output);

            /** \brief Element body writing.
             *
             * Writes the element's size and body to a byte stream providing a
             * std::ostream interface.
             *
             * Void elements may or may not fill their body with 0x00, based on
             * the setting of the fill member property. Whether or not the body
             * is actually filled by this method, the return value and the
             * write position pointer in the output stream will reflect the
             * full size of the void element.
             *
             * \return The number of bytes written.
             */
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief Element reading.
            std::streamsize read(std::istream& input);

        private:
            /// The size of space to reserve in the byte stream.
            std::streamsize size_;
            /// If the element's body should be filled with zeroes or not.
            bool fill_;
            /// Extra bytes for writing the body size value if necessary.
            std::streamsize extra_size_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const { return size_; }

            /** \brief Element body loading.
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
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class VoidElement
}; // namespace tawara

/// @}
// group implementations

#endif // TAWARA_INT_ELEMENT_H_

