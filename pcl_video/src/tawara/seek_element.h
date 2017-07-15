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

#if !defined(TAWARA_SEEK_ELEMENT_H_)
#define TAWARA_SEEK_ELEMENT_H_

#include <ios>
#include <tawara/binary_element.h>
#include <tawara/el_ids.h>
#include <tawara/master_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The Seek element, which contains a single index point in the
     * SeekHead element.
     *
     * The SeekHead (Metaseek) element is used as an index into the file's data
     * stream, giving the byte offsets of other level 1 elements. A Seek
     * element is a single index entry in the Metaseek. It stores the element
     * ID and the byte offset of the element.
     */
    class TAWARA_EXPORT SeekElement : public MasterElement
    {
        public:
            /** Create a new Seek element.
             *
             * \param[in] id The element ID to index.
             * \param[in] offset The position in the bytestream from the start
             * of the segment to the element's ID. 0 for the first element in
             * the segment.
             */
            SeekElement(ids::ID id, std::streampos offset);

            /// \brief Destructor.
            virtual ~SeekElement() {}

            /// \brief Get the ID that is indexed by this Seek element.
            ids::ID indexed_id() const;
            /// \brief Set the ID that is indexed.
            void indexed_id(ids::ID id);

            /// \brief Get the offset of the indexed ID.
            std::streamsize offset() const { return offset_.value(); }
            /// \brief Set the offset of the indexed ID.
            void offset(std::streamsize offset) { offset_.value(offset); }

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

        protected:
            BinaryElement indexed_id_;
            UIntElement offset_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class SeekElement
}; // namespace tawara

/// @}

#endif // TAWARA_SEEK_ELEMENT_H_

