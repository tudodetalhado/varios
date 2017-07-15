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

#if !defined(TAWARA_STRING_ELEMENT_H_)
#define TAWARA_STRING_ELEMENT_H_

#include <tawara/prim_element.h>
#include <tawara/win_dll.h>

#include <string>

/// \addtogroup implementations Implementations
/// @{

namespace tawara
{
    /** String primitive element.
     *
     * This element stores a UTF-8 string. Upon writing to a store, the string
     * may or may not be padded with null bytes.
     */
    class TAWARA_EXPORT StringElement : public PrimitiveElement<std::string>
    {
        public:
            /** \brief Create a new string element with no default.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             */
            StringElement(uint32_t id, std::string value);

            /** \brief Create a new string element with a default value.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             * \param[in] default_value The default value of the element.
             */
            StringElement(uint32_t id, std::string value,
                    std::string default_value);

            /// \brief Value assignment operator.
            virtual StringElement& operator=(std::string const& rhs);

            /// \brief Get the amount of padding used.
            virtual uint64_t padding() const { return padding_; }
            /** \brief Set the amount of padding to use.
             *
             * Strings can be zero-padded at the end. This is particularly
             * useful when overwriting an existing string with one that is
             * shorter, so that the file does not need to be rewritten or a
             * void element used.
             *
             * Management of this value is the responsibility of the user of
             * the StringElement. It will never be adjusted automatically.
             */
            virtual void padding(uint64_t padding) { padding_ = padding; }

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

        protected:
            uint64_t padding_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class StringElement
}; // namespace tawara

/// @}
// group implementations

#endif // TAWARA_STRING_ELEMENT_H_

