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

#if !defined(TAWARA_DATE_ELEMENT_H_)
#define TAWARA_DATE_ELEMENT_H_

#include <tawara/prim_element.h>
#include <tawara/win_dll.h>

#include <stdint.h>

/// \addtogroup implementations Implementations
/// @{

namespace tawara
{
    /** Date primitive element.
     *
     * This element stores a date. A date is represented as a signed, 64-bit
     * integer giving the number of nanoseconds since 2001-01-01 00:00:00.
     */
    class TAWARA_EXPORT DateElement : public PrimitiveElement<int64_t>
    {
        public:
            /** \brief Create a new date element with no default.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             */
            DateElement(uint32_t id, int64_t value);

            /** \brief Create a new date element with a default value.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             * \param[in] default_value The default value of the element.
             */
            DateElement(uint32_t id, int64_t value, int64_t default_value);

            /// \brief Value assignment operator.
            virtual DateElement& operator=(int64_t const& rhs);

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

        protected:
            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /** \brief Element body loading.
             *
             * \exception BadElementLength if the date element is an incorrect
             * length (i.e. not 8 bytes).
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class DateElement
}; // namespace tawara

/// @}
// group implementations

#endif // TAWARA_DATE_ELEMENT_H_

