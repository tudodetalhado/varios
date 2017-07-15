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

#if !defined(TAWARA_FLOAT_ELEMENT_H_)
#define TAWARA_FLOAT_ELEMENT_H_

#include <tawara/prim_element.h>
#include <tawara/win_dll.h>

/// \addtogroup implementations Implementations
/// @{

namespace tawara
{
    /// \brief Precision of float elements.
    enum EBMLFloatPrec
    {
        /// Single precision
        EBML_FLOAT_PREC_SINGLE,
        /// Double precision
        EBML_FLOAT_PREC_DOUBLE
    };

    /** Float primitive element.
     *
     * This element stores an IEEE floating-point number. 4-byte and 8-byte
     * floats are allowed.
     */
    class TAWARA_EXPORT FloatElement : public PrimitiveElement<double>
    {
        public:
            /** \brief Create a new float element with no default.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             * \param[in] precision The element's precision, single or double.
             * This only has an effect when writing the float to file.
             */
            FloatElement(uint32_t id, double value,
                    EBMLFloatPrec precision=EBML_FLOAT_PREC_DOUBLE);

            /** \brief Create a new float element with a default value.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             * \param[in] precision The element's precision, single or double.
             * This only has an effect when writing the float to a file.
             * \param[in] default_value The default value of the element.
             */
            FloatElement(uint32_t id, double value, double default_value,
                    EBMLFloatPrec precision=EBML_FLOAT_PREC_DOUBLE);

            /// \brief Value assignment operator.
            virtual FloatElement& operator=(double const& rhs);

            /// \brief Get the precision setting.
            virtual EBMLFloatPrec precision() const { return prec_; }
            /** \brief Set the precision setting
             *
             * This value determines if the float is single or double
             * precision. The precision value has no effect until the float is
             * written to a file, at which point single-precision floats are
             * written using 4 bytes while double-precision floats are written
             * using 8 bytes.
             */
            virtual void precision(EBMLFloatPrec precision)
            { prec_ = precision; }

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

        protected:
            EBMLFloatPrec prec_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /** \brief Element body loading.
             *
             * \exception BadElementLength if the float element is an incorrect
             * length (i.e. not 4 or 8 bytes).
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class FloatElement
}; // namespace tawara

/// @}
// group implementations

#endif // TAWARA_FLOAT_ELEMENT_H_

