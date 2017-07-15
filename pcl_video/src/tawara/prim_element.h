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

#if !defined(TAWARA_PRIM_ELEMENT_H_)
#define TAWARA_PRIM_ELEMENT_H_


#include <boost/operators.hpp>
#include <stdint.h>
#include <string>
#include <tawara/element.h>
#include <tawara/exceptions.h>
#include <tawara/win_dll.h>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The primitive data element interface.
     *
     * Primitive elements store one of the EBML primitive data types. These
     * are:
     *
     * - Signed integers, up to 8 bytes.
     * - Unsigned integers, up to 8 bytes.
     * - IEEE 4-byte and 8-byte floats.
     * - Strings, with or without null-byte padding on the end.
     * - Dates, represented as a 64-bit integer giving the number of
     *   nanoseconds since 2001-01-01 00:00:00.
     * - Raw binary data.
     *
     * A default value can be specified for the element. If a default is given,
     * it may allow the element to be skipped when writing to a store.
     *
     * Any type specified for T must initialise itself upon construction, be
     * copy-constructable, have an assignment operator, and perform its own
     * clean-up in its destructor. POD types qualify for this.
     */
    template<typename T>
    class TAWARA_EXPORT PrimitiveElement : public Element,
        public boost::equality_comparable<PrimitiveElement<T> >
    {
        public:
            /** \brief Create a new element with no default.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             */
            PrimitiveElement(uint32_t id, T value)
                : Element(id),
                value_(value), has_default_(false)
            {
            }

            /** \brief Create a signed integer element with a default value.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] value The element's value.
             * \param[in] default_value The default value of the element.
             */
            PrimitiveElement(uint32_t id, T value, T default_value)
                : Element(id),
                value_(value), default_(default_value), has_default_(true)
            {
            }

            /// \brief Value assignment operator.
            virtual PrimitiveElement& operator=(T const& rhs)
            {
                value_ = rhs;
                return *this;
            }

            /// Get the element's ID.
            virtual uint32_t id() const { return Element::id(); }

            /** \brief Set the element's ID.
             *
             * \param[in] id The element's new ID, as an unsigned integer up to
             * 28 bits.
             */
            virtual void id(uint32_t id)
            {
                if (id == 0 ||
                        id == 0xFF ||
                        id == 0xFFFF ||
                        id == 0xFFFFFF ||
                        id == 0xFFFFFFFF)
                {
                    throw InvalidElementID() << err_id(id);
                }
                id_ = id;
            }

            /// \brief Get the value.
            virtual T value() const { return value_; }
            /// \brief Set the value.
            virtual void value(T value) { value_ = value; }
            /// \brief Cast to the stored type.
            operator T() const { return value_; }

            /// \brief Check if a default value is set.
            virtual bool has_default() const { return has_default_; }
            /// \brief Get the default value.
            virtual T get_default() const { return default_; }
            /// \brief Set the default value.
            virtual void set_default(T default_value)
            {
                default_ = default_value;
                has_default_ = true;
            }
            /** \brief Remove the default value.
             *
             * \return The value of the default that was removed.
             */
            virtual T remove_default()
            {
                has_default_ = false;
                return default_;
            }
            /** Check if this element is at the default value.
             *
             * If the current value is the same as the default value, this
             * element may not need to be stored when being written.
             */
            virtual bool is_default() const
                { return value_ == default_ && has_default_; }

            /// \brief Equality operator.
            friend bool operator==(PrimitiveElement<T> const& lhs,
                    PrimitiveElement<T> const& rhs)
            {
                return lhs.value_ == rhs.value_;
            }

        protected:
            T value_;
            T default_;
            bool has_default_;

            virtual bool equal_(PrimitiveElement<T> const& rhs)
            {
                return value_ == rhs.value_;
            }
    }; // class Element
}; // namespace tawara

/// @}
/// group interfaces

#endif // TAWARA_PRIM_ELEMENT_H_

