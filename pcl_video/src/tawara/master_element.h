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

#if !defined(TAWARA_MASTER_ELEMENT_H_)
#define TAWARA_MASTER_ELEMENT_H_

#include <tawara/element.h>
#include <tawara/win_dll.h>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The MasterElement interface.
     *
     * EBML elements can be divided into primitive elements and master
     * elements. While primitive elements directly store a single piece of data
     * in a format corresponding to their type, master elements are used to
     * group other elements together. Instead of a single data value, they
     * store zero or more child elements, which can themselves be master or
     * primitive elements.
     *
     * Generally, each master element will have quite specific functionality.
     * In such cases, a new class should be created that inherits from and
     * implements this interface.
     */
    class TAWARA_EXPORT MasterElement : public Element
    {
        public:
            /** \brief Create a new MasterElement.
             *
             * \param[in] id The element's ID, as an unsigned integer up to 28
             * bits.
             * \param[in] crc Use a CRC32 element to provide a check for file
             * corruption.
             */
            MasterElement(uint32_t id, bool crc=false);

            /// \brief Destructor
            virtual ~MasterElement() {};

        private:
            bool crc_;
    }; // class MasterElement
}; // namespace tawara

/// @}
// group interfaces

#endif // TAWARA_MASTER_ELEMENT_H_

