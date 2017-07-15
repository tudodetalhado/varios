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

#if !defined(TAWARA_EBML_ELEMENT_H_)
#define TAWARA_EBML_ELEMENT_H_

#include <tawara/el_ids.h>
#include <tawara/master_element.h>
#include <tawara/tawara_config.h>
#include <tawara/string_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

#include <string>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The EBML Header element.
     *
     * This is the Header element as defined in the EBML RFC draft. It is
     * \b required to be the first element in an EBML document.
     *
     * The header defines various meta-data about the EBML document to follow.
     */
    class TAWARA_EXPORT EBMLElement : public MasterElement
    {
        public:
            /** \brief Create a new Element.
             *
             * \param[in] doc_type The DocType that will be created. When
             * reading, if it is blank, it will be populated. If it is not
             * blank, then the value in the file will be checked against the
             * value specified here and IncorrectDocType will be thrown if they
             * do not match.
             */
            EBMLElement(std::string const& doc_type="tawara");

            /// \brief Destructor.
            virtual ~EBMLElement() {};

            /** \brief Get the EBML version.
             *
             * The EBML version is the version of the parser used to create the
             * file. It is set internally by the Tawara EBML parser, and read
             * from the file.
             */
            unsigned int version() const { return ver_.value(); }
            /** \brief Get the EBML read version.
             *
             * This property gives the minimum EBML parser version necessary to
             * read the file. It is set internally by the Tawara EBML parser, and
             * read from the file.
             */
            unsigned int read_version() const { return read_ver_.value(); }
            /** \brief Get the maximum ID length.
             *
             * This is the maximum length of IDs that can be used in a file
             * with this header.
             */
            unsigned int max_id_length() const
                { return max_id_length_.value(); }
            /// \brief Set the maximum ID length.
            void max_id_length(unsigned int max_id_length)
                { max_id_length_.value(max_id_length); }
            /** \brief Get the maximum size length.
             *
             * This is the maximum number of bytes that can be used for an
             * element size in a file with this header.
             */
            unsigned int max_size_length() const
                { return max_size_length_.value(); }
            /// \brief Set the maximum size length.
            void max_size_length(unsigned int max_size_length)
                { max_size_length_.value(max_size_length); }
            /** \brief Get the document type.
             *
             * The document type is the type of EBML document that is read or
             * written.
             */
            std::string doc_type() const { return doc_type_.value(); }
            /// \brief Set the document type.
            void doc_type(std::string doc_type) { doc_type_.value(doc_type); }
            /** \brief Get the document type version.
             *
             * This is the version of the document type contained in the file.
             */
            unsigned int doc_version() const { return doc_type_ver_.value(); }
            /// \brief Set the document type version.
            void doc_version(unsigned int doc_version)
                { doc_type_ver_.value(doc_version); }
            /** \brief Get the minimum document type version to read.
             *
             * This is the minimum document type version that must be readable
             * for a Tawara parser to read this file.
             */
            unsigned int doc_read_version() const
                { return doc_type_read_ver_.value(); }
            /// \brief Set the minimum required document type version.
            void doc_read_version(unsigned int doc_read_version)
                { doc_type_read_ver_.value(doc_read_version); }

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

        protected:
            /// EBML version
            UIntElement ver_;
            /// EBML minimum-version-to-read
            UIntElement read_ver_;
            /// Maximum ID length in bytes
            UIntElement max_id_length_;
            /// Maximum size length in bytes
            UIntElement max_size_length_;
            /// EBML document type
            StringElement doc_type_;
            /// Document type version
            UIntElement doc_type_ver_;
            /// Minimum document type version necessary to read
            UIntElement doc_type_read_ver_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /// \brief Sets all child elements to their default values.
            void set_defaults_();
    }; // class Element
}; // namespace tawara

/// @}
/// group interfaces

#endif // TAWARA_EBML_ELEMENT_H_

