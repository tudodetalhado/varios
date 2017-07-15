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

#if !defined(TAWARA_ATTACHMENTS_H_)
#define TAWARA_ATTACHMENTS_H_

#include <boost/operators.hpp>
#include <boost/shared_ptr.hpp>
#include <tawara/binary_element.h>
#include <tawara/master_element.h>
#include <tawara/string_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The data of an attached file.
     *
     * The data in a single attachment is stored as a binary blob, using an
     * EBML binary element.
     */
    class TAWARA_EXPORT FileData : public BinaryElement
    {
        public:
            /// \brief Constructor.
            FileData(std::vector<char> data)
                : BinaryElement(ids::FileData, data)
            {
            }

            /// \brief Type of a pointer to a FileData instance.
            typedef boost::shared_ptr<FileData> Ptr;
            /// \brief Type of a pointer to a const FileData instance.
            typedef boost::shared_ptr<FileData const> ConstPtr;
    }; // class FileData

    /** \brief An attachment is a binary blob attached to a segment.
     *
     * This object contains a single attachment. Instances are stored in the
     * Attachments class.
     */
    class TAWARA_EXPORT AttachedFile : public MasterElement,
            public boost::equality_comparable<AttachedFile>
    {
        public:
            /// \brief Constructor.
            AttachedFile();

            /** \brief Constructor.
             *
             * \param[in] name The file name of the attachemnt.
             * \param[in] mime_type The MIME type of the stored file.
             * \param[in] data The data to be attached.
             * \param[in] uid A unique UID to represent the attachment.
             */
            AttachedFile(std::string const& name,
                    std::string const& mime_type,
                    FileData::Ptr data,
                    uint64_t uid);

            /** \brief Get the attachment's description.
             *
             * The attachment's description is a human-friendly name for the
             * attached file.
             */
            std::string description() const { return desc_; }
            /// \brief Set the attachment's description.
            void description(std::string const& desc) { desc_ = desc; }

            /// \brief Get the attachment's file name.
            std::string name() const { return name_; }
            /// \brief Set the attachment's file name.
            void name(std::string const& name) { name_ = name; }

            /** Get the MIME type of the file.
             *
             * The MIME type is used to identify the type of file stored.
             * Without this, programs reading the file cannot easily recognise
             * the type of file and so how to use it.
             */
            std::string mime_type() const { return mime_; }
            /// \brief Set the MIME type of the file.
            void mime_type(std::string const& mime_type) { mime_ = mime_type; }

            /** Get the file data.
             *
             * The attched file data is stored as a binary blob. How it is
             * interpreted is up to the reading program, and is usually guided
             * by the stored MIME type.
             */
            FileData::ConstPtr data() const { return data_; }
            /// Set the file data.
            void data(FileData::Ptr& data);

            /** \brief Get the attached file's UID.
             *
             * The UID for the attached file is used to identify it within the
             * segment. It should be as unique as possible.
             */
            uint64_t uid() const { return uid_; }
            /// \brief Set the attached file's UID.
            void uid(uint64_t uid);

            /// \brief Equality operator.
            friend bool operator==(AttachedFile const& lhs,
                    AttachedFile const& rhs);

        protected:
            StringElement desc_;
            StringElement name_;
            StringElement mime_;
            FileData::Ptr data_;
            UIntElement uid_;

            /////////////////////
            // Element interface
            /////////////////////

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief Reset the values to their defaults
            void reset();
    }; // class AttachedFile

    /// \brief Equality operator for the AttachedFile object.
    bool operator==(AttachedFile const& lhs, AttachedFile const& rhs);


    /** \brief The Attachments element stores opaque data attached to a
     * segment.
     *
     * The Attachments element is used to store binary blobs that are attached
     * to the segment. An attachment may contain any data at all. Because
     * attachments can be linked to specific tracks, they are sometimes used
     * for storing such things as the definition file (e.g. an IDL file) of the
     * data type stored in the track, or a binary library containing a decoder
     * for the track's codec.
     */
    class TAWARA_EXPORT Attachments : public MasterElement,
            public boost::equality_comparable<Attachments>
    {
        public:
            /// \brief The value type of this container.
            typedef std::vector<AttachedFile>::value_type value_type;
            /// \brief The size type of this container.
            typedef std::vector<AttachedFile>::size_type size_type;
            /// \brief The reference type.
            typedef std::vector<AttachedFile>::reference reference;
            /// \brief The constant reference type.
            typedef std::vector<AttachedFile>::const_reference const_reference;
            /// \brief The random access iterator type.
            typedef std::vector<AttachedFile>::iterator iterator;
            /// \brief The constant random access iterator type.
            typedef std::vector<AttachedFile>::const_iterator const_iterator;
            /// \brief The reversed random access iterator type.
            typedef std::vector<AttachedFile>::reverse_iterator reverse_iterator;
            /// \brief The constant reversed random access iterator type.
            typedef std::vector<AttachedFile>::const_reverse_iterator
                const_reverse_iterator;

            /// \brief Constructor.
            Attachments();

            /** \brief Get the attachment at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified attachment.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type& at(size_type pos)
                { return files_.at(pos); }
            /** \brief Get the attachment at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified attachment.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type const& at(size_type pos) const
                { return files_.at(pos); }

            /** \brief Get a reference to an attachment. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a attachment stored in this
             * block.
             */
            virtual value_type& operator[](size_type pos)
                { return files_[pos]; }
            /** \brief Get a reference to an attachment. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a attachment stored in this
             * block.
             */
            virtual value_type const& operator[](size_type pos) const
                { return files_[pos]; }

            /// \brief Get an iterator to the first attachment.
            virtual iterator begin() { return files_.begin(); }
            /// \brief Get an iterator to the first attachment.
            virtual const_iterator begin() const { return files_.begin(); }
            /// \brief Get an iterator to the position past the last attachment.
            virtual iterator end() { return files_.end(); }
            /// \brief Get an iterator to the position past the last attachment.
            virtual const_iterator end() const { return files_.end(); }
            /// \brief Get a reverse iterator to the last attachment.
            virtual reverse_iterator rbegin() { return files_.rbegin(); }
            /// \brief Get a reverse iterator to the last attachment.
            virtual const_reverse_iterator rbegin() const
                { return files_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * attachment.
             */
            virtual reverse_iterator rend() { return files_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * attachment.
             */
            virtual const_reverse_iterator rend() const { return files_.rend(); }

            /** \brief Check if there are no attachments.
             *
             * An empty Attachments element may not occur in a Tawara file. If
             * this returns true, an error will occur when write() is called.
             */
            virtual bool empty() const { return files_.empty(); }
            /// \brief Get the number of attachments.
            virtual size_type count() const { return files_.size(); }
            /// \brief Get the maximum number of attachments.
            virtual size_type max_count() const { return files_.max_size(); }

            /// \brief Remove all attachments.
            virtual void clear() { files_.clear(); }

            /** \brief Erase the attachment at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(iterator position) { files_.erase(position); }
            /** \brief Erase a range of attachments.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(iterator first, iterator last)
                { files_.erase(first, last); }

            /// \brief Add an attachment.
            virtual void push_back(value_type const& value)
                { files_.push_back(value); }

            /// \brief Resizes the attachments storage.
            virtual void resize(size_type count) { files_.resize(count); }

            /** \brief Swaps the contents of this Attachments element with
             * another.
             *
             * \param[in] other The other Attachments element
             */
            virtual void swap(Attachments& other) { files_.swap(other.files_); }

            /// \brief Equality operator.
            friend bool operator==(Attachments const& lhs,
                    Attachments const& rhs);

        protected:
            std::vector<AttachedFile> files_;

            /////////////////////
            // Element interface
            /////////////////////

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class Attachments

    /// \brief Equality operator for the Attachemnts object.
    bool operator==(Attachments const& lhs, Attachments const& rhs);
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_ATTACHMENTS_H_

