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

#if !defined(TAWARA_TRACKS_H_)
#define TAWARA_TRACKS_H_

#include <boost/operators.hpp>
#include <map>
#include <tawara/master_element.h>
#include <tawara/track_entry.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The Tracks element, listing all tracks in the segment.
     *
     * The Tracks element contains the list of tracks that have data in the
     * segment. Each track is represented by a TrackEntry element, and each has
     * a number and a UID. The numbers and UIDs must be unique within the
     * segment, and UIDs should be as unique as possible.
     */
    class TAWARA_EXPORT Tracks : public MasterElement,
            public boost::equality_comparable<Tracks>
    {
        public:
            /// \brief The key type (Key) of this container.
            typedef uint64_t key_type;
            /// \brief The mapped type (T) of this container.
            typedef TrackEntry::Ptr mapped_type;

        protected:
            /// \brief The type of the internal storage.
            typedef std::map<key_type, mapped_type> storage_type_;

        public:
            /** \brief Construct a new Tracks element.
             *
             * Upon construction, the list of tracks will be empty. At least
             * one TrackEntry must be added before writing the element.
             */
            Tracks();

            /// \brief Destructor.
            virtual ~Tracks() {};

            /// \brief The value type of this container.
            typedef storage_type_::value_type value_type;
            /// \brief The size type of this container.
            typedef storage_type_::size_type size_type;
            /// \brief The reference type.
            typedef storage_type_::reference reference;
            /// \brief The constant reference type.
            typedef storage_type_::const_reference const_reference;
            /// \brief The random access iterator type.
            typedef storage_type_::iterator iterator;
            /// \brief The constant random access iterator type.
            typedef storage_type_::const_iterator const_iterator;
            /// \brief The reversed random access iterator type.
            typedef storage_type_::reverse_iterator reverse_iterator;
            /// \brief The constant reversed random access iterator type.
            typedef storage_type_::const_reverse_iterator
                const_reverse_iterator;

            /** \brief Get the TrackEntry with the given track number.
             *
             * \return A reference to the specified TrackEntry.
             * \throw std::out_of_range if the track number is invalid.
             */
            mapped_type& at(key_type const& pos)
                { return entries_.at(pos); }
            /** \brief Get the TrackEntry with the given track number.
             *
             * \return A reference to the specified TrackEntry.
             * \throw std::out_of_range if the track number is invalid.
             */
            mapped_type const& at(key_type const& pos) const
                { return entries_.at(pos); }

            /** \brief Gets a reference to the TrackEntry with the given track
             * number.
             *
             * \return A reference to a TrackEntry with the given track number.
             * \throw std::out_of_range if the track number is invalid.
             */
            mapped_type& operator[](key_type const& key);
            /** \brief Gets a reference to the TrackEntry with the given track
             * number.
             *
             * \return A reference to a TrackEntry with the given track number.
             * \throw std::out_of_range if the track number is invalid.
             */
            mapped_type const& operator[](key_type const& key) const;

            /// \brief Get an iterator to the first TrackEntry.
            iterator begin() { return entries_.begin(); }
            /// \brief Get an iterator to the first TrackEntry.
            const_iterator begin() const { return entries_.begin(); }
            /// \brief Get an iterator to the position past the last TrackEntry.
            iterator end() { return entries_.end(); }
            /// \brief Get an iterator to the position past the last TrackEntry.
            const_iterator end() const { return entries_.end(); }
            /// \brief Get a reverse iterator to the last TrackEntry.
            reverse_iterator rbegin() { return entries_.rbegin(); }
            /// \brief Get a reverse iterator to the last TrackEntry.
            const_reverse_iterator rbegin() const { return entries_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * TrackEntry.
             */
            reverse_iterator rend() { return entries_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * TrackEntry.
             */
            const_reverse_iterator rend() const { return entries_.rend(); }

            /// \brief Check if there are no TrackElements.
            bool empty() const { return entries_.empty(); }
            /// \brief Get the number of TrackElements.
            size_type count() const { return entries_.size(); }
            /// \brief Get the maximum number of TrackElements.
            size_type max_count() const { return entries_.max_size(); }

            /// \brief Remove all TrackElements.
            void clear() { entries_.clear(); }
            /** \brief Insert a new TrackElement.
             *
             * If a TrackElement already exists with the same track number, the
             * new one will not replace it, and the return code will indicate
             * that no insertion took place.
             *
             * \param[in] value The TrackEntry to insert. Its track number will
             * be used as the key.
             *
             * \return A pair of the iterator at the position where the
             * TrackEntry was added (or blocked) and a boolean indicating if
             * the insertion took place.
             * \throw DuplicateTrackNumber if an existing track entry uses the
             * same track number.
             * \throw DuplicateUID if an existing track entry uses the same
             * UID.
             */
            std::pair<iterator, bool> insert(mapped_type const& value);
            /** \brief Insert a range of TrackElements.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             * \throw DuplicateTrackNumber if an existing track entry uses the
             * same track number.
             * \throw DuplicateUID if an existing track entry uses the same
             * UID.
             */
            void insert(const_iterator first, const_iterator last);
            /** \brief Erase the TrackEntry at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            void erase(iterator position)
                { entries_.erase(position); }
            /** \brief Erase a range of TrackEntries.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            void erase(iterator first, iterator last)
                { entries_.erase(first, last); }
            /** \brief Erase the TrackEntry with the given track number.
             *
             * \param[in] number The track number to erase.
             * \return The number of TrackEntries erased.
             */
            size_type erase(key_type const& number)
                { return entries_.erase(number); }
            /** \brief Swaps the contents of this Tracks element with another.
             *
             * \param[in] other The other Tracks element to swap with.
             */
            void swap(Tracks& other)
                { entries_.swap(other.entries_); }

            /** \brief Search for the TrackEntry with the given track number.
             *
             * \param[in] number The track number to search for.
             * \return An iterator to the matching TrackEntry, or end() if
             * there is no TrackEntry with that number.
             */
            iterator find(key_type const& number)
                { return entries_.find(number); }
            /** \brief Search for the TrackEntry with the given track number.
             *
             * \param[in] number The track number to search for.
             * \return An iterator to the matching TrackEntry, or end() if
             * there is no TrackEntry with that number.
             */
            const_iterator find(key_type const& number) const
                { return entries_.find(number); }

            /// \brief Equality operator.
            friend bool operator==(Tracks const& lhs, Tracks const& rhs);

        protected:
            /** \brief The track entry store.
             *
             * This must always be sorted by TrackNumber.
             */
            storage_type_ entries_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /** \brief Element body writing.
             *
             * \throw DuplicateTrackNumber if more than one TrackEntry in the
             * element has the same track number.
             * \throw DuplicateUID if more than one TrackEntry in the element
             * has the same UID.
             */
            virtual std::streamsize write_body(std::ostream& output);

            /** \brief Element body loading.
             *
             * \throw DuplicateTrackNumber if more than one TrackEntry in the
             * stored element has the same track number.
             * \throw DuplicateUID if more than one TrackEntry in the stored
             * element has the same UID.
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /** \brief Checks for duplicate track numbers and UIDs.
             *
             * \throw DuplicateTrackNumber if more than one TrackEntry in the
             * element has the same track number.
             * \throw DuplicateUID if more than one TrackEntry in the element
             * has the same UID.
             */
            void validate_entries() const;

            /** \brief Looks for a duplicate track entry, throws an exception
             * if it is.
             *
             * This function searchers the stored TrackEntry elements for one
             * with the same track number OR track UID as the given entry. The
             * result is indicated by throwing an exception. No exception means
             * the TrackEntry is unique.
             *
             * \param[in] entry The entry to look for a duplicate for.
             * \throw DuplicateTrackNumber if more than one TrackEntry in the
             * element has the same track number.
             * \throw DuplicateUID if more than one TrackEntry in the element
             * has the same UID.
             */
            void verify_not_duplicate(TrackEntry::Ptr entry) const;
    }; // class Tracks

    bool operator==(Tracks const& lhs, Tracks const& rhs);
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_TRACKS_H_

