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

#if !defined(TAWARA_CUES_H_)
#define TAWARA_CUES_H_

#include <boost/operators.hpp>
#include <boost/shared_ptr.hpp>
#include <tawara/master_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>
#include <vector>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The position in the segment of the data for a single track.
     *
     * The CueTackPositions element stores the position in the segment's data
     * for the data for a single track. It is used by a CuePoint to hold the
     * track positions for a timecode.
     */
    class TAWARA_EXPORT CueTrackPosition : public MasterElement,
            public boost::equality_comparable<CueTrackPosition>
    {
        public:
            /// \brief Constructor.
            CueTrackPosition();

            /** \brief Constructor.
             *
             * \param[in] track Track number.
             * \param[in] cluster_pos Cluster position.
             */
            CueTrackPosition(uint64_t track, uint64_t cluster_pos);

            /// \brief The track number this element contains positions for.
            uint64_t track() const { return track_; }
            /// \brief Set the track number.
            void track(uint64_t track);

            /** \brief Get the cluster position.
             *
             * The cluster position is the offset in bytes of the cluster in
             * the segment.
             */
            uint64_t cluster_pos() const { return cluster_pos_; }
            /// \brief Set the cluster position.
            void cluster_pos(uint64_t cluster_pos)
                { cluster_pos_ = cluster_pos; }

            /** \brief Get the number of the relevant block.
             *
             * This gives the one-based index of the block corresponding to
             * the timecode in its cluster. Must not be zero.
             */
            uint64_t block_num() const { return block_num_; }
            /// \brief Set the block number of the relevant block.
            void block_num(uint64_t block_num);

            /** \brief Get the index of the relevant codec state.
             *
             * This index gives the relevant codec state to use when jumping to
             * the timecode.
             */
            uint64_t codec_state() const { return codec_state_; }
            /// \brief Set the index of the codec state.
            void codec_state(uint64_t codec_state)
                { codec_state_ = codec_state; }

            /** \brief Get the vector of reference block timecodes.
             *
             * If the block this element points to depends on reference blocks
             * for decoding, this vector contains the timecodes of those
             * blocks.
             */
            std::vector<uint64_t>& reference_times()
                { return ref_blocks_; }

            /// \brief Equality operator.
            friend bool operator==(CueTrackPosition const& lhs,
                    CueTrackPosition const& rhs);

        protected:
            UIntElement track_;
            UIntElement cluster_pos_;
            UIntElement block_num_;
            UIntElement codec_state_;
            std::vector<uint64_t> ref_blocks_;

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

            /// \brief Read an occurance of the CueReference element.
            std::streamsize read_cue_reference(std::istream& input,
                    std::streamsize size);

            /// \brief Reset the values to their defaults
            void reset();
    }; // class CueTrackPosition

    /// \brief Equality operator for the CueTrackPosition element.
    bool operator==(CueTrackPosition const& lhs,
            CueTrackPosition const& rhs);


    /** \brief A CuePoint is an index from a timecode to one or more
     * cluster/block positions.
     *
     * The CuePoint element stores the index for a single timecode, providing
     * the location in the segment of the relevant cluster (and, often, block)
     * that corresponds to that timecode for each track active at that
     * timecode. The track positions are accessible through a map interface.
     *
     * A CuePoint must contain at least one track position before it can be
     * written to a segment.
     */
    class TAWARA_EXPORT CuePoint : public MasterElement,
            public boost::equality_comparable<CuePoint>
    {
        protected:
            /// \brief The storage type.
            typedef std::vector<CueTrackPosition> storage_type_;
        public:
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
            /// \brief The pointer to this type.
            typedef boost::shared_ptr<CuePoint> Ptr;

            /// \brief Constructor.
            CuePoint();

            /** \brief Constructor.
             *
             * \param[in] timecode The timecode of the cue point.
             */
            CuePoint(uint64_t timecode);

            /** \brief Get the timecode of this cue point.
             *
             * The cue point's timecode is used when searching for the closest
             * cue point to the desired time.
             */
            uint64_t timecode() const { return timecode_; }
            /// \brief Set the timecode.
            void timecode(uint64_t timecode) { timecode_ = timecode; }

            /** \brief Get the CueTracksPosition at the given position, with
             * bounds checking.
             *
             * \return A reference to the specified CueTracksPosition.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type& at(size_type pos)
                { return positions_.at(pos); }
            /** \brief Get the CueTracksPosition at the given position, with
             * bounds checking.
             *
             * \return A reference to the specified CueTracksPosition.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type const& at(size_type pos) const
                { return positions_.at(pos); }

            /** \brief Get a reference to a CueTracksPosition. No bounds
             * checking is performed.
             */
            virtual value_type& operator[](size_type pos)
                { return positions_[pos]; }
            /** \brief Get a reference to a CueTracksPosition. No bounds
             * checking is performed.
             */
            virtual value_type const& operator[](size_type pos) const
                { return positions_[pos]; }

            /// \brief Get an iterator to the first cue.
            virtual iterator begin() { return positions_.begin(); }
            /// \brief Get an iterator to the first cue.
            virtual const_iterator begin() const { return positions_.begin(); }
            /// \brief Get an iterator to the position past the last cue.
            virtual iterator end() { return positions_.end(); }
            /// \brief Get an iterator to the position past the last cue.
            virtual const_iterator end() const { return positions_.end(); }
            /// \brief Get a reverse iterator to the last cue.
            virtual reverse_iterator rbegin() { return positions_.rbegin(); }
            /// \brief Get a reverse iterator to the last cue.
            virtual const_reverse_iterator rbegin() const
                { return positions_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * cue.
             */
            virtual reverse_iterator rend() { return positions_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * cue.
             */
            virtual const_reverse_iterator rend() const
                { return positions_.rend(); }

            /** \brief Check if there are no cue positions.
             *
             * An empty CuePoint element may not occur in a Tawara file. If this
             * returns true, an error will occur when write() is called.
             */
            virtual bool empty() const { return positions_.empty(); }
            /// \brief Get the number of cue positions.
            virtual size_type count() const { return positions_.size(); }
            /// \brief Get the maximum number of cue positions.
            virtual size_type max_count() const
                { return positions_.max_size(); }

            /// \brief Remove all cue positions.
            virtual void clear() { positions_.clear(); }

            /** \brief Erase the CueTrackPosition at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(iterator position)
                { positions_.erase(position); }
            /** \brief Erase a range of CueTrackPosition.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(iterator first, iterator last)
                { positions_.erase(first, last); }

            /// \brief Add a CueTrackPosition.
            virtual void push_back(value_type const& value)
                { positions_.push_back(value); }

            /// \brief Resizes the vector.
            virtual void resize(size_type count) { positions_.resize(count); }

            /** \brief Swaps the contents of this CuePoint element with
             * another.
             *
             * \param[in] other The other CuePoint element
             */
            virtual void swap(CuePoint& other)
                { positions_.swap(other.positions_); }

            /// \brief Equality operator.
            friend bool operator==(CuePoint const& lhs, CuePoint const& rhs);

        protected:
            UIntElement timecode_;
            storage_type_ positions_;

            /////////////////////
            // Element interface
            /////////////////////

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

            /** \brief Element body loading.
             *
             * \throw DuplicateTimecode if a CuePoint is read for a timecode
             * that already exists in the map.
             */
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class CuePoint

    /// \brief Equality operator for the CuePoint element.
    bool operator==(CuePoint const& lhs, CuePoint const& rhs);


    /** \brief The Cues element provides a list of indexes into the blocks for
     * specific timecodes.
     *
     * The Cues element stores a list of CuePoints, each of which is a timecode
     * and the positions of clusters and/or blocks that correspond to that
     * timecode (typically one for each track active at that timecode). The
     * resolution of cues (i.e. how far apart they are in time) is left up to
     * the writer of the Tawara file. Generally, at a minimum, one cue point
     * should exist for the start of each cluster in the segment. Another
     * approach is one cue point per set time interval.
     *
     * The cue points are accessible through a map interface, with the timecode
     * used as the key.
     *
     * The Cues must contain at least one CuePoint before it can be written to
     * a segment.
     */
    class TAWARA_EXPORT Cues : public MasterElement,
            public boost::equality_comparable<Cues>
    {
        public:
            /// \brief The key type (Key) of this container.
            typedef uint64_t key_type;
            /// \brief The mapped type (T) of this container.
            typedef CuePoint mapped_type;

        protected:
            /// \brief The type of the internal storage.
            typedef std::map<key_type, mapped_type> storage_type_;

        public:
            /// \brief Constructor.
            Cues();

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

            /** \brief Get the CuePoint with the given timecode.
             *
             * \return A reference to the specified CuePoint.
             * \throw std::out_of_range if the timecode is invalid.
             */
            mapped_type& at(key_type const& pos)
                { return cues_.at(pos); }
            /** \brief Get the CuePoint with the given timecode.
             *
             * \return A reference to the specified CuePoint.
             * \throw std::out_of_range if the timecode is invalid.
             */
            mapped_type const& at(key_type const& pos) const
                { return cues_.at(pos); }

            /** \brief Gets a reference to the CuePoint with the given
             * timecode, without range checking.
             *
             * \return A reference to a CuePoint with the given timecode.
             */
            mapped_type& operator[](key_type const& key)
                { return cues_[key]; }
            /** \brief Gets a reference to the CuePoint with the given
             * timecode, without range checking.
             *
             * \return A reference to a CuePoint with the given timecode.
             */
            mapped_type const& operator[](key_type const& key) const
                { return cues_.find(key)->second; }

            /// \brief Get an iterator to the first CuePoint.
            iterator begin() { return cues_.begin(); }
            /// \brief Get an iterator to the first CuePoint.
            const_iterator begin() const { return cues_.begin(); }
            /// \brief Get an iterator to the position past the last CuePoint.
            iterator end() { return cues_.end(); }
            /// \brief Get an iterator to the position past the last CuePoint.
            const_iterator end() const { return cues_.end(); }
            /// \brief Get a reverse iterator to the last CuePoint.
            reverse_iterator rbegin() { return cues_.rbegin(); }
            /// \brief Get a reverse iterator to the last CuePoint.
            const_reverse_iterator rbegin() const { return cues_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * CuePoint.
             */
            reverse_iterator rend() { return cues_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * CuePoint.
             */
            const_reverse_iterator rend() const { return cues_.rend(); }

            /// \brief Check if there are no CuePoints.
            bool empty() const { return cues_.empty(); }
            /// \brief Get the number of CuePoints.
            size_type count() const { return cues_.size(); }
            /// \brief Get the maximum number of CuePoints.
            size_type max_count() const { return cues_.max_size(); }

            /// \brief Remove all CuePoints.
            void clear() { cues_.clear(); }
            /** \brief Insert a new CuePoint.
             *
             * If a CuePoint already exists with the same track number, the new
             * one will not replace it, and the return code will indicate that
             * no insertion took place.
             *
             * \param[in] value The CuePoint to insert. Its timecode will be
             * used as the key.
             *
             * \return A pair of the iterator at the position where the
             * CuePoint was added (or blocked) and a boolean indicating if
             * the insertion took place.
             */
            std::pair<iterator, bool> insert(mapped_type const& value)
                { return cues_.insert(std::make_pair(value.timecode(), value)); }
            /** \brief Insert a range of CuePoints.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            void insert(const_iterator first, const_iterator last)
                { cues_.insert(first, last); }
            /** \brief Erase the CuePoint at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            void erase(iterator position)
                { cues_.erase(position); }
            /** \brief Erase a range of CuePoints.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            void erase(iterator first, iterator last)
                { cues_.erase(first, last); }
            /** \brief Erase the CuePoint with the given timecode.
             *
             * \param[in] number The timecode to erase.
             * \return The number of CuePoints erased.
             */
            size_type erase(key_type const& number)
                { return cues_.erase(number); }
            /** \brief Swaps the contents of this Cues element with another.
             *
             * \param[in] other The other Cues element to swap with.
             */
            void swap(Cues& other) { cues_.swap(other.cues_); }

            /** \brief Search for the CuePoint with the given timecode.
             *
             * \param[in] number The timecode to search for.
             * \return An iterator to the matching CuePoint, or end() if there
             * is no CuePoint with that number.
             */
            iterator find(key_type const& number)
                { return cues_.find(number); }
            /** \brief Search for the CuePoint with the given timecode.
             *
             * \param[in] number The timecode to search for.
             * \return An iterator to the matching CuePoint, or end() if there
             * is no CuePoint with that number.
             */
            const_iterator find(key_type const& number) const
                { return cues_.find(number); }

            /// \brief Equality operator.
            friend bool operator==(Cues const& lhs, Cues const& rhs);

        protected:
            storage_type_ cues_;

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
    }; // class Cues

    /// \brief Equality operator for the Cues element.
    bool operator==(Cues const& lhs, Cues const& rhs);
}; // namespace tawara;

/// @}
// group elements

#endif // TAWARA_CUES_H_

