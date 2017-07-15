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

#if !defined(TAWARA_TRACK_ENTRY_H_)
#define TAWARA_TRACK_ENTRY_H_

#include <boost/operators.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <tawara/binary_element.h>
#include <tawara/float_element.h>
#include <tawara/master_element.h>
#include <tawara/string_element.h>
#include <tawara/track_operation.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief A track entry provides the meta-data for a single track.
     *
     * All the necessary information to understand the data for a track is
     * stored in the TrackEntry element. This includes things such as the codec
     * used, private data stored by the codec (such as invariant sensor
     * information), and playback settings.
     *
     * Some tracks may be virtual, which means that they don't have any data of
     * their own. Instead, they use data from other tracks, combined using one
     * or more specified operations.
     */
    class TAWARA_EXPORT TrackEntry : public MasterElement,
            public boost::equality_comparable<TrackEntry>
    {
        public:
            /** \brief Construct a new TrackEntry.
             *
             * \param[in] number The track number. Must not be zero.
             * \param[in] uid The track's unique identifier. Must not be zero.
             * \param[in] codec The ID of the codec used in this track.
             */
            TrackEntry(uint64_t number, uint64_t uid,
                    std::string const& codec);

            /// \brief Destructor.
            virtual ~TrackEntry() {}

            /** \brief Get the track number.
             *
             * The track's number is used to identify which track a block
             * belongs to. It must not be zero.
             */
            uint64_t number() const { return number_; }
            /** \brief Set the track number.
             *
             * \throw ValueOutOfRange if a value outside the allowable range is
             * set.
             */
            void number(uint64_t number);

            /** \brief Get the track UID.
             *
             * The track's UID is used to uniquely identify it. Generally, when
             * making a direct stream copy of the track, the UID should be
             * preserved. It must not be zero.
             */
            uint64_t uid() const { return uid_; }
            /** \brief Set the track UID.
             *
             * \throw ValueOutOfRange if a value outside the allowable range is
             * set.
             */
            void uid(uint64_t uid);

            /** \brief Get the track type.
             *
             * The track's type may be one of the values allowed by the
             * specification. Usually, 0x70 will be used, indicating a data
             * track.
             */
            uint8_t type() const { return type_; }
            /** \brief Set the track type.
             *
             * \throw ValueOutOfRange if a value outside the allowable range is
             * set.
             */
            void type(uint8_t type);

            /** \brief Check if the track is enabled.
             *
             * Disabled tracks are typically not played back.
             *
             * The default is true.
             */
            bool enabled() const { return enabled_; }
            /// \brief Set if the track is enabled.
            void enabled(bool enabled) { enabled_ = enabled; }

            /** \brief Check if the track is forced.
             *
             * A forced track MUST be played back.
             *
             * The default is false.
             */
            bool forced() const { return forced_; }
            /// \brief Set if the track is forced.
            void forced(bool forced) { forced_ = forced; }

            /** \brief Check if the track can use lacing.
             *
             * If the track can use lacing, it means that it may store multiple
             * frames of data in a single block. This is used to reduce
             * overhead when the size of an individual frame is small relative
             * to the size of Tawara data around it.
             *
             * A value of true does not imply that the track must use lacing,
             * only that it may.
             *
             * The default is true.
             */
            bool lacing() const { return lacing_; }
            /// \brief Set if the track can use lacing.
            void lacing(bool lacing) { lacing_ = lacing; }

            /** \brief Get the minimum cache size.
             *
             * This value gives the minimum number of blocks that should be
             * cached before beginning playback. Generally, this value depends
             * on the needs of the codec used for the data.
             *
             * A value of 0 indicates that no caching is necessary.
             *
             * The default is 0.
             */
            uint64_t min_cache() const { return min_cache_; }
            /// \brief Set the minimum cache size.
            void min_cache(uint64_t min_cache) { min_cache_ = min_cache; }

            /** \brief Get the maximum cache size.
             *
             * This value gives the maximum cache size necessary to satisfy
             * reference block requirements. This value heavily depends on the
             * needs on the codec used for data. For example, a codec that may
             * reference up to 10 blocks to decode the current block would
             * require a value of 10 here.
             *
             * A value of 0 indicates that no caching is necessary.
             *
             * The default is 0.
             */
            uint64_t max_cache() const { return max_cache_; }
            /// \brief Set the maximum cache size.
            void max_cache(uint64_t max_cache) { max_cache_ = max_cache; }

            /** \brief Get the default duration of blocks in the track.
             *
             * This value, specified in nanoseconds, is used to give the length
             * of each block in the track if individual timecodes are not used.
             * If specified, it must not be zero.
             */
            uint64_t default_duration() const { return default_dur_; }
            /** \brief Set the default duration of blocks in the track.
             *
             * Set to 0 to disable. No element will be written.
             */
            void default_duration(uint64_t default_duration)
                { default_dur_ = default_duration; }

            /** \brief Get the track's timecode scale.
             *
             * This value speeds up or slows down the track with respect to
             * other tracks. 1.0 means play back at the normal speed.
             *
             * It must be greater than zero.
             */
            double timecode_scale() const { return timecode_scale_; }
            /** \brief Set the track's timecode scale.
             *
             * \throw ValueOutOfRange if a value outside the allowable range is
             * set.
             */
            void timecode_scale(double timecode_scale);

            /** \brief Get the maximum BlockAdditions ID.
             *
             * A value of 0 means there are no BlockAdditions for this track.
             *
             * The default is 0.
             */
            uint64_t max_block_add_id() const
                { return max_block_add_id_; }
            /// \brief Set the maximum BlockAdditions ID.
            void max_block_add_id(uint64_t max_id)
                { max_block_add_id_ = max_id; }

            /// \brief Get the track's name.
            std::string name() const { return name_; }
            /// \brief Set the track's name.
            void name(std::string name) { name_ = name; }

            /** \brief Get the track's codec's ID.
             *
             * The codec ID is vital for determining how to understand the
             * track's data.
             */
            std::string codec_id() const { return codec_id_; }
            /** \brief Set the track's codec's ID.
             *
             * \throw ValueOutOfRange if a value outside the allowable range is
             * set.
             */
            void codec_id(std::string id);

            /** \brief Get the codec-private data.
             *
             * Some codecs store data that does not vary by frame in this
             * element.
             */
            std::vector<char> codec_private() const
                { return codec_private_; }
            /// \brief Set the codec-private data.
            void codec_private(std::vector<char> const& data)
                { codec_private_ = data; }

            /** \brief Get the track's codec's name.
             *
             * This name should be human-readable.
             */
            std::string codec_name() const { return codec_name_; }
            /// \brief Set the track's codec's name.
            void codec_name(std::string name) { codec_name_ = name; }

            /** \brief Get the UID of a linked attachment.
             *
             * If the codec uses data stored in an attachment (for example, a
             * binary providing the codec implementation), this UID must
             * specify the relevant attachment.
             *
             * This value, if specified, must not be zero.
             */
            uint64_t attachment_link() const
                { return attachment_link_; }
            /** \brief Set the UID of a linked attachment.
             *
             * Set to 0 to remove a linked attachment. No element will be
             * written.
             */
            void attachment_link(uint64_t uid) { attachment_link_ = uid; }

            /// \brief Check if this track's codec can decode damaged data.
            bool decode_all() const { return decode_all_; }
            /// \brief Set if this track's codec can decode damaged data.
            void decode_all(bool decode_all) { decode_all_ = decode_all; }

            /** \brief Get the UIDs of overlay tracks.
             *
             * When this track has a gap in its data, the first track in the
             * overlay list with available data will be used instead.
             */
            std::vector<uint64_t> overlays() const;
            /// \brief Set the list of overlay track UIDs.
            void overlays(std::vector<uint64_t> const& uids);

            /** \brief Check if this track is virtual.
             *
             * If this track has a TrackOperation, it is virtual and so its
             * data comes from the blocks of its source tracks. If the track is
             * not virtual, it has its own blocks.
             */
            bool is_virtual() const { return operation_.get(); }
            /** \brief Get the operation used to create this track.
             *
             * If this track is virtual, this operation specifies how to
             * combine the blocks of the source tracks.
             *
             * If this value is empty, then the track is not virtual.
             */
            TrackOperationBase::Ptr operation() const { return operation_; }
            /// \brief Set the operation used to create this track.
            void operation(TrackOperationBase::Ptr const& operation)
                { operation_ = operation; }

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief The type of a shared pointer to a TrackEntry.
            typedef boost::shared_ptr<TrackEntry> Ptr;
            /// \brief The type of a shared pointer to a constant TrackEntry.
            typedef boost::shared_ptr<TrackEntry const> ConstPtr;

            /// \brief Equality operator.
            friend bool operator==(TrackEntry const& lhs,
                    TrackEntry const& rhs);

        protected:
            UIntElement number_;
            UIntElement uid_;
            UIntElement type_;
            UIntElement enabled_;
            UIntElement forced_;
            UIntElement lacing_;
            UIntElement min_cache_;
            UIntElement max_cache_;
            UIntElement default_dur_;
            FloatElement timecode_scale_;
            UIntElement max_block_add_id_;
            StringElement name_;
            StringElement codec_id_;
            BinaryElement codec_private_;
            StringElement codec_name_;
            UIntElement attachment_link_;
            UIntElement decode_all_;
            std::vector<UIntElement> overlays_;
            TrackOperationBase::Ptr operation_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /// \brief Resets all child elements to clean values.
            void reset();
            /** \brief Reads the operation child element.
             *
             * \param[in] input The input stream to read from.
             */
            std::streamsize read_operation(std::istream& input);
    }; // class TrackEntry

    bool operator==(TrackEntry const& lhs, TrackEntry const& rhs);
}; // namespace tawara;

/// @}
// group elements

#endif // TAWARA_TRACK_ENTRY_H_

