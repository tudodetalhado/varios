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

#if !defined(TAWARA_BLOCK_GROUP_H_)
#define TAWARA_BLOCK_GROUP_H_

#include <boost/operators.hpp>
#include <tawara/binary_element.h>
#include <tawara/block_element.h>
#include <tawara/block_additions.h>
#include <tawara/block_impl.h>
#include <tawara/master_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The BlockGroup is the standard block.
     *
     * The BlockGroup contains a Block itself as well as meta-data and extra
     * data for that block, such as reference blocks and private codec data for
     * the block.
     *
     * Despite its name, the BlockGroup only stores a single Block. It
     * implements the Block interface, and so can be treated as a Block,
     * similar to the SimpleBlock element.
     */
    class BlockGroup : public BlockElement,
        public boost::equality_comparable<BlockGroup>
    {
        public:
            /** \brief Create a new BlockGroup.
             *
             * \param[in] track_number The track number this block belongs to.
             * \param[in] timecode The timecode of this block.
             * \param[in] lacing The type of lacing to use.
             * \param[in] duration The duration of the block.
             * \param[in] ref_priority The reference priority for this block.
             */
            BlockGroup(uint64_t track_number, int16_t timecode,
                    LacingType lacing=LACING_NONE, uint64_t duration=0,
                    uint64_t ref_priority=0);

            /// \brief Access the block additions property.
            BlockAdditions& additions() { return additions_; }

            /** \brief Get the block's duration.
             *
             * The duration of this block may come from one of three sources:
             * - If the block's track has no default duration and this value is
             *   0, the duration is the difference between this block's time
             *   code and the timecode of the next block for the track.
             * - If the block's track has a default duration and this value is
             *   0, the track's default duration is used.
             * - If the block's track has a default duration and this value is
             *   not zero, this value is used.
             *
             * The units are the TimecodeScale of the segment.
             */
            uint64_t duration() const { return duration_; }
            /// \brief Set the block's duration.
            void duration(uint64_t duration) { duration_ = duration; }

            /** \brief Get the block's reference priority.
             *
             * If this block is referenced and cached, then only another block
             * of higher priority should replace it in the cache. A value of 0
             * means the block is not referenced.
             */
            uint64_t ref_priority() const { return ref_priority_; }
            /// \brief Set the block's reference priority.
            void ref_priority(uint64_t ref_priority)
                { ref_priority_ = ref_priority; }

            /** \brief Get the reference block timecode array.
             *
             * These timecodes, specified as relative to the cluster's
             * timecode, locate the reference blocks used to understand this
             * block.
             */
            std::vector<int16_t>& ref_blocks() { return ref_blocks_; }

            /** \brief Get the codec state for this block.
             *
             * Sometimes a block may reset the codec state to a new value. This
             * property stores the new state as a binary blob. Understanding
             * the data stored in the blob is codec-specific.
             */
            std::vector<char> codec_state() const
                { return codec_state_; }
            /// \brief Set the codec state for this block.
            void codec_state(std::vector<char> const& codec_state)
                { codec_state_ = codec_state; }

            ///////////////////
            // Block interface
            ///////////////////

            /** \brief The block's track number.
             *
             * This property specifies the track that this block belongs to.
             * The data stored in the block should be interpreted by the codec
             * for its track.
             */
            virtual uint64_t track_number() const
                { return block_.track_number(); }
            /// \brief Set the block's track number.
            virtual void track_number(uint64_t track_number)
                { block_.track_number(track_number); }

            /** \brief The timecode of this block.
             *
             * Each block has a timecode relative to its containing cluster. It
             * is measured in the units specified by the containing segment's
             * TimecodeScale, and is a 16-bit signed integer.
             */
            virtual int16_t timecode() const { return block_.timecode(); }
            /// \brief Set the block's timecode.
            virtual void timecode(int16_t timecode)
                { block_.timecode(timecode); }

            /** \brief If this block is invisible.
             *
             * Invisible blocks should be decoded by the codec (thus updating
             * codec state) but not used for playback.
             */
            virtual bool invisible() const { return block_.invisible(); }
            /// \brief Set if this block is invisible.
            virtual void invisible(bool invisible)
                { block_.invisible(invisible); }

            /** \brief Get the lacing type in use.
             *
             * The data in a block is typically a single frame, but sometimes
             * multiple frames may be stored. This is called "lacing," and is
             * usually used to reduce overhead when the size of the data itself
             * is small. However, lacing also reduces seekability of the file,
             * so laces should usually be kept small. A common number is up to
             * three frames in a lace.
             */
            virtual LacingType lacing() const { return block_.lacing(); }
            /// \brief Set the lacing type in use.
            virtual void lacing(LacingType lacing)
                { block_.lacing(lacing); }

            /** \brief Get the frame at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified frame's data.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type& at(size_type pos) { return block_.at(pos); }
            /** \brief Get the frame at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified frame's data.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type const& at(size_type pos) const
                { return block_.at(pos); }

            /** \brief Get a reference to a frame. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a frame stored in this
             * block.
             */
            virtual value_type& operator[](size_type pos)
                { return block_[pos]; }
            /** \brief Get a reference to a frame. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a frame stored in this
             * block.
             */
            virtual value_type const& operator[](size_type pos) const
                { return block_[pos]; }

            /// \brief Get an iterator to the first frame.
            virtual iterator begin() { return block_.begin(); }
            /// \brief Get an iterator to the first frame.
            virtual const_iterator begin() const { return block_.begin(); }
            /// \brief Get an iterator to the position past the last frame.
            virtual iterator end() { return block_.end(); }
            /// \brief Get an iterator to the position past the last frame.
            virtual const_iterator end() const { return block_.end(); }
            /// \brief Get a reverse iterator to the last frame.
            virtual reverse_iterator rbegin() { return block_.rbegin(); }
            /// \brief Get a reverse iterator to the last frame.
            virtual const_reverse_iterator rbegin() const
                { return block_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * frame.
             */
            virtual reverse_iterator rend() { return block_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * frame.
             */
            virtual const_reverse_iterator rend() const
                { return block_.rend(); }

            /** \brief Check if there are no frames.
             *
             * Empty blocks cannot be written. If this returns true, an error
             * will occur when write() is called.
             */
            virtual bool empty() const { return block_.empty(); }
            /// \brief Get the number of frames.
            virtual size_type count() const { return block_.count(); }
            /** \brief Get the maximum number of frames.
             *
             * If lacing is not enabled, this will always return 1.
             *
             * If lacing is enabled, this will be the maximum number of frames
             * that can be stored in a lace within a single block.
             */
            virtual size_type max_count() const { return block_.max_count(); }

            /// \brief Remove all frames.
            virtual void clear() { block_.clear(); }

            /** \brief Erase the frame at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(iterator position) { block_.erase(position); }
            /** \brief Erase a range of frames.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(iterator first, iterator last)
                { block_.erase(first, last); }

            /** \brief Add a frame to this block.
             *
             * When lacing is enabled, this will append an additional frame to
             * the block to be stored.
             *
             * When lacing is not enabled, the value of frame_count() must be
             * zero or an error will occur.
             *
             * \throw MaxLaceSizeExceeded if the new size is incompatible with
             * the lacing type.
             * \throw EmptyFrame if the frame data is empty.
             */
            virtual void push_back(value_type const& value)
                { block_.push_back(value); }

            /** \brief Resizes the frames storage.
             *
             * When lacing is not enabled, the new size must be 1 or an error
             * will occur.
             *
             * If the current size is less than the new size, additional empty
             * frames will be added. These should be filled with data before
             * calling write() or an error will occur.
             *
             * If the current size is greater than the new size, frames past
             * the new end will be dropped.
             *
             * \throw MaxLaceSizeExceeded if the new size is incompatible with
             * the lacing type.
             */
            virtual void resize(size_type count) { block_.resize(count); }

            /** \brief Swaps the contents of this block with another.
             *
             * \param[in] other The other block to swap with.
             */
            virtual void swap(BlockGroup& other);

            /// \brief Equality operator.
            friend bool operator==(BlockGroup const& lhs,
                    BlockGroup const& rhs);

        protected:
            BlockAdditions additions_;
            UIntElement duration_;
            UIntElement ref_priority_;
            std::vector<int16_t> ref_blocks_;
            BinaryElement codec_state_;
            BlockImpl block_;

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

            /// \brief Reset all values to their defaults.
            void reset();
    }; // class BlockGroup

    /// \brief Equality operator for BlockGroup elements.
    bool operator==(BlockGroup const& lhs, BlockGroup const& rhs);
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_BLOCK_GROUP_H_

