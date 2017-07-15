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

#if !defined(TAWARA_SIMPLE_BLOCK_H_)
#define TAWARA_SIMPLE_BLOCK_H_

#include <tawara/block_element.h>
#include <tawara/block_impl.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The SimpleBlock element, a simplified version of Blocks.
     *
     * The SimpleBlock element is used to store a block of data without any
     * extra information, such as reference frames or private codec data. The
     * block is stored as a binary element, with the internal format of the
     * binary data specified by the Simple Block format, which this element
     * implementation interprets.
     */
    class TAWARA_EXPORT SimpleBlock : public BlockElement,
        public boost::equality_comparable<SimpleBlock>
    {
        public:
            /** Create a new SimpleBlock element.
             *
             * \param[in] track_number The track number this block belongs to.
             * \param[in] timecode The timecode of this block.
             * \param[in] lacing The type of lacing to use.
             */
            SimpleBlock(uint64_t track_number, int16_t timecode,
                    LacingType lacing=LACING_NONE);

            /// \brief Check if this block is a keyframe or not.
            bool keyframe() const { return keyframe_; }
            /** \brief Set if this block is a keyframe.
             *
             * How this flag is used is codec-dependent.
             *
             * Keyframe blocks are generally of high importance to the codec,
             * as they provide a complete, independent frame of data that does
             * not rely on other frames to be decoded. In some codecs they may
             * allow the codec to completely re-initialise its state, which is
             * useful if previous frames have been corrupted. In the simple
             * codecs, which do not perform inter-frame encoding, keyframes are
             * irrelevant and so every frame has equal status, making this flag
             * irrelevant.
             */
            void keyframe(bool keyframe) { keyframe_ = keyframe; }

            /// \brief Check if this block can be discarded during playback.
            bool discardable() const { return discardable_; }
            /** \brief Set if this block can be discarded during playback.
             *
             * How this flag is used is codec-dependent.
             *
             * Some frames of data can be discarded without affecting the
             * decoding of subsequent frames. In simple codecs, which do not
             * have a continuously-updating state, every frame may be
             * considered discardable. For codecs that need to know which
             * frames matter when dropping frames to keep up with timing
             * requirements, this flag is important.
             */
            void discardable(bool discardable) { discardable_ = discardable; }

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
            virtual void swap(SimpleBlock& other);

            /// \brief Equality operator.
            friend bool operator==(SimpleBlock const& lhs,
                    SimpleBlock const& rhs);

        private:
            bool keyframe_;
            bool discardable_;
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
    }; // class SimpleBlock

    /// \brief Equality operator for SimpleBlock elements.
    bool operator==(SimpleBlock const& lhs, SimpleBlock const& rhs);
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_SIMPLE_BLOCK_H_

