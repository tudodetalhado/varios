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

#if !defined(TAWARA_BLOCK_H_)
#define TAWARA_BLOCK_H_

#include <boost/operators.hpp>
#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <tawara/win_dll.h>
#include <vector>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief Block interface.
     *
     * The Block interface defines the functionality of a Tawara block. A block
     * is the storage for one (or sometimes more than one, if lacing is used)
     * frame of data.
     */
    class TAWARA_EXPORT Block :
        public boost::equality_comparable<Block>
    {
        public:
            /// \brief Pointer to a block.
            typedef boost::shared_ptr<Block> Ptr;
            /// \brief Constant pointer to a block.
            typedef boost::shared_ptr<Block const> ConstPtr;

            /// \brief Lacing types.
            enum LacingType
            {
                /// No lacing
                LACING_NONE,
                /// EBML-style lacing
                LACING_EBML,
                /// Fixed-size lacing
                LACING_FIXED
            };
            /// \brief The type of a single frame of data.
            typedef std::vector<char> Frame;
            /// \brief A pointer to a frame of data.
            typedef boost::shared_ptr<Frame> FramePtr;
            /// \brief The value type of this container.
            typedef std::vector<FramePtr>::value_type value_type;
            /// \brief The size type of this container.
            typedef std::vector<FramePtr>::size_type size_type;
            /// \brief The reference type.
            typedef std::vector<FramePtr>::reference reference;
            /// \brief The constant reference type.
            typedef std::vector<FramePtr>::const_reference const_reference;
            /// \brief The random access iterator type.
            typedef std::vector<FramePtr>::iterator iterator;
            /// \brief The constant random access iterator type.
            typedef std::vector<FramePtr>::const_iterator const_iterator;
            /// \brief The reversed random access iterator type.
            typedef std::vector<FramePtr>::reverse_iterator reverse_iterator;
            /// \brief The constant reversed random access iterator type.
            typedef std::vector<FramePtr>::const_reverse_iterator
                const_reverse_iterator;

            /** \brief Constructor.
             *
             * \param[in] track_number The track number this block belongs to.
             * \param[in] timecode The timecode of this block.
             * \param[in] lacing The type of lacing to use.
             */
            Block(uint64_t track_number, int16_t timecode,
                    LacingType lacing=LACING_NONE)
            {}

            /// \brief Desctructor.
            virtual ~Block() = 0;

            /** \brief The block's track number.
             *
             * This property specifies the track that this block belongs to.
             * The data stored in the block should be interpreted by the codec
             * for its track.
             */
            virtual uint64_t track_number() const = 0;
            /// \brief Set the block's track number.
            virtual void track_number(uint64_t track_number) = 0;

            /** \brief The timecode of this block.
             *
             * Each block has a timecode relative to its containing cluster. It
             * is measured in the units specified by the containing segment's
             * TimecodeScale, and is a 16-bit signed integer.
             */
            virtual int16_t timecode() const = 0;
            /// \brief Set the block's timecode.
            virtual void timecode(int16_t timecode) = 0;

            /** \brief If this block is invisible.
             *
             * Invisible blocks should be decoded by the codec (thus updating
             * codec state) but not used for playback.
             */
            virtual bool invisible() const = 0;
            /// \brief Set if this block is invisible.
            virtual void invisible(bool invisible) = 0;

            /** \brief Get the lacing type in use.
             *
             * The data in a block is typically a single frame, but sometimes
             * multiple frames may be stored. This is called "lacing," and is
             * usually used to reduce overhead when the size of the data itself
             * is small. However, lacing also reduces seekability of the file,
             * so laces should usually be kept small. A common number is up to
             * three frames in a lace.
             */
            virtual LacingType lacing() const = 0;
            /// \brief Set the lacing type in use.
            virtual void lacing(LacingType lacing) = 0;

            /** \brief Get the frame at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified frame's data.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type& at(size_type pos) = 0;
            /** \brief Get the frame at the given position, with bounds
             * checking.
             *
             * \return A reference to the specified frame's data.
             * \throw std::out_of_range if the position is invalid.
             */
            virtual value_type const& at(size_type pos) const = 0;

            /** \brief Get a reference to a frame. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a frame stored in this
             * block.
             */
            virtual value_type& operator[](size_type pos) = 0;
            /** \brief Get a reference to a frame. No bounds checking is
             * performed.
             *
             * \return A reference to the binary data of a frame stored in this
             * block.
             */
            virtual value_type const& operator[](size_type pos) const = 0;

            /// \brief Get an iterator to the first frame.
            virtual iterator begin() = 0;
            /// \brief Get an iterator to the first frame.
            virtual const_iterator begin() const = 0;
            /// \brief Get an iterator to the position past the last frame.
            virtual iterator end() = 0;
            /// \brief Get an iterator to the position past the last frame.
            virtual const_iterator end() const = 0;
            /// \brief Get a reverse iterator to the last frame.
            virtual reverse_iterator rbegin() = 0;
            /// \brief Get a reverse iterator to the last frame.
            virtual const_reverse_iterator rbegin() const = 0;
            /** \brief Get a reverse iterator to the position before the first
             * frame.
             */
            virtual reverse_iterator rend() = 0;
            /** \brief Get a reverse iterator to the position before the first
             * frame.
             */
            virtual const_reverse_iterator rend() const = 0;

            /** \brief Check if there are no frames.
             *
             * Empty blocks cannot be written. If this returns true, an error
             * will occur when write() is called.
             */
            virtual bool empty() const = 0;
            /// \brief Get the number of frames.
            virtual size_type count() const = 0;
            /** \brief Get the maximum number of frames.
             *
             * If lacing is not enabled, this will always return 1.
             *
             * If lacing is enabled, this will be the maximum number of frames
             * that can be stored in a lace within a single block.
             */
            virtual size_type max_count() const = 0;

            /// \brief Remove all frames.
            virtual void clear() = 0;

            /** \brief Erase the frame at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(iterator position) = 0;
            /** \brief Erase a range of frames.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(iterator first, iterator last) = 0;

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
            virtual void push_back(value_type const& value) = 0;

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
            virtual void resize(size_type count) = 0;

            /** \brief Swaps the contents of this block with another.
             *
             * \param[in] other The other block to swap with.
             */
            virtual void swap(Block& other) {}
    }; // class Block
}; // namespace tawara

/// @}
// group interfaces

#endif // TAWARA_BLOCK_H_

