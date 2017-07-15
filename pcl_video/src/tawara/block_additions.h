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

#if !defined(TAWARA_BLOCK_ADDITIONS_H_)
#define TAWARA_BLOCK_ADDITIONS_H_

#include <boost/operators.hpp>
#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <tawara/master_element.h>
#include <vector>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief This element is used to specify reference blocks.
     *
     * When decoding a block requires data from other blocks, their IDs are
     * listed using this element. It also contains private codec data that can
     * be used in combination with the other blocks and the owning block.
     */
    class TAWARA_EXPORT BlockAdditions : public MasterElement,
        public boost::equality_comparable<BlockAdditions>
    {
        public:
            /// \brief The type of a single block addition of data.
            typedef std::pair<uint64_t, std::vector<char> > Addition;
            /// \brief A pointer to an addition.
            typedef boost::shared_ptr<Addition> AdditionPtr;
            /// \brief The value type of this container.
            typedef std::vector<AdditionPtr>::value_type value_type;
            /// \brief The size type of this container.
            typedef std::vector<AdditionPtr>::size_type size_type;
            /// \brief The reference type.
            typedef std::vector<AdditionPtr>::reference reference;
            /// \brief The constant reference type.
            typedef std::vector<AdditionPtr>::const_reference const_reference;
            /// \brief The random access iterator type.
            typedef std::vector<AdditionPtr>::iterator iterator;
            /// \brief The constant random access iterator type.
            typedef std::vector<AdditionPtr>::const_iterator const_iterator;
            /// \brief The reversed random access iterator type.
            typedef std::vector<AdditionPtr>::reverse_iterator reverse_iterator;
            /// \brief The constant reversed random access iterator type.
            typedef std::vector<AdditionPtr>::const_reverse_iterator
                const_reverse_iterator;

            /// \brief Constructor.
            BlockAdditions();

            /** \brief Get the addition at the given position, with bounds
             * checking.
             */
            value_type& at(size_type pos)
                { return additions_.at(pos); }
            /** \brief Get the addition at the given position, with bounds
             * checking.
             */
            value_type const& at(size_type pos) const
                { return additions_.at(pos); }

            /** \brief Get a reference to an addition. No bounds checking is
             * performed.
             */
            value_type& operator[](size_type pos)
                { return additions_[pos]; }
            /** \brief Get a reference to an addition. No bounds checking is
             * performed.
             */
            value_type const& operator[](size_type pos) const
                { return additions_[pos]; }

            /// \brief Get an iterator to the first addition.
            iterator begin() { return additions_.begin(); }
            /// \brief Get an iterator to the first addition.
            const_iterator begin() const { return additions_.begin(); }
            /// \brief Get an iterator to the position past the last addition.
            iterator end() { return additions_.end(); }
            /// \brief Get an iterator to the position past the last addition.
            const_iterator end() const { return additions_.end(); }
            /// \brief Get a reverse iterator to the last addition.
            reverse_iterator rbegin() { return additions_.rbegin(); }
            /// \brief Get a reverse iterator to the last addition.
            const_reverse_iterator rbegin() const
                { return additions_.rbegin(); }
            /** \brief Get a reverse iterator to the position before the first
             * addition.
             */
            reverse_iterator rend() { return additions_.rend(); }
            /** \brief Get a reverse iterator to the position before the first
             * addition.
             */
            const_reverse_iterator rend() const { return additions_.rend(); }

            /// \brief Check if there are no additions.
            bool empty() const { return additions_.empty(); }
            /// \brief Get the number of additions.
            size_type count() const { return additions_.size(); }
            /// \brief Get the maximum number of additions.
            size_type max_count() const { return additions_.max_size(); }

            /// \brief Remove all additions.
            void clear() { additions_.clear(); }

            /// \brief Erase the addition at the specified iterator.
            void erase(iterator position)
                { additions_.erase(position); }
            /// \brief Erase a range of additions.
            void erase(iterator first, iterator last)
                { additions_.erase(first, last); }

            /// \brief Add an addition to this block.
            void push_back(value_type const& value);

            /// \brief Resizes the additions storage.
            void resize(size_type count)
                { additions_.resize(count); }

            /// \brief Swaps the contents of this BlockAdditions with another.
            void swap(BlockAdditions& other)
                { additions_.swap(other.additions_); }

            /// \brief Equality operator.
            friend bool operator==(BlockAdditions const& lhs,
                    BlockAdditions const& rhs);

        protected:
            std::vector<AdditionPtr> additions_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body writing.
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /// \brief Loading BlockMore elements
            std::streamsize read_addition(std::istream& input,
                    std::streamsize size);
    }; // class BlockAdditions

    /// \brief Equality operator for BlockAdditions elements.
    bool operator==(BlockAdditions const& lhs, BlockAdditions const& rhs);
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_BLOCK_ADDITIONS_H_

