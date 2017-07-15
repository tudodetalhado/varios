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

#if !defined(TAWARA_MEMORY_CLUSTER_H_)
#define TAWARA_MEMORY_CLUSTER_H_

#include <boost/iterator/iterator_facade.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/utility/enable_if.hpp>
#include <tawara/block_element.h>
#include <tawara/cluster.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The in-memory Cluster implementation.
     *
     * This implementation of the Cluster interface stores the entire cluster
     * in memory, using a std::vector to store the Blocks. It provides rapid
     * access to the blocks at the expense of a larger memory footprint and
     * longer loading time when opening the cluster.
     */
    class TAWARA_EXPORT MemoryCluster : public Cluster
    {
        protected:
            /// Block storage type
            typedef std::vector<BlockElement::Ptr> BlockStore;

        public:
            /// \brief Pointer to a memory-based cluster.
            typedef boost::shared_ptr<MemoryCluster> Ptr;

            /** \brief Construct a new cluster.
             *
             * \param[in] timecode The timecode of the cluster, in the units
             * specified by TimecodeScale.
             */
            MemoryCluster(uint64_t timecode=0);

            //////////////////////////////////////////////////////////////////
            // Iterator types
            //////////////////////////////////////////////////////////////////

            template <typename BlockType, typename IterType>
            class TAWARA_EXPORT IteratorBase
                : public boost::iterator_facade<
                    IteratorBase<BlockType, IterType>,
                    BlockType, boost::bidirectional_traversal_tag>
            {
                private:
                    struct enabler {};

                public:
                    /// \brief Base constructor.
                    IteratorBase()
                    {
                    }

                    /** \brief Constructor.
                     *
                     * \param[in] iter The storage iterator to wrap.
                     */
                    IteratorBase(IterType iter)
                        : iter_(iter)
                    {
                    }

                    /** \brief Templated base constructor.
                     *
                     * Used to provide interoperability with compatible
                     * iterators.
                     */
                    template <typename OtherType, typename OtherIterType>
                    IteratorBase(IteratorBase<OtherType, OtherIterType> const& other)
                        : iter_(other.iter_)
                    {
                    }

                protected:
                    // Necessary for Boost::iterator implementation.
                    friend class boost::iterator_core_access;

                    // Integrate with owning container
                    friend class MemoryCluster;

                    IterType iter_;

                    /// \brief Increment the Iterator to the next block.
                    void increment()
                    {
                        ++iter_;
                    }

                    /// \brief Decrement the Iterator to the previous block.
                    void decrement()
                    {
                        --iter_;
                    }

                    /** \brief Test for equality with another Iterator.
                     *
                     * \param[in] other The other iterator.
                     */
                    template <typename OtherType, typename OtherIterType>
                    bool equal(
                            IteratorBase<OtherType, OtherIterType> const& other) const
                    {
                        return iter_ == other.iter_;
                    }

                    /** \brief Dereference the iterator to get the Block
                     * pointer.
                     */
                    BlockType& dereference() const
                    {
                        return *iter_;
                    }
            }; // class IteratorBase

            /** \brief Block iterator interface.
             *
             * This interface provides access to the blocks in the cluster.
             */
            typedef IteratorBase<BlockElement::Ptr, BlockStore::iterator> Iterator;
            /** \brief Block const iterator interface.
             *
             * This interface provides access to the blocks in the cluster.
             * The access is const, preventing modification of the blocks.
             */
            typedef IteratorBase<Block::ConstPtr, BlockStore::const_iterator>
                ConstIterator;

            //////////////////////////////////////////////////////////////////
            // Iterator access
            //////////////////////////////////////////////////////////////////

            /** \brief Access the start of the blocks.
             *
             * Gets an iterator pointing to the first block in the cluster.
             */
            Iterator begin();
            /** \brief Access the start of the blocks.
             *
             * Gets an iterator pointing to the first block in the cluster.
             */
            ConstIterator begin() const;
            /** \brief Access the end of the blocks.
             *
             * Gets an iterator pointing beyond the last block in the cluster.
             */
            Iterator end();
            /** \brief Access the end of the blocks.
             *
             * Gets an iterator pointing beyond the last block in the cluster.
             */
            ConstIterator end() const;


            //////////////////////////////////////////////////////////////////
            // Cluster interface
            //////////////////////////////////////////////////////////////////

            /// \brief Check if there are no blocks.
            virtual bool empty() const { return blocks_.empty(); }
            /// \brief Get the number of blocks.
            virtual size_type count() const { return blocks_.size(); }
            /// \brief Remove all blocks.
            virtual void clear() { blocks_.clear(); }

            /** \brief Erase the block at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(Iterator position)
                { blocks_.erase(position.iter_); }
            /** \brief Erase a range of blocks.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(Iterator first, Iterator last)
                { blocks_.erase(first.iter_, last.iter_); }

            /** \brief Add a block to this cluster.
             *
             * The cluster must be in the writable state. This means that
             * write() has been called and finalise() has not been called.
             */
            virtual void push_back(value_type const& value)
                { blocks_.push_back(value); }

            /// \brief Finalise writing of the cluster.
            std::streamsize finalise(std::ostream& output);

        protected:
            /// Block storage
            BlockStore blocks_;

            /// \brief Get the size of the blocks in this cluster.
            std::streamsize blocks_size() const;

            /// \brief Read the blocks in this cluster from the output stream.
            std::streamsize read_blocks(std::istream& input,
                    std::streamsize size);
    }; // class MemoryCluster
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_MEMORY_CLUSTER_H_

