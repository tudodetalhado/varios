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

#if !defined(TAWARA_FILE_CLUSTER_H_)
#define TAWARA_FILE_CLUSTER_H_

#include <functional>
#include <tawara/block_element.h>
#include <tawara/block_group.h>
#include <tawara/cluster.h>
#include <tawara/simple_block.h>
#include <tawara/win_dll.h>


/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The in-file Cluster implementation.
     *
     * This implementation of the cluster interface stores only a minimal amount
     * of information in memory. When reading, each block is left in the file
     * until needed. When writing, blocks are written to the file immediately as
     * they become available. It provides a lower memory footprint than the
     * MemoryCluster implementation, at the expense of slower block retrieval
     * and addition.
     */
    class TAWARA_EXPORT FileCluster : public Cluster
    {
        public:
            /// \brief Pointer to a file-based cluster.
            typedef boost::shared_ptr<FileCluster> Ptr;

            /** \brief Construct a new cluster.
             *
             * \param[in] timecode The timecode in the cluster, in the units
             * specified by TimecodeScale.
             */
            FileCluster(uint64_t timecode=0);

            //////////////////////////////////////////////////////////////////
            // Iterator types
            //////////////////////////////////////////////////////////////////

            template <typename BlockType>
            class TAWARA_EXPORT IteratorBase
                : public boost::iterator_facade 
				<IteratorBase<BlockType>, BlockType, 
                    boost::forward_traversal_tag>
            {
                private:
                    struct enabler {};

                public:
                    /** \brief Base constructor.
                     *
                     * Constructs an empty iterator.
                     */
                    IteratorBase()
                        : cluster_(0)
                    {
                    }

                    /** \brief Base constructor.
                     *
                     * \param[in] cluster The cluster containing the blocks.
                     * \param[in] stream The stream to read blocks from.
                     * \param[in] pos The position in the file of the first
                     * block to read.
                     */
                    IteratorBase(FileCluster const* cluster,
                            std::istream& stream, std::streampos pos)
                        : cluster_(cluster), stream_(&stream)
                    {
                        // Open the block at the provided position
                        load_block(pos);
                    }

                    /** \brief Templated base constructor.
                     *
                     * Used to provide interoperability with compatible
                     * iterators.
                     */
                    template <typename OtherType>
                    IteratorBase(IteratorBase<OtherType> const& other)
                        : cluster_(other.cluster_), stream_(other.stream_),
                        block_(other.block_)
                    {
                    }

                protected:
                    // Necessary for Boost::iterator implementation.
                    friend class boost::iterator_core_access;

                    // Integrate with owning container.
                    friend class FileCluster;

                    FileCluster const* cluster_;
                    std::istream* stream_;
                    boost::shared_ptr<BlockType> block_;

                    void load_block(std::streampos pos)
                    {
                        if (pos == cluster_->blocks_end_pos_)
                        {
                            // End of the blocks
                            block_.reset();
                        }
                        else
                        {
                            // Save the current read position
                            std::streampos cur_read(stream_->tellg());
                            // Jump to the expected block location
                            stream_->seekg(pos);
                            // Read the block
                            ids::ReadResult id_res = ids::read(*stream_);
                            if (id_res.first == ids::SimpleBlock)
                            {
                                BlockElement::Ptr new_block(new SimpleBlock(0, 0));
                                new_block->read(*stream_);
                                // TODO Ick. Needs fixing.
                                boost::shared_ptr<BlockType> new_const_block(new_block);
                                block_.swap(new_const_block);
                            }
                            else if (id_res.first == ids::BlockGroup)
                            {
                                BlockElement::Ptr new_block(new BlockGroup(0, 0));
                                new_block->read(*stream_);
                                // TODO Ick. Needs fixing.
                                boost::shared_ptr<BlockType> new_const_block(new_block);
                                block_.swap(new_const_block);
                            }
                            else
                            {
                                throw InvalidChildID() << err_id(id_res.first) <<
                                    err_par_id(cluster_->id_) <<
                                    // The cast here makes Apple's LLVM compiler happy
                                    err_pos(static_cast<std::streamsize>(stream_->tellg()) -
                                            id_res.second);
                            }
                            // Return to the original read position
                            stream_->seekg(cur_read);
                        }
                    }

                    /// \brief Increment the iterator to the next block.
                    void increment()
                    {
                        // Don't increment if at the end
                        if (block_)
                        {
                            // Load the block after this one
                            load_block(block_->offset() + block_->size());
                        }
                    }

                    /** \brief Test for equality with another iterator.
                     *
                     * \param[in] other The other iterator.
                     */
                    template <typename OtherType>
                    bool equal(IteratorBase<OtherType> const& other) const
                    {
                        if (block_)
                        {
                            // This iterator is not at the end
                            if (other.block_)
                            {
                                // Neither is the other
                                return block_->offset() ==
                                    other.block_->offset();
                            }
                            return false;
                        }
                        else
                        {
                            // This iterator is at the end
                            if (!other.block_)
                            {
                                // So is the other
                                return true;
                            }
                            return false;
                        }
                    }

                    /** \brief Dereference the iterator to get a pointer to the
                     * block.
                     */
                    BlockType& dereference() const
                    {
                        return *block_;
                    }
            }; // class IteratorBase

            /** \brief File-based cluster iterator interface.
             *
             * This interface provides access to the blocks in the cluster.
             */
            typedef IteratorBase<BlockElement> Iterator;

            //////////////////////////////////////////////////////////////////
            // Iterator access
            //////////////////////////////////////////////////////////////////

            /** \brief Access the start of the blocks.
             *
             * Gets an iterator pointing to the first block in the cluster.
             */
            Iterator begin();
            /** \brief Access the end of the blocks.
             *
             * Gets an iterator pointing beyond the last block in the cluster.
             */
            Iterator end();


            //////////////////////////////////////////////////////////////////
            // Cluster interface
            //////////////////////////////////////////////////////////////////

            /// \brief Check if there are no blocks.
            virtual bool empty() const;
            /// \brief Get the number of blocks.
            virtual size_type count() const;
            /** \brief Remove all blocks.
             *
             * This will not erase the blocks from the file. It will instead
             * overwrite them with a void element, hiding them.
             */
            virtual void clear();

            /** \brief Erase the block at the specified iterator.
             *
             * This will not erase the block from the file. It will instead
             * overwrite it with a void element, hiding the data.
             *
             * \param[in] position The position to erase at.
             */
            virtual void erase(Iterator position);
            /** \brief Erase a range of blocks.
             *
             * This will not erase the blocks from the file. It will instead
             * overwrite them with a void element, hiding them.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            virtual void erase(Iterator first, Iterator last);

            /** \brief Add a block to this cluster.
             *
             * The cluster must be in the writable state. This means that
             * write() has been called and finalise() has not been called.
             */
            virtual void push_back(value_type const& value);

            /// \brief Element writing.
            std::streamsize write(std::ostream& output);

            /// \brief Finalise writing of the cluster.
            std::streamsize finalise(std::ostream& output);

        protected:
            std::ostream* ostream_;
            std::istream* istream_;
            std::streampos blocks_start_pos_;
            std::streampos blocks_end_pos_;

            /// \brief Get the size of the blocks in this cluster.
            std::streamsize blocks_size() const;

            /// \brief Read the blocks in this cluster from the output stream.
            std::streamsize read_blocks(std::istream& input,
                    std::streamsize size);
    }; // class FileCluster
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_FILE_CLUSTER_H_

