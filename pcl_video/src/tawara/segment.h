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

#if !defined(TAWARA_SEGMENT_H_)
#define TAWARA_SEGMENT_H_

#include <map>
#include <tawara/master_element.h>
#include <tawara/file_cluster.h>
#include <tawara/memory_cluster.h>
#include <tawara/metaseek.h>
#include <tawara/segment_info.h>
#include <tawara/win_dll.h>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief The Segment element.
     *
     * A segment makes up the body of a Tawara document. It is the only top-level
     * element, excluding the EBMLHeader. All other Tawara elements are contained
     * within it. Usually, a Tawara document contains a single segment.
     *
     * The segment contains all the document's blocks (stored in clusters), and
     * all relevant meta-data, such as document information, tags, attachments
     * and cue points.
     *
     * Because the body of a segment contains data that arrives at differing
     * times, segments cannot be written in a single operation. Instead, a
     * segment is opened for writing by calling write(), which provides it with
     * its target output byte stream.  This prepares the byte stream for the
     * segment to be written. Once writing is complete, the segment is
     * finalised by calling finalise(). Finalising a segment completes it,
     * writing such things as the segment's final size into the output stream.
     *
     * Likewise, segments are not read in a single operation. Trying to do so
     * for any reasonably-sized file would require a significant quantity of
     * memory and be inefficient. Instead, the segment is opened for reading by
     * calling the read() method, which prepares the segment for reading by the
     * meta-seek element (if present) and filling in the index table. The child
     * elements are then read directly from the file as needed. The segment
     * does not need to be closed once reading is complete.
     */
    class TAWARA_EXPORT Segment : public MasterElement
    {
        public:
            /// \brief Pointer to a segment.
            typedef boost::shared_ptr<Segment> Ptr;

            /** \brief Create a new segment element.
             *
             * \param[in] pad_size The size of the padding to place at the
             * start of the segment when opening it for writing.
             */
            Segment(std::streamsize pad_size=4096);

            //////////////////////////////////////////////////////////////////
            // Iterator types
            //////////////////////////////////////////////////////////////////

            // All clusters in the segment.
            template <typename ClusterType>
            class TAWARA_EXPORT ClusterIteratorBase
                : public boost::iterator_facade<
                    ClusterIteratorBase<ClusterType>,
                    ClusterType, boost::forward_traversal_tag>
            {
                private:
                    struct enabler {};

                public:
                    /** \brief Constructor.
                     *
                     * \param[in] segment The segment containing the clusters.
                     * \param[in] stream The stream to read clusters from.
                     */
                    ClusterIteratorBase(Segment const* segment,
                            std::istream& stream)
                        : segment_(segment), stream_(stream)
                    {
                        // Read the first cluster from the stream to populate
                        // the cluster pointer.
                        std::streampos current_pos(stream_.tellg());

                        SeekHead::const_iterator first_cluster(
                                segment->index.find(ids::Cluster));
                        if (first_cluster != segment->index.end())
                        {
                            stream_.seekg(segment_->to_stream_offset(
                                        first_cluster->second));
                            open_cluster();
                        }
                        // Otherwise there are no clusters so this iterator
                        // should be left at the end (i.e. cluster_ is empty).

                        // Restore the read position
                        stream_.seekg(current_pos);
                    }

                    /** \brief Templated base constructor.
                     *
                     * Used to provide interoperability with compatible
                     * iterators.
                     */
                    template <typename OtherType>
                    ClusterIteratorBase(
                            ClusterIteratorBase<OtherType> const& other)
                        : segment_(other.segment_), stream_(other.stream_),
                        cluster_(other.cluster_)
                    {
                    }

                protected:
                    // Necessary for Boost::iterator implementation.
                    friend class boost::iterator_core_access;

                    // Integrate with owning container.
                    friend class Segment;

                    Segment const* segment_;
                    std::istream& stream_;
                    boost::shared_ptr<ClusterType> cluster_;

                    void open_cluster()
                    {
                        tawara::ids::ReadResult id(tawara::ids::read(stream_));
                        if (id.first != tawara::ids::Cluster)
                        {
                            throw InvalidChildID() << err_id(id.first) <<
                                err_par_id(segment_->id_) <<
                                err_pos(static_cast<std::streamsize>(
                                        stream_.tellg()) - id.second);
                        }

                        boost::shared_ptr<ClusterType> new_cluster(new ClusterType);
                        new_cluster->read(stream_);

                        cluster_.swap(new_cluster);
                    }

                    /// \brief Increment the iterator to the next cluster.
                    void increment()
                    {
                        // Preserve the current read position.
                        std::streampos current_pos(stream_.tellg());
                        // Jump to the current cluster's position.
                        stream_.seekg(cluster_->offset());
                        // Skip the cluster
                        stream_.seekg(cluster_->size(), std::ios::cur);
                        // Search for the next cluster
                        while(true)
                        {
                            // Check if the end of the segment has been passed
                            if (stream_.tellg() >=
                                    segment_->size_ + segment_->offset_)
                            {
                                // End of the clusters
                                cluster_.reset();
                                break;
                            }
                            try
                            {
                                // Attempt to open a cluster
                                open_cluster();
                            }
                            catch (InvalidChildID())
                            {
                                // The next element was not a cluster; skip it
                                skip_read(stream_, false);
                                continue;
                            }
                            break;
                        }
                        // Restore the read position
                        stream_.seekg(current_pos);
                    }

                    /** \brief Test for equality with another iterator.
                     *
                     * \param[in] other The other iterator.
                     */
                    template <typename OtherType>
                    bool equal(
                            ClusterIteratorBase<OtherType> const& other) const
                    {
                        if (!cluster_ && !other.cluster_)
                        {
                            return true;
                        }
                        else if (!cluster_)
                        {
                            return false;
                        }
                        else if (!other.cluster_)
                        {
                            return false;
                        }
                        else if (cluster_->offset() == other.cluster_->offset())
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }

                    /** \brief Dereference the iterator to get a pointer to the
                     * cluster.
                     */
                    ClusterType& dereference() const
                    {
                        return *cluster_;
                    }
            }; // class IteratorBase

            /** \brief Memory-based cluster iterator interface.
             *
             * This interface provides access to the clusters in the segment,
             * with each cluster read entirely into memory.
             */
            typedef ClusterIteratorBase<MemoryCluster> MemClusterIterator;

            /** \brief File-based cluster iterator interface.
             *
             * This interface provides access to the clusters in the segment,
             * with each cluster loading its blocks on demand.
             */
            typedef ClusterIteratorBase<FileCluster> FileClusterIterator;


            // All blocks in the segment.
            template <typename ClusterItrType, typename BlockItrType>
            class TAWARA_EXPORT BlockIteratorBase
                : public boost::iterator_facade<
                    BlockIteratorBase<ClusterItrType, BlockItrType>,
                    typename BlockItrType::value_type,
                    boost::forward_traversal_tag>
            {
                private:
                    struct enabler {};

                public:
                    /** \brief Constructor.
                     *
                     * \param[in] segment The segment containing the clusters.
                     * \param[in] cluster The cluster to read blocks from.
                     */
                    BlockIteratorBase(Segment* segment,
                            ClusterItrType const& cluster)
                        : segment_(segment), cluster_(cluster)
                    {
                        if (cluster_.cluster_)
                        {
                            // Get the first block in the start cluster.
                            block_ = cluster_->begin();
                            // If the cluster doesn't have any blocks, run through
                            // clusters until one with a block is found.
                            while (block_ == cluster_->end() &&
                                    cluster_.cluster_)
                            {
                                ++cluster_;
                                block_ = cluster_->begin();
                            }
                        }
                    }

                    /** \brief Templated base constructor.
                     *
                     * Used to provide interoperability with compatible
                     * iterators.
                     */
                    template <typename OtherCIType, typename OtherBIType>
                    BlockIteratorBase(
                            BlockIteratorBase<OtherCIType, OtherBIType> const& other)
                        : cluster_(other.cluster_), block_(other.block_)
                    {
                    }

                    /// \brief Access to the cluster for the current block.
                    ClusterItrType cluster() const
                        { return cluster_; }

                protected:
                    // Necessary for Boost::iterator implementation.
                    friend class boost::iterator_core_access;

                    // Integrate with owning container.
                    friend class Segment;

                    Segment* segment_;
                    ClusterItrType cluster_;
                    BlockItrType block_;

                    /// \brief Increment the iterator to the next block.
                    void increment()
                    {
                        ++block_;
                        // If the end of the cluster has been reached, go
                        // through the clusters to find the next with blocks.
                        while (block_ == cluster_->end() &&
                                cluster_.cluster_)
                        {
                            ++cluster_;
                            if (!cluster_.cluster_)
                            {
                                break;
                            }
                            block_ = cluster_->begin();
                        }
                    }

                    /** \brief Test for equality with another Iterator.
                     *
                     * \param[in] other The other iterator.
                     */
                    template <typename OtherCIType, typename OtherBIType>
                    bool equal(
                        BlockIteratorBase<OtherCIType, OtherBIType> const& other) const
                    {
                        if (cluster_ == other.cluster_)
                        {
                            if (!cluster_.cluster_)
                            {
                                return true;
                            }
                            else
                            {
                                return block_ == other.block_;
                            }
                        }
                        else
                        {
                            return false;
                        }
                    }

                    /** \brief Dereference the iterator to get a pointer to the
                     * block.
                     */
                    typename BlockItrType::value_type& dereference() const
                    {
                        return *block_;
                    }
            };

            /** \brief Memory-based block iterator interface.
             *
             * This interface provides access to the blocks in the segment,
             * stored across all the clusters.
             */
            typedef BlockIteratorBase<MemClusterIterator,
                    MemoryCluster::Iterator> MemBlockIterator;

            /** \brief File-based block iterator interface.
             *
             * This interface provides access to the blocks in the segment,
             * stored across all the clusters.
             */
            typedef BlockIteratorBase<FileClusterIterator,
                    FileCluster::Iterator> FileBlockIterator;


            //////////////////////////////////////////////////////////////////
            // Iterator access
            //////////////////////////////////////////////////////////////////

            /** \brief Access the start of the clusters.
             *
             * Gets an iterator pointing to the first cluster in the segment,
             * using the memory-based cluster implementation.
             */
            MemClusterIterator clusters_begin_mem(std::istream& stream);
            /** \brief Access the end of the clusters.
             *
             * Gets an iterator pointing to the last cluster in the segment,
             * using the memory-based cluster implementation.
             */
            MemClusterIterator clusters_end_mem(std::istream& stream);

            /** \brief Access the start of the blocks.
             *
             * Gets an iterator pointing to the first block in the segment,
             * using the memory-based cluster implementation.
             */
            MemBlockIterator blocks_begin_mem(std::istream& stream);
            /** \brief Access the end of the blocks.
             *
             * Gets an iterator pointing to the last block in the segment,
             * using the memory-based cluster implementation.
             */
            MemBlockIterator blocks_end_mem(std::istream& stream);


            /** \brief Access the start of the clusters.
             *
             * Gets an iterator pointing to the first cluster in the segment,
             * using the file-based cluster implementation.
             */
            FileClusterIterator clusters_begin_file(std::istream& stream);
            /** \brief Access the end of the clusters.
             *
             * Gets an iterator pointing to the last cluster in the segment,
             * using the file-based cluster implementation.
             */
            FileClusterIterator clusters_end_file(std::istream& stream);

            /** \brief Access the start of the blocks.
             *
             * Gets an iterator pointing to the first block in the segment,
             * using the file-based cluster implementation.
             */
            FileBlockIterator blocks_begin_file(std::istream& stream);
            /** \brief Access the end of the blocks.
             *
             * Gets an iterator pointing to the last block in the segment,
             * using the file-based cluster implementation.
             */
            FileBlockIterator blocks_end_file(std::istream& stream);


            //////////////////////////////////////////////////////////////////
            // Segment interface
            //////////////////////////////////////////////////////////////////

            /** \brief Get the padding size.
             *
             * When the segment is opened for writing, it places a certain
             * quantity of padding at the start of the segment using a Void
             * element. This padding is over-written during finalisation with
             * the SeekHead and SegmentInfo elements. The length of the padding
             * is controlled by this property. Setting it to a value that is
             * too small for one or both of the SeekHead and SegmentInfo
             * elements will result in those that don't fit being written at
             * the end of the segment. In the case of the SeekHead, this can
             * significantly degrade start-up performance when reading the
             * segment.
             */
            std::streamsize pad_size() const { return pad_size_; }
            /// \brief Set the padding size.
            void pad_size(std::streamsize pad_size) { pad_size_ = pad_size; }

            /// \brief Get the total size of the element.
            std::streamsize size() const;

            /** \brief Finalise writing of the segment.
             *
             * Calculates the total size of the segment and writes it into the
             * segment header, followed by writing in the SeekHead element as
             * the first child of the Segment, thus finalising the segment.
             *
             * The write pointer in stream must be positioned at the first byte
             * after the last byte of the segment before this method is called.
             *
             * \param[in] stream The byte stream to write the segment to.
             * \return The final size, in bytes, of the segment (including the
             * element header).
             * \throw NotWriting if the segment has not yet been opened for
             * writing by calling write().
             */
            std::streamsize finalise(std::iostream& stream);

            /** \brief The segment index.
             *
             * All known level 1 elements are included in this index. It is a
             * mapping from element ID (the key) to position, in bytes, in the
             * segment (the value). The first level 1 element has a position of
             * 0. An ID may occur multiple times.
             *
             * There may be additional level 1 elements that are not mentioned
             * in this index. Typically, all but the first cluster are not
             * found in the index.
             */
            SeekHead index;

            /** \brief The segment information.
             *
             * This property stores all the segment's meta-data, such as the
             * origin timecode of the segment and its timecode scale.
             */
            SegmentInfo info;

            /** \brief Calculate an offset within the segment.
             *
             * This function turns an offset in the output stream into an
             * offset in this segment, which is necessary for the index.
             */
            std::streamsize to_segment_offset(std::streamsize stream_offset) const;

            /** \brief Calculate the offset in a stream of a position in the
             * segment.
             *
             * This function turns an offset within the segment into an
             * absolute position in a byte stream.
             */
            std::streamsize to_stream_offset(std::streamsize seg_offset) const;

        protected:
            /// The size of the padding to place at the start of the file.
            std::streamsize pad_size_;
            /// The size of the segment, as read from the file.
            std::streamsize size_;
            /// If the segment is currently being written.
            bool writing_;

            /** \brief Get the size of the body of this element.
             *
             * This function will not return the actual size of the segment
             * until either the segment is read from a Tawara document or it is
             * finalised in a Tawara document.
             */
            std::streamsize body_size() const
                { return size_; }

            /// \brief Element size writing.
            std::streamsize write_size(std::ostream& output);

            /** \brief Element body writing.
             *
             * This function, which opens up a segment for writing, does not
             * actually write any final content. Instead, it preserves a chunk
             * of space at the beginning of the file using a Void element for
             * the SeekHead and SegmentInfo elements to be written into by
             * finalise().
             *
             * The size of space reserved is controlled by pad_size().
             */
            std::streamsize write_body(std::ostream& output);

            /** \brief Element body loading.
             *
             * This function does not actually read the body of the segment.
             * Instead, it reads some of the level 1 elements and validates
             * that the segment is usable:
             *
             * - Reads the meta-seek element (if present), filling in the
             *   segment's index.
             * - Reads the SegmentInfo element. If not present, raises an
             *   error.
             * - Checks for the existence of the Tracks element. Raises an
             *   error if not present.
             *   Checks for the existence of at least one Cluster element.
             *   Raises an error if not present.
             *
             * When this function returns, the stream's read pointer will be
             * placed at the end of the SegmentInformation element, unless the
             * SeekHead element occurs after it, in which case it will be
             * placed at the end of the SeekHead element.
             *
             * \return The number of bytes read from the stream, excluding
             * bytes read during element presence checks. In other words, this
             * is the number of bytes into the segment that the read pointer
             * has moved.
             * \throw MultipleSeekHeads if the segment has more than one index.
             * \throw NoSegmentInfo if the SegmentInfo element is not found.
             * \throw NoTracks if the Tracks element is not found.
             * \throw NoClusters if at least one Cluster element is not found.
             */
            std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class Segment
}; // namespace tawara

/// @}

#endif // TAWARA_SEGMENT_H_

