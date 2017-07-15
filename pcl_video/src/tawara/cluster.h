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

#if !defined(TAWARA_CLUSTER_H_)
#define TAWARA_CLUSTER_H_

#include <tawara/block_element.h>
#include <tawara/master_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>

/// \addtogroup interfaces Interfaces
/// @{

namespace tawara
{
    /** \brief The track number of a silent track.
     *
     * This element stores the track number of a track that has been made
     * silent within this cluster.
     */
    class SilentTrackNumber : public UIntElement
    {
        public:
            SilentTrackNumber(uint64_t track_number)
                : UIntElement(ids::SilentTrackNumber, track_number)
            {
            }
    };

    /** \brief The base Cluster, defining the common interface for Cluster
     * element implementations.
     *
     * This class is the base class for all Cluster implementations. Different
     * concrete implementations of this interface implement reading and writing
     * of blocks in different ways. The two most commonly-used implementations
     * are the in-memory cluster and the streamed-writing cluster.
     *
     * Because of their nature as streamed data, Clusters are the most complex
     * element to write. They are often written in stages, with a dummy size
     * value and the other data written first, before the blocks are streamed
     * in, and finally the correct size value written over the dummy size value
     * at the start of the cluster. Alternative implementations may store all
     * cluster data in memory (or even in another file) before writing the
     * cluster in one hit.
     *
     * The Cluster interface supports both streaming and all-at-once approaches
     * to writing. The sequence of method calls is the same for both cases, but
     * what is done for each call varies. The sequence of method calls that
     * must be performed is:
     *
     * \verbatim
     *  cluster.write(output)
     *          ||
     *          \/
     *  [Capture blocks]
     *          ||
     *          \/
     *  cluster.finalise(output)
     * \endverbatim
     *
     * The purpose of the write step is to allow Cluster implementations that
     * use stream-based writing to prepare the file for writing the blocks.
     * The finalise step is used to finalise the cluster in the file, ensuring,
     * for example, that the correct size value is written into the element
     * header.
     *
     * For a Cluster implementation that stores the block data elsewhere (e.g.
     * in memory) before writing the entire cluster in one go, the method calls
     * could be implemented to do the following:
     *
     * \verbatim
     *  cluster.write(output)    -> Prepare space outside of the file to store
     *          ||                  the blocks while they are accumulated. For
     *          ||                  example, a block of memory could be
     *          ||                  allocated to store the blocks.
     *          \/
     *  [Capture blocks]         -> Store blocks in the reserved space.
     *          ||
     *          \/
     *  cluster.finalise(output) -> Write the Cluster ID and size (calculated
     *                              from the stored blocks), followed by the
     *                              other Cluster fields, and then the stored
     *                              blocks.
     * \endverbatim
     *
     * For a Cluster implementation that streams the blocks directly into the
     * file as they arrive, the method calls could be implemented to do the
     * following:
     *
     * \verbatim
     *  cluster.write(output)    -> Write the Cluster ID with a base value for
     *          ||                  the size (a good value to use is the size
     *          ||                  of an empty cluster).  Following this,
     *          ||                  write the other Cluster fields.
     *          \/
     *  [Capture blocks]         -> Write blocks directly to the file as they
     *          ||                  are received.
     *          \/
     *  cluster.finalise(output) -> Calculate the actual cluster size (e.g.
     *                              subtract the position in the file of the
     *                              first block from the position in the file
     *                              after the last block), and write it over
     *                              the dummy value written earlier.
     * \endverbatim
     *
     * The second approach described above has a \e very important limitation:
     * the cluster implementation must manage the file write pointer carefully
     * to ensure that blocks are placed in the correct place in the file. For
     * example, upon receiving a block to write, the file write pointer is
     * positioned after the previously-written block, and after writing the
     * block is returned to the position it was originally in.
     */
    //template <typename Impl>
    class TAWARA_EXPORT Cluster : public MasterElement
    {
        public:
            /// \brief Pointer to a cluster.
            typedef boost::shared_ptr<Cluster> Ptr;
            /// \brief The value type of this container.
            typedef BlockElement::Ptr value_type;
            /// \brief The size type of this container.
            typedef size_t size_type;
            /// \brief The reference type.
            typedef value_type& reference;
            /// \brief The constant reference type.
            typedef value_type const& const_reference;

            /** \brief Construct a new Cluster.
             *
             * \param[in] timecode The timecode of the cluster, in the units
             * specified by TimecodeScale.
             */
            Cluster(uint64_t timecode=0);

            /// \brief Destructor.
            virtual ~Cluster() {};

            //////////////////////////////////////////////////////////////////
            // Cluster interface
            //////////////////////////////////////////////////////////////////

            /// \brief Check if there are no blocks.
            virtual bool empty() const = 0;
            /// \brief Get the number of blocks.
            virtual size_type count() const = 0;
            /// \brief Remove all blocks.
            virtual void clear() = 0;

            /* \brief Erase the block at the specified iterator.
             *
             * \param[in] position The position to erase at.
             */
            //virtual void erase(typename Impl::Iterator position) = 0;
            /* \brief Erase a range of blocks.
             *
             * \param[in] first The start of the range.
             * \param[in] last The end of the range.
             */
            //virtual void erase(typename Impl::Iterator first,
                    //typename Impl::Iterator last) = 0;

            /** \brief Add a block to this cluster.
             *
             * The cluster must be in the writable state. This means that
             * write() has been called and finish_write() has not been called.
             *
             * \throw ClusterNotReady if the Cluster is not in the correct
             * state for writing blocks.
             */
            virtual void push_back(value_type const& value) = 0;

            /** \brief Get the cluster's timecode.
             *
             * This timecode defines the base timecode for all blocks in the
             * cluster. It is specified in units of the TimecodeScale found in
             * the SegmentInfo element for the same segment as the cluster.
             */
            uint64_t timecode() const { return timecode_; }
            /// \brief Set the cluster's timecode.
            void timecode(uint64_t timecode) { timecode_ = timecode; }

            /** \brief Get the list of silent tracks.
             *
             * Some tracks in a cluster may be marked as silent. This means
             * that all blocks for those tracks should be ignored within this
             * cluster. This property lists the \e track \e numbers of the
             * silent tracks.
             *
             * A track being made silent in this cluster has no effect on its
             * silence in other clusters.
             */
            std::vector<SilentTrackNumber>& silent_tracks()
                { return silent_tracks_; }

            /** \brief Get the position of this cluster in the segment.
             *
             * This property gives the byte-offset of this cluster in its segment.
             * This value is useful for re-synchronising damaged streams.
             *
             * If it is zero, then the cluster has not been written or was not
             * read from a byte stream.
             */
            uint64_t position() const;

            /** \brief Get the size of the previous cluster in the segment.
             *
             * This property gives the size of the previous cluster in bytes.
             * This can be used to jump straight to the start of the previous
             * cluster, rather than searching for it.
             *
             * It it is zero, the size of the previous cluster is unknown.
             */
            uint64_t previous_size() const { return prev_size_; }
            /// \brief Set the size of the previous cluster in the segment.
            void previous_size(uint64_t size) { prev_size_ = size; }

            /// \brief Get the total size of the element.
            std::streamsize size() const;

            /** \brief Element reading.
             *
             * \throw DuplicateTrackNumber if more than one TrackEntry in the
             * stored element has the same track number.
             * \throw DuplicateUID if more than one TrackEntry in the stored
             * element has the same UID.
             */
            std::streamsize read(std::istream& input)
                { return Element::read(input); }

            /** \brief Finalise writing of the cluster.
             *
             * See the Cluster documentation for more details of how this
             * method should be implemented. Once this is called, the cluster
             * should be considered final in the stream, including all the
             * cluster's meta-data and all blocks.
             *
             * \param[in] output The byte stream to write the cluster to.
             * \return The final total size, in bytes, of the cluster.
             */
            virtual std::streamsize finalise(std::ostream& output) = 0;

        protected:
            UIntElement timecode_;
            std::vector<SilentTrackNumber> silent_tracks_;
            UIntElement position_;
            UIntElement prev_size_;
            bool writing_;

            /// \brief Get the size of the meta-data portion of the body of
            //this element.
            std::streamsize meta_size() const;

            /// \brief Get the size of the body of this element.
            std::streamsize body_size() const
                { return meta_size() + blocks_size(); }

            /// \brief Element size writing.
            std::streamsize write_size(std::ostream& output);

            /// \brief Element body writing.
            std::streamsize write_body(std::ostream& output);

            /// \brief Element body loading.
            std::streamsize read_body(std::istream& input,
                    std::streamsize size);

            /// \brief Get the size of the blocks in this cluster.
            virtual std::streamsize blocks_size() const = 0;

            /** \brief Read the blocks in this cluster from the output stream.
             *
             * This function may not necessarily perform the actual reading,
             * but once called, the blocks should be accessible through
             * whatever interface the Cluster implementation provides.
             *
             * For example, if the blocks are actually read by an iterator,
             * calling this function should prepare for the iterators' use. It
             * might, for example, read the position of each block and store it
             * in an index.
             *
             * \param[in] input The input byte stream to read blocks from.
             * \param[in] size The number of bytes available for reading.
             * Exactly this many bytes should be used, or an error should be
             * reported.
             * \return The total size of the cluster's blocks (as stored in the
             * stream), i.e. the quantity of data "read". Even if only a small
             * quantity of data is actually read, it must return the complete
             * blocks size of the cluster in order to meet the Element
             * interface requirements.
             */
            virtual std::streamsize read_blocks(std::istream& input,
                    std::streamsize size) = 0;

            /** \brief Read the SilentTracks child element.
             *
             * \return The number of bytes read.
             */
            std::streamsize read_silent_tracks(std::istream& input);

            /// \brief Reset the cluster's members to default values.
            virtual void reset();
    }; // class Cluster
}; // namespace tawara

/// @}
// group interfaces

#endif // TAWARA_CLUSTER_H_

