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

#if !defined(TAWARA_EXCEPTIONS_H_)
#define TAWARA_EXCEPTIONS_H_

#include <tawara/el_ids.h>
#include <tawara/win_dll.h>

#include <boost/exception/all.hpp>
#include <exception>
#include <stdint.h>
#include <vector>

/// \addtogroup exceptions Exceptions
/// @{

namespace tawara
{
    /// \brief Base error type.
    struct TawaraError : virtual std::exception, virtual boost::exception {};

///////////////////////////////////////////////////////////////////////////////
// Error types
///////////////////////////////////////////////////////////////////////////////

    /// \brief Something is not supported.
    struct NotImplemented : virtual TawaraError {};

    /** \brief File is not an EBML file.
     *
     * Tawara uses EBML for its file format. If an opened file is not an EBML
     * file, this error will occur.
     */
    struct NotEBML : virtual TawaraError {};

    /** \brief File is not a Tawara file.
     *
     * Usually encountered when trying to open an existing name (either for
     * reading or appending) that is not a Tawara object (e.g. a file that is not
     * a Tawara file).
     */
    struct NotTawara : virtual TawaraError {};

    /** \brief The required EBML read version is too high.
     *
     * The EBML header specifies the minimum EBML parser version necessary to
     * be able to read the EBML file. If it is too high, this error occurs.
     *
     * The version from the file may be attached as an err_ver tag.
     */
    struct BadReadVersion : virtual TawaraError {};

    /** \brief The required Tawara read version is too high.
     *
     * The EBML header specifies the minimum Tawara parser version necessary to
     * be able to read the Tawara file. If it is too high, this error occurs.
     *
     * The version from the file may be attached as an err_ver tag.
     */
    struct BadDocReadVersion : virtual TawaraError {};

    /** \brief An invalid EBML class ID was found.
     *
     * EBML class IDs are encoded as variable-length integers. This means they
     * must occupy certain ranges within each set of bytes used. If a
     * variable-length integer outside one of the valid ranges is found while
     * reading or writing IDs, this error occurs.
     *
     * An err_varint tag may be included, giving the invalid ID.
     *
     * An err_pos tag may be included, indicating where the bad ID was
     * encountered.
     */
    struct InvalidEBMLID : virtual TawaraError {};

    /** \brief An invalid variable-length integer was found.
     *
     * Encountered when reading a value stored as a variable-length integer,
     * such as a tag or an element size value, that is incorrectly-formatted.
     * This indicates that the file is corrupted.
     *
     * An err_pos tag will often be included indicating where the bad
     * variable-length integer was encountered.
     */
    struct InvalidVarInt : virtual TawaraError {};

    /** \brief A variable-length integer is too large to be encoded.
     *
     * Encountered when encoding an integer as a variable-length integer for
     * writing to a Tawara object. The maximum allowable size of a
     * variable-length integer is given in the EBML specification.
     *
     * The err_varint tag will often be included, indicating the value that
     * was attempted to be encoded.
     *
     * The err_bufsize tag may be included to indicate the size of a buffer
     * that the variable-length integer was to be written to.
     */
    struct VarIntTooBig : virtual TawaraError {};

    /** \brief A specified size for a variable-length integer is too small.
     *
     * Encountered when encoding a variable-length integer into a fixed size.
     * Usually, this is a size that is larger than would normally be necessary,
     * but if the variable-length integer requires more bytes than the
     * specified fixed size, this error will occur.
     *
     * The err_varint tag may be included, indicating the value that was
     * attempted to be encoded.
     *
     * The err_reqsize tag may be included, indicating the size is required to
     * encode the integer.
     *
     * The err_specsize tag may be included, giving the size that was
     * requested.
     */
    struct SpecSizeTooSmall : virtual TawaraError {};

    /** \brief A buffer was too small for the data.
     *
     * Encountered in any situation where data will be written to a buffer. For
     * example, attempting to write a large variable-length integer to a buffer
     * too small to hold all the bytes will trigger this error.
     *
     * The err_bufsize tag may be included to indicate the size of the buffer.
     *
     * The err_reqsize tag may be included to indicate the required size.
     */
    struct BufferTooSmall : virtual TawaraError {};

    /** \brief A read error was encountered during a read.
     *
     * This error may occur anywhere that involves reading a file or file-like
     * stream. It most commonly indicates an end-of-file error, i.e. a
     * truncated file.
     *
     * The err_name tag may be included to indicate the name of the file.
     *
     * The err_pos tag may be included to indicate where in the file the error
     * occured.
     *
     * The err_reqsize tag may be included to indicate the size of the read
     * that was attempted.
     */
    struct ReadError : virtual TawaraError {};

    /** \brief A write error was encountered during a write.
     *
     * This error may occur anywhere that involves writing a file or file-like
     * stream. It most commonly indicates that there is no more space available
     * for writing to.
     *
     * The err_name tag may be included to indicate the name of the file.
     *
     * The err_pos tag may be included to indicate where in the file the error
     * occured.
     */
    struct WriteError : virtual TawaraError {};

    /** \brief An invalid Element ID was provided.
     *
     * When setting the ID of an Element, if the ID is one of the invalid
     * values, this error will occur.
     *
     * The err_id tag may be provided, indicating the invalid ID.
     */
    struct InvalidElementID : virtual TawaraError {};

    /** \brief A fixed-length element is truncated or lengthened in the file.
     *
     * Some elements are stored with a fixed length in the file:
     * - Date elements are always 8 bytes.
     * - Float elements are 4 or 8 bytes, depending on necessary size.
     *
     * If one of these elements is found with an incorrect size, this error is
     * raised.
     *
     * The err_pos tag may be included to indicate where in the file the error
     * occured.
     *
     * The err_id tag may be provided to indicate the ID of the element that is
     * corrupted.
     *
     * The err_el_size tag may be provided, giving the size found.
     *
     * The err_valid_sizes tag may be provided, giving the valid sizes.
     */
    struct BadElementLength : virtual TawaraError {};

    /** \brief A read body size does not match the actual body size in the
     * file.
     *
     * The body size given at the start of a master element must match the
     * total size of all child elements. If it does not, this error occurs.
     *
     * The err_el_size tag may be provided, giving the size found.
     */
    struct BadBodySize : virtual TawaraError {};

    /** \brief A child element was found where it doesn't belong.
     *
     * When reading an element's children, if an element ID is found that does
     * not belong in the parent, this error is raised.
     *
     * The err_id tag may be included to indicate the invalid ID.
     *
     * The err_par_id tag may be included to indicate the parent element being
     * read.
     *
     * The err_pos tag may be included to indicate where in the file the error
     * occured.
     */
    struct InvalidChildID : virtual TawaraError {};

    /** \brief A necessary child element was missing.
     *
     * Some child elements are required to be present, but don't have default
     * values. When a child element is not found, this error occurs.
     *
     * The err_id tag may be included to give the ID of the missing child
     * element.
     *
     * The err_par_id tag may be included to give the ID of the parent element.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct MissingChild : virtual TawaraError{};

    /** \brief A child element's value was set outside the allowable range.
     *
     * Often, child elements will have an allowable range of values, such as
     * not zero or positive integers. This error occurs when a child element's
     * value is set outside its allowable range. See the Tawara format
     * specification for the allowable range of each element.
     *
     * The err_id tag may be included to give the ID of the bad child
     * element.
     *
     * The err_par_id tag may be included to give the ID of the parent element.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct ValueOutOfRange : virtual TawaraError{};

    /** \brief A child element's size is below or above the required size.
     *
     * Some string or binary child elements have a required size, such as the
     * 8 bytes necessary for the UID elements in a SegmentInfo element. If a
     * child element's value is set to something with an incorrect size, this
     * error occurs.
     *
     * The err_id tag may be included to give the ID of the bad child
     * element.
     *
     * The err_par_id tag may be included to give the ID of the parent element.
     */
    struct ValueSizeOutOfRange : virtual TawaraError{};

    /** \brief An empty Tracks element was read or written.
     *
     * The Tracks element must have at least one TrackEntry to be valid. This
     * error occurs if a Tracks element with no TrackEntry children is read, or
     * when an empty Tracks element is about to be written.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct EmptyTracksElement : virtual TawaraError{};

    /** \brief A duplicate track number was encountered.
     *
     * All tracks within an element must have a unique track number. When
     * reading or creating a Tracks element, if more than one TrackEntry in the
     * element has the same track number, this error occurs.
     *
     * The err_track_num tag may be included to indicate the duplicated track
     * number.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct DuplicateTrackNumber : virtual TawaraError{};

    /** \brief A UID collision was encountered.
     *
     * In many places, the same element may occur multiple times, with each
     * instance distinguished by a UID represented as an unsigned integer. When
     * the UIDs of two elements clash, this error occurs.
     *
     * The err_int_uid tag may be included to indicate the clashing UID.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct DuplicateUID : virtual TawaraError{};

    /** \brief An empty block was encountered.
     *
     * A block must contain frame data. An empty block is an error, either when
     * reading or writing.
     */
    struct EmptyBlock : virtual TawaraError{};

    /** \brief An empty frame was encountered.
     *
     * Frames without data are illegal. This error occurs when one is
     * encountered.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured, if it occured while reading a file.
     */
    struct EmptyFrame : virtual TawaraError{};

    /** \brief The maximum lace size for a block was exceeded.
     *
     * Different forms of lacing may have different limits on the number of
     * frames that can be stored other than infinity. In the case of
     * tawara::BlockBase::LACING_NONE, this limit is 0. This error occurs when
     * more frames than the block can hold, based on its lacing policy, are
     * added.
     *
     * The err_max_lace tag may be included to give the maximum number of
     * frames in the lace.
     *
     * The err_req_lace tag may be included to give the requested number of
     * frames in the lace.
     */
    struct MaxLaceSizeExceeded : virtual TawaraError{};

    /** \brief A frame with a bad size was added to a block.
     *
     * When a block is using fixed lacing, all frames in the block must be the
     * same size. If a frame is added to a block with a different size from the
     * first frame (if present), or when a block is written with fixed lacing
     * and frames of different sizes, this error occurs.
     *
     * This error may also occur when reading a block written using EBML lacing
     * if the calculated size of one of the frames is less than zero.
     *
     * The err_frame_size tag may be included to indicate the size of the bad
     * frame.
     */
    struct BadLacedFrameSize : virtual TawaraError{};

    /** \brief An empty BlockAdditions element was read or written.
     *
     * The BlockAdditions element must have at least one BlockMore to be valid.
     * This error occurs if a BlockAdditions element with no BlockMore children
     * is read, or when an empty BlockAdditions element is about to be written.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct EmptyBlockAdditionsElement : virtual TawaraError{};

    /** \brief A segment was found with multiple meta-seeks.
     *
     * Segments may only have one SeekHead element. If multiple are found, this
     * error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct MultipleSeekHeads : virtual TawaraError{};

    /** \brief A segment was found without a segment info element.
     *
     * Every Segment element must have an Info element present. If none is
     * found, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct NoSegmentInfo : virtual TawaraError{};

    /** \brief A segment was found without a tracks information element.
     *
     * Every Segment element must have a Tracks element present. If none is
     * found, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct NoTracks : virtual TawaraError{};

    /** \brief A segment was found without at least one cluster.
     *
     * Every Segment element must have at least one Cluster element present. If
     * none is found, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct NoClusters : virtual TawaraError{};

    /** \brief A segment or cluster was finalised before being started.
     *
     * Segments and clusters must be finalised to complete their writing, but
     * this must occur after a call to write. If a segment or cluster is
     * finalised before writing is begun, this error occurs.
     */
    struct NotWriting : virtual TawaraError{};

    /** \brief The requested size of a void element is too small.
     *
     * Void elements must be at least 2 bytes long to accomodate the ID and the
     * element size value.
     *
     * The err_reqsize tag may be included to give the requested size.
     */
    struct VoidTooSmall : virtual TawaraError{};

    /** \brief An attachments element with no attachments was read or written.
     *
     * An Attachments element must have at least one attachment. If an empty
     * element is read or written, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct NoAttachments : virtual TawaraError{};

    /** \brief An attached file with no data was read or written.
     *
     * An attached file must contain at least 1 byte of data. If one without
     * any data is read or written, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct NoAttachedData : virtual TawaraError{};

    /** \brief A duplicate timecode was encountered in the cues.
     *
     * All cue points in the cues must have a unique timecode. When reading or
     * creating a Cues element, if more than one cue point uses the same time
     * code, this error occurs.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct DuplicateTimecode : virtual TawaraError{};

    /** \brief An empty Cues element was read or written.
     *
     * The Cues element must have at least one CuePoint to be valid. This
     * error occurs if a Cues element with no CuePoint children is read, or
     * when an empty Cues element is about to be written.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct EmptyCuesElement : virtual TawaraError{};

    /** \brief An empty CuePoint element was read or written.
     *
     * The CuePoint element must have at least one CueTrackPositions to be
     * valid. This error occurs if a CuePoint element with no CueTrackPositions
     * children is read, or when an empty CuePoint element is about to be
     * written.
     *
     * The err_pos tag may be included to give the approximate position in the
     * file where the error occured.
     */
    struct EmptyCuePointElement : virtual TawaraError{};


///////////////////////////////////////////////////////////////////////////////
// Error information tags
///////////////////////////////////////////////////////////////////////////////

    /// \brief A version.
    typedef boost::error_info<struct tag_ver, std::streamsize> err_ver;

    /// \brief Position in a Tawara file.
    typedef boost::error_info<struct tag_pos, std::streamsize> err_pos;

    /// \brief Value of a variable-length integer.
    typedef boost::error_info<struct tag_varint, uint64_t> err_varint;

    /// \brief The size of a buffer.
    typedef boost::error_info<struct tag_bufsize, std::streamsize> err_bufsize;

    /// \brief The required size of a buffer or a file read.
    typedef boost::error_info<struct tag_reqsize, std::streamsize> err_reqsize;

    /// \brief The specified size to encode a variable-length integer into.
    typedef boost::error_info<struct tag_specsize, std::streamsize>
        err_specsize;

    /// \brief An Element ID.
    typedef boost::error_info<struct tag_id, ids::ID> err_id;

    /// \brief A parent element ID.
    typedef boost::error_info<struct tag_par_id, uint32_t> err_par_id;

    /// \brief A set of valid element sizes.
    typedef boost::error_info<struct tag_valid_sizes,
            std::vector<std::streamsize> > err_valid_sizes;

    /// \brief The size of an element.
    typedef boost::error_info<struct tag_el_size, std::streamsize> err_el_size;

    /// \brief A track number.
    typedef boost::error_info<struct tag_track_num, uint64_t> err_track_num;

    /// \brief An integer UID.
    typedef boost::error_info<struct tag_int_uid, uint64_t> err_int_uid;

    /// \brief The maximum size of a lace.
    typedef boost::error_info<struct tag_max_lace, unsigned int> err_max_lace;

    /// \brief The requested size of a lace.
    typedef boost::error_info<struct tag_req_lace, unsigned int> err_req_lace;

    /// \brief The size of a frame.
    typedef boost::error_info<struct tag_frame_size, std::streamsize>
        err_frame_size;
}; // namespace tawara

/// @}
// group exceptions

#endif // TAWARA_EXCEPTIONS_H_

