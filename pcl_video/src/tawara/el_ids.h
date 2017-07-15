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

#if !defined(TAWARA_EL_IDS_H_)
#define TAWARA_EL_IDS_H_

#include <ios>
#include <stdint.h>
#include <vector>

/// \addtogroup ebml EBML
/// @{

namespace tawara
{
    /** \brief This namespace contains constants and functions for managing
     * EBML class IDs.
     *
     * The EBML class IDs used for the elements in a Tawara document and
     * functions for reading/writing them to byte streams are contained in this
     * namespace.
     */
    namespace ids
    {
        typedef uint32_t ID;

        const ID Null(0x80);

        const ID Void(0xEC);
        const ID CRC32(0xBF);

        const ID EBML(0x1A45DFA3);
            const ID EBMLVersion(0x4286);
            const ID EBMLReadVersion(0x42F7);
            const ID EBMLMaxIDLength(0x42F2);
            const ID EBMLMaxSizeLength(0x42F3);
            const ID DocType(0x4282);
            const ID DocTypeVersion(0x4287);
            const ID DocTypeReadVersion(0x4285);

        const ID Segment(0x18538067);

            const ID SeekHead(0x114D9B74);
                const ID Seek(0x4DBB);
                    const ID SeekID(0x53AB);
                    const ID SeekPosition(0x53AC);

            const ID Info(0x1549A966);
                const ID SegmentUID(0x73A4);
                const ID SegmentFileName(0x7384);
                const ID PrevUID(0x3CB923);
                const ID PrevFileName(0x3C83AB);
                const ID NextUID(0x3EB923);
                const ID NextFileName(0x3E83AB);
                const ID SegmentFamily(0x4444);
                const ID TimecodeScale(0x2AD7B1);
                const ID Duration(0x4489);
                const ID DateUTC(0x4461);
                const ID Title(0x7BA9);
                const ID MuxingApp(0x4D80);
                const ID WritingApp(0x5741);

            const ID Cluster(0x1F43B675);
                const ID Timecode(0xE7);
                const ID SilentTracks(0x5854);
                    const ID SilentTrackNumber(0x58D7);
                const ID Position(0xA7);
                const ID PrevSize(0xAB);
                const ID SimpleBlock(0xA3);
                const ID BlockGroup(0xA0);
                    const ID Block(0xA1);
                    const ID BlockAdditions(0x75A1);
                        const ID BlockMore(0xA6);
                            const ID BlockAddID(0xEE);
                            const ID BlockAdditional(0xA5);
                    const ID BlockDuration(0x9B);
                    const ID ReferencePriority(0xFA);
                    const ID ReferenceBlock(0xFB);
                    const ID CodecState(0xA4);

            const ID Tracks(0x1654AE6B);
                const ID TrackEntry(0xAE);
                    const ID TrackNumber(0xD7);
                    const ID TrackUID(0x73C5);
                    const ID TrackType(0x83);
                    const ID FlagEnabled(0xB9);
                    const ID FlagDefault(0x88);
                    const ID FlagForced(0x55AA);
                    const ID FlagLacing(0x9C);
                    const ID MinCache(0x6DE7);
                    const ID MaxCache(0x6DF8);
                    const ID DefaultDuration(0x23E383);
                    const ID TrackTimecodeScale(0x23314F);
                    const ID MaxBlockAdditionID(0x55EE);
                    const ID Name(0x536E);
                    const ID CodecID(0x86);
                    const ID CodecPrivate(0x63A2);
                    const ID CodecName(0x258688);
                    const ID AttachmentLink(0x7446);
                    const ID CodecDecodeAll(0xAA);
                    const ID TrackOverlay(0x6F24);
                    const ID TrackOperation(0xE2);
                        const ID TrackJoinBlocks(0xE9);
                            const ID TrackJoinUID(0xED);

            const ID Cues(0x1C53BB6B);
                const ID CuePoint(0xBB);
                    const ID CueTime(0xB3);
                    const ID CueTrackPosition(0xB7);
                        const ID CueTrack(0xF7);
                        const ID CueClusterPosition(0xF1);
                        const ID CueBlockNumber(0x5378);
                        const ID CueCodecState(0xEA);
                        const ID CueReference(0xDB);
                            const ID CueRefTime(0x96);

            const ID Attachments(0x1941A469);
                const ID AttachedFile(0x61A7);
                    const ID FileDescription(0x467E);
                    const ID FileName(0x466E);
                    const ID FileMimeType(0x4660);
                    const ID FileData(0x465C);
                    const ID FileUID(0x46AE);

            const ID Chapters(0x1043A770);
                const ID EditionEntry(0x45B9);
                    const ID EditionUID(0x45BC);
                    const ID EditionFlagHidden(0x45BD);
                    const ID EditionFlagDefault(0x45DB);
                    const ID EditionFlagOrdered(0x45DD);
                    const ID ChapterAtom(0xB6);
                        const ID ChapterUID(0x73C4);
                        const ID ChapterTimeStart(0x91);
                        const ID ChapterTimeEnd(0x92);
                        const ID ChapterFlagHidden(0x98);
                        const ID ChapterFlagEnabled(0x4598);
                        const ID ChapterSegmentUID(0x6E67);
                        const ID ChapterTrack(0x8F);
                            const ID ChapterTrackNumber(0x89);
                        const ID ChapterDisplay(0x80);
                            const ID ChapString(0x85);
                            const ID ChapLanguage(0x437C);
                            const ID ChapCountry(0x437E);

            const ID Tags(0x1254C367);
                const ID Tag(0x7373);
                    const ID Targets(0x63C0);
                        const ID TagTrackUID(0x63C5);
                        const ID TagEditionUID(0x63C9);
                        const ID TagChapterUID(0x63C4);
                        const ID TagAttachmentUID(0x63C6);
                    const ID SimpleTag(0x67C8);
                        const ID TagName(0x45A3);
                        const ID TagLanguage(0x447A);
                        const ID TagDefault(0x4484);
                        const ID TagString(0x4487);
                        const ID TagBinary(0x4485);

        /** \brief Get the number of bytes required by an ID.
         *
         * The size required by an ID depends on its value, and will range from
         * 1 to 8 bytes.
         *
         * \param[in] id The ID to get the size of.
         * \return The size, in bytes, that the ID will require for optimum
         * storage.
         * \exception InvalidEBMLID if the ID is invalid.
         */
        std::streamsize size(ID id);

        /** \brief Encode an unsigned integer into a buffer.
         *
         * Encodes an unsigned variable-length integer according to the EBML
         * specification. Leading zero bits are used to indicate the length of
         * the encoded integer in bytes.
         *
         * \param[in] integer The integer to encode.
         * \return A vector containing the encoded data.
         * \exception InvalidEBMLID if the ID is invalid.
         */
        std::vector<char> encode(ID integer);

        /** \brief The result of a decode operation is a pair of the ID
         * decoded and an iterator pointing to the first element after the used
         * data.
         */
        typedef std::pair<uint64_t, std::vector<char>::const_iterator>
            DecodeResult;

        /** \brief Decode an ID from a buffer.
         *
         * Decodes the ID stored in the buffer.
         *
         * \param[in] buffer The buffer holding the raw data.
         * \return The DecodeResult, containing the decoded ID
         * and an iterator pointing to the first element after the used data.
         * \exception InvalidVarInt if the first byte in the buffer is
         * zero, an invalid starting byte for a variable-length integer.
         * \exception BufferTooSmall if the expected encoded length of the
         * variable-length integer is larger than the available buffer length.
         * \exception InvalidEBMLID if the ID is invalid.
         */
        DecodeResult decode(std::vector<char> const& buffer);

        /** \brief Write an ID to an output stream.
         *
         * This function writes an ID to an output stream, using the value of
         * the ID to calculate the number of bytes required for storage.
         *
         * \param[in] id The ID to write.
         * \param[in] output The std::ostream object to write to.
         * \return The number of bytes written.
         * \exception InvalidEBMLID if the ID is invalid.
         * \exception WriteError if there is an error writing to the stream.
         */
        std::streamsize write(ID id, std::ostream& output);

        /** \brief The result of a read operation is a pair of the ID read
         * and the number of bytes read.
         */
        typedef std::pair<ID, std::streamsize> ReadResult;

        /** \brief Read an ID from an input stream.
         *
         * This function reads an ID from an input stream, using the value of
         * the first byte to determine the length of the ID.
         *
         * \param[in] input The std::istream object to read bytes from.
         * \return A pair containing the ID read in the first and the number
         * of bytes read from the stream in the second.
         * \exception InvalidEBMLID if the ID is invalid.
         * \exception InvalidVarInt if the ID in the byte stream is unreadable.
         * \exception ReadError if there is an error reading the input stream.
         */
        ReadResult read(std::istream& input);
    }; // namespace ids
}; // namespace tawara

/// @} // group ebml

#endif // TAWARA_EL_IDS_H_

