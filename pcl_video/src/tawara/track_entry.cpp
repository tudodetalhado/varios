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

#include <tawara/track_entry.h>

#include <boost/foreach.hpp>
#include <tawara/el_ids.h>
#include <tawara/element.h>
#include <tawara/vint.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

TrackEntry::TrackEntry(uint64_t number, uint64_t uid, std::string const& codec)
    : MasterElement(ids::TrackEntry),
    number_(ids::TrackNumber, number), uid_(ids::TrackUID, uid),
    type_(ids::TrackType, 0x70), enabled_(ids::FlagEnabled, 1, 1),
    forced_(ids::FlagForced, 0, 0), lacing_(ids::FlagLacing, 1, 1),
    min_cache_(ids::MinCache, 0, 0), max_cache_(ids::MaxCache, 0, 0),
    default_dur_(ids::DefaultDuration, 0),
    timecode_scale_(ids::TrackTimecodeScale, 1.0, 1.0),
    max_block_add_id_(ids::MaxBlockAdditionID, 0, 0),
    name_(ids::Name, ""), codec_id_(ids::CodecID, codec),
    codec_private_(ids::CodecPrivate, std::vector<char>()),
    codec_name_(ids::CodecName, ""), attachment_link_(ids::AttachmentLink, 0),
    decode_all_(ids::CodecDecodeAll, 0, 0)
{
    if (number == 0)
    {
        throw ValueOutOfRange() << err_id(number_.id()) << err_par_id(id_);
    }
    if (uid == 0)
    {
        throw ValueOutOfRange() << err_id(uid_.id()) << err_par_id(id_);
    }
    if (codec.empty())
    {
        throw ValueOutOfRange() << err_id(codec_id_.id()) <<
            err_par_id(id_);
    }
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void TrackEntry::number(uint64_t number)
{
    if (number == 0)
    {
        throw ValueOutOfRange() << err_id(number_.id()) << err_par_id(id_);
    }
    number_ = number;
}


void TrackEntry::uid(uint64_t uid)
{
    if (uid == 0)
    {
        throw ValueOutOfRange() << err_id(uid_.id()) << err_par_id(id_);
    }
    uid_ = uid;
}


void TrackEntry::type(uint8_t type)
{
    // Valid range: 0 - 254
    if (type == 255)
    {
        throw ValueOutOfRange() << err_id(type_.id()) << err_par_id(id_);
    }
    type_ = type;
}


void TrackEntry::timecode_scale(double timecode_scale)
{
    if (timecode_scale <= 0.0)
    {
        throw ValueOutOfRange() << err_id(timecode_scale_.id()) <<
            err_par_id(id_);
    }
    timecode_scale_ = timecode_scale;
}


void TrackEntry::codec_id(std::string id)
{
    if (id.empty())
    {
        throw ValueOutOfRange() << err_id(codec_id_.id()) <<
            err_par_id(id_);
    }
    codec_id_ = id;
}


std::vector<uint64_t> TrackEntry::overlays() const
{
    std::vector<uint64_t> result;
    BOOST_FOREACH(UIntElement overlay, overlays_)
    {
        result.push_back(overlay);
    }
    return result;
}


void TrackEntry::overlays(std::vector<uint64_t> const& uids)
{
    overlays_.clear();
    BOOST_FOREACH(uint64_t uid, uids)
    {
        overlays_.push_back(UIntElement(ids::TrackOverlay, uid));
    }
}


///////////////////////////////////////////////////////////////////////////////
// Operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(TrackEntry const& lhs, TrackEntry const& rhs)
{
    return lhs.number_ == rhs.number_ &&
        lhs.uid_ == rhs.uid_ &&
        lhs.type_ == rhs.type_ &&
        lhs.enabled_ == rhs.enabled_ &&
        lhs.forced_ == rhs.forced_ &&
        lhs.lacing_ == rhs.lacing_ &&
        lhs.min_cache_ == rhs.min_cache_ &&
        lhs.max_cache_ == rhs.max_cache_ &&
        lhs.default_dur_ == rhs.default_dur_ &&
        lhs.timecode_scale_ == rhs.timecode_scale_ &&
        lhs.max_block_add_id_ == rhs.max_block_add_id_ &&
        lhs.name_ == rhs.name_ &&
        lhs.codec_id_ == rhs.codec_id_ &&
        lhs.codec_private_ == rhs.codec_private_ &&
        lhs.codec_name_ == rhs.codec_name_ &&
        lhs.attachment_link_ == rhs.attachment_link_ &&
        lhs.decode_all_ == rhs.decode_all_ &&
        lhs.overlays_ == rhs.overlays_ &&
        lhs.operation_ == rhs.operation_;
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize TrackEntry::body_size() const
{
    std::streamsize size(0);
    size += number_.size();
    size += uid_.size();
    size += type_.size();
    size += codec_id_.size();
    if (!enabled_.is_default())
    {
        size += enabled_.size();
    }
    if (!forced_.is_default())
    {
        size += forced_.size();
    }
    if (!lacing_.is_default())
    {
        size += lacing_.size();
    }
    if (!min_cache_.is_default())
    {
        size += min_cache_.size();
    }
    if (!max_cache_.is_default())
    {
        size += max_cache_.size();
    }
    if (default_dur_ != 0)
    {
        size += default_dur_.size();
    }
    if (!timecode_scale_.is_default())
    {
        size += timecode_scale_.size();
    }
    if (!max_block_add_id_.is_default())
    {
        size += max_block_add_id_.size();
    }
    if (!name_.value().empty())
    {
        size += name_.size();
    }
    if (!codec_private_.value().empty())
    {
        size += codec_private_.size();
    }
    if (!codec_name_.value().empty())
    {
        size += codec_name_.size();
    }
    if (attachment_link_ != 0)
    {
        size += attachment_link_.size();
    }
    if (!decode_all_.is_default())
    {
        size += decode_all_.size();
    }
    BOOST_FOREACH(UIntElement overlay, overlays_)
    {
        size += overlay.size();
    }
    if (operation_)
    {
        size += tawara::ids::size(ids::TrackOperation) +
            tawara::vint::size(operation_->size()) +
            operation_->size();
    }
    return size;
}


std::streamsize TrackEntry::write_body(std::ostream& output)
{
    assert(number_ != 0);
    assert(uid_ != 0);
    assert(type_ >= 0 && type_ < 255);
    assert(enabled_ == 0 || enabled_ == 1);
    assert(forced_ == 0 || forced_ == 1);
    assert(lacing_ == 0 || lacing_ == 1);
    assert(timecode_scale_ > 0.0);
    assert(decode_all_ == 0 || decode_all_ == 1);

    std::streamsize written(0);
    written += number_.write(output);
    written += uid_.write(output);
    written += type_.write(output);
    written += codec_id_.write(output);
    if (!enabled_.is_default())
    {
        written += enabled_.write(output);
    }
    if (!forced_.is_default())
    {
        written += forced_.write(output);
    }
    if (!lacing_.is_default())
    {
        written += lacing_.write(output);
    }
    if (!min_cache_.is_default())
    {
        written += min_cache_.write(output);
    }
    if (!max_cache_.is_default())
    {
        written += max_cache_.write(output);
    }
    if (default_dur_ != 0)
    {
        written += default_dur_.write(output);
    }
    if (!timecode_scale_.is_default())
    {
        written += timecode_scale_.write(output);
    }
    if (!max_block_add_id_.is_default())
    {
        written += max_block_add_id_.write(output);
    }
    if (!name_.value().empty())
    {
        written += name_.write(output);
    }
    if (!codec_private_.value().empty())
    {
        written += codec_private_.write(output);
    }
    if (!codec_name_.value().empty())
    {
        written += codec_name_.write(output);
    }
    if (attachment_link_ != 0)
    {
        written += attachment_link_.write(output);
    }
    if (!decode_all_.is_default())
    {
        written += decode_all_.write(output);
    }
    BOOST_FOREACH(UIntElement overlay, overlays_)
    {
        written += overlay.write(output);
    }
    if (operation_)
    {
        written += tawara::ids::write(ids::TrackOperation, output);
        written += tawara::vint::write(operation_->size(), output);
        written += operation_->write(output);
    }
    return written;
}


std::streamsize TrackEntry::read_body(std::istream& input,
        std::streamsize size)
{
    // Reset to defaults
    reset();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    bool have_type(false);
    while (read_bytes < size)
    {
        UIntElement uid(ids::TrackOverlay, 0);
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::TrackNumber:
                read_bytes += number_.read(input);
                if (number_ == 0)
                {
                    throw ValueOutOfRange() << err_id(number_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::TrackUID:
                read_bytes += uid_.read(input);
                if (uid_ == 0)
                {
                    throw ValueOutOfRange() << err_id(uid_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::TrackType:
                read_bytes += type_.read(input);
                if (type_ > 254)
                {
                    throw ValueOutOfRange() << err_id(type_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                have_type = true;
                break;
            case ids::FlagEnabled:
                read_bytes += enabled_.read(input);
                if (enabled_ != 0 && enabled_ != 1)
                {
                    throw ValueOutOfRange() << err_id(enabled_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::FlagDefault:
                // Skip this element
                skip_read(input, false);
                break;
            case ids::FlagForced:
                read_bytes += forced_.read(input);
                if (forced_ != 0 && forced_ != 1)
                {
                    throw ValueOutOfRange() << err_id(forced_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::FlagLacing:
                read_bytes += lacing_.read(input);
                if (lacing_ != 0 && lacing_ != 1)
                {
                    throw ValueOutOfRange() << err_id(lacing_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::MinCache:
                read_bytes += min_cache_.read(input);
                break;
            case ids::MaxCache:
                read_bytes += max_cache_.read(input);
                break;
            case ids::DefaultDuration:
                read_bytes += default_dur_.read(input);
                if (default_dur_ == 0)
                {
                    throw ValueOutOfRange() << err_id(default_dur_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::TrackTimecodeScale:
                read_bytes += timecode_scale_.read(input);
                if (timecode_scale_ <= 0.0)
                {
                    throw ValueOutOfRange() << err_id(timecode_scale_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::MaxBlockAdditionID:
                read_bytes += max_block_add_id_.read(input);
                break;
            case ids::Name:
                read_bytes += name_.read(input);
                break;
            case ids::CodecID:
                read_bytes += codec_id_.read(input);
                if (codec_id_.value().empty())
                {
                    throw ValueOutOfRange() << err_id(codec_id_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::CodecPrivate:
                read_bytes += codec_private_.read(input);
                break;
            case ids::CodecName:
                read_bytes += codec_name_.read(input);
                break;
            case ids::AttachmentLink:
                read_bytes += attachment_link_.read(input);
                if (attachment_link_ == 0)
                {
                    throw ValueOutOfRange() << err_id(attachment_link_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::CodecDecodeAll:
                read_bytes += decode_all_.read(input);
                if (decode_all_ != 0 && decode_all_ != 1)
                {
                    throw ValueOutOfRange() << err_id(decode_all_.id()) <<
                        err_par_id(id_) << err_pos(input.tellg());
                }
                break;
            case ids::TrackOverlay:
                read_bytes += uid.read(input);
                overlays_.push_back(uid);
                break;
            case ids::TrackOperation:
                read_bytes += read_operation(input);
                break;
            default:
                throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                    // The cast here makes Apple's LLVM compiler happy
                    err_pos(static_cast<std::streamsize>(input.tellg()) -
                            id_res.second);
        };
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (number_ == 0)
    {
        throw MissingChild() << err_id(ids::TrackNumber) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (uid_ == 0)
    {
        throw MissingChild() << err_id(ids::TrackUID) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (!have_type)
    {
        throw MissingChild() << err_id(ids::TrackType) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (codec_id_.value().empty())
    {
        throw MissingChild() << err_id(ids::CodecID) << err_par_id(id_) <<
            err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

void TrackEntry::reset()
{
    number_ = 0;
    uid_ = 0;
    type_ = 0x70;
    enabled_ = enabled_.get_default();
    forced_ = forced_.get_default();
    lacing_ = lacing_.get_default();
    min_cache_ = min_cache_.get_default();
    max_cache_ = max_cache_.get_default();
    default_dur_ = 0;
    timecode_scale_ = timecode_scale_.get_default();
    max_block_add_id_ = max_block_add_id_.get_default();
    name_ = name_.get_default();
    codec_id_ = "";
    codec_private_ = std::vector<char>();
    codec_name_ = "";
    attachment_link_ = 0;
    decode_all_ = decode_all_.get_default();
    overlays_.clear();
    operation_.reset();
}


std::streamsize TrackEntry::read_operation(std::istream& input)
{
    std::streamsize read_bytes(0);
    vint::ReadResult op_size_res = vint::read(input);
    read_bytes += op_size_res.second;
    if (read_bytes == 0)
    {
        return 0;
        // An empty operation is legal.
    }
    ids::ReadResult op_id_res = ids::read(input);
    read_bytes += op_id_res.second;
    // Currently, only the TrackJoinBlocks operation is supported
    if (op_id_res.first != ids::TrackJoinBlocks)
    {
        throw InvalidChildID() << err_id(op_id_res.first) <<
            err_par_id(id_) << err_pos(input.tellg());
    }
    TrackOperationBase::Ptr op(new TrackJoinBlocks());
    read_bytes += op->read(input);
    operation_ = op;
    return read_bytes;
}

