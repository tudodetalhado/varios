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

#include <tawara/segment_info.h>

#include <algorithm>
#include <functional>
#include <tawara/exceptions.h>
#include <tawara/vint.h>

using namespace tawara;
using namespace std::placeholders;


///////////////////////////////////////////////////////////////////////////////
// Constructors and destructors
///////////////////////////////////////////////////////////////////////////////

SegmentInfo::SegmentInfo()
    : MasterElement(ids::Info),
    uid_(ids::SegmentUID, std::vector<char>()), have_uid_(false),
    seg_fn_(ids::SegmentFileName, ""), have_seg_fn_(false),
    prev_uid_(ids::PrevUID, std::vector<char>()), have_prev_uid_(false),
    prev_fn_(ids::PrevFileName, ""), have_prev_fn_(false),
    next_uid_(ids::NextUID, std::vector<char>()), have_next_uid_(false),
    next_fn_(ids::NextFileName, ""), have_next_fn_(false),
    seg_fam_(ids::SegmentFamily, std::vector<char>()), have_seg_fam_(false),
    tc_scale_(ids::TimecodeScale, 1000000, 1000000),
    duration_(ids::Duration, 1), have_duration_(false),
    date_(ids::DateUTC, 0), have_date_(false),
    title_(ids::Title, ""), have_title_(false),
    muxer_(ids::MuxingApp, ""), have_muxer_(false),
    writer_(ids::WritingApp, ""), have_writer_(false)
{
}


///////////////////////////////////////////////////////////////////////////////
// Accessors
///////////////////////////////////////////////////////////////////////////////

void SegmentInfo::uid(std::vector<char> const& uid)
{
    if (uid.empty())
    {
        uid_ = uid;
        have_uid_ = false;
    }
    else
    {
        std::vector<char>::const_iterator non_zero(std::find_if(uid.begin(),
                    uid.end(), std::bind2nd(std::not_equal_to<int>(), 0)));
        if (non_zero == uid.end())
        {
            throw ValueOutOfRange() << err_id(uid_.id()) <<
                err_par_id(id_);
        }
        if (uid.size() != 16)
        {
            throw ValueSizeOutOfRange() << err_id(ids::SegmentUID) <<
                err_par_id(id_);
        }

        uid_ = uid;
        have_uid_ = true;
    }
}


void SegmentInfo::filename(std::string const& filename)
{
    seg_fn_ = filename;
    if (filename.empty())
    {
        have_seg_fn_ = false;
    }
    else
    {
        have_seg_fn_ = true;
    }
}


void SegmentInfo::prev_uid(std::vector<char> const& uid)
{
    if (uid.empty())
    {
        prev_uid_ = uid;
        have_prev_uid_ = false;
    }
    else
    {
        std::vector<char>::const_iterator non_zero(std::find_if(uid.begin(),
                    uid.end(), std::bind2nd(std::not_equal_to<int>(), 0)));
        if (non_zero == uid.end())
        {
            throw ValueOutOfRange() << err_id(prev_uid_.id()) <<
                err_par_id(id_);
        }
        if (uid.size() != 16)
        {
            throw ValueSizeOutOfRange() << err_id(prev_uid_.id()) <<
                err_par_id(id_);
        }

        prev_uid_ = uid;
        have_prev_uid_ = true;
    }
}


void SegmentInfo::prev_filename(std::string const& filename)
{
    prev_fn_ = filename;
    if (filename.empty())
    {
        have_prev_fn_ = false;
    }
    else
    {
        have_prev_fn_ = true;
    }
}


void SegmentInfo::next_uid(std::vector<char> const& uid)
{
    if (uid.empty())
    {
        next_uid_ = uid;
        have_next_uid_ = false;
    }
    else
    {
        std::vector<char>::const_iterator non_zero(std::find_if(uid.begin(),
                    uid.end(), std::bind2nd(std::not_equal_to<int>(), 0)));
        if (non_zero == uid.end())
        {
            throw ValueOutOfRange() << err_id(next_uid_.id()) <<
                err_par_id(id_);
        }
        if (uid.size() != 16)
        {
            throw ValueSizeOutOfRange() << err_id(next_uid_.id()) <<
                err_par_id(id_);
        }

        next_uid_ = uid;
        have_next_uid_ = true;
    }
}


void SegmentInfo::next_filename(std::string const& filename)
{
    next_fn_ = filename;
    if (filename.empty())
    {
        have_next_fn_ = false;
    }
    else
    {
        have_next_fn_ = true;
    }
}


void SegmentInfo::segment_family(std::vector<char> const& segment_family)
{
    if (segment_family.empty())
    {
        seg_fam_ = segment_family;
        have_seg_fam_ = false;
    }
    else
    {
        std::vector<char>::const_iterator
            non_zero(std::find_if(segment_family.begin(), segment_family.end(),
                        std::bind2nd(std::not_equal_to<int>(), 0)));
        if (non_zero == segment_family.end())
        {
            throw ValueOutOfRange() << err_id(seg_fam_.id()) <<
                err_par_id(id_);
        }
        if (segment_family.size() != 16)
        {
            throw ValueSizeOutOfRange() << err_id(seg_fam_.id()) <<
                err_par_id(id_);
        }

        seg_fam_ = segment_family;
        have_seg_fam_ = true;
    }
}


void SegmentInfo::timecode_scale(uint64_t scale)
{
    if (scale == 0)
    {
        tc_scale_ = tc_scale_.get_default();
    }
    else
    {
        tc_scale_ = scale;
    }
}


void SegmentInfo::duration(double duration)
{
    if (duration <= 0)
    {
        throw ValueOutOfRange() << err_id(ids::Duration) <<
            err_id(ids::Info);
    }
    duration_ = duration;
    have_duration_ = true;
}


void SegmentInfo::date(int64_t date)
{
    date_ = date;
    have_date_ = true;
}


void SegmentInfo::title(std::string const& title)
{
    title_ = title;
    if (title.empty())
    {
        have_title_ = false;
    }
    else
    {
        have_title_ = true;
    }
}


void SegmentInfo::muxing_app(std::string const& muxing_app)
{
    muxer_ = muxing_app;
    if (muxing_app.empty())
    {
        have_muxer_ = false;
    }
    else
    {
        have_muxer_ = true;
    }
}


void SegmentInfo::writing_app(std::string const& writing_app)
{
    writer_ = writing_app;
    if (writing_app.empty())
    {
        have_writer_ = false;
    }
    else
    {
        have_writer_ = true;
    }
}


///////////////////////////////////////////////////////////////////////////////
// Element interface
///////////////////////////////////////////////////////////////////////////////

std::streamsize SegmentInfo::body_size() const
{
    std::streamsize result(tc_scale_.size());

    if (have_uid_)
    {
        result += uid_.size();
    }
    if (have_seg_fn_)
    {
        result += seg_fn_.size();
    }
    if (have_prev_uid_)
    {
        result += prev_uid_.size();
    }
    if (have_prev_fn_)
    {
        result += prev_fn_.size();
    }
    if (have_next_uid_)
    {
        result += next_uid_.size();
    }
    if (have_next_fn_)
    {
        result += next_fn_.size();
    }
    if (have_seg_fam_)
    {
        result += seg_fam_.size();
    }
    if (have_duration_)
    {
        result += duration_.size();
    }
    if (have_date_)
    {
        result += date_.size();
    }
    if (have_title_)
    {
        result += title_.size();
    }
    if (have_muxer_)
    {
        result += muxer_.size();
    }
    if (have_writer_)
    {
        result += writer_.size();
    }

    return result;
}


std::streamsize SegmentInfo::write_body(std::ostream& output)
{
    std::streamsize written(0);

    // The spec may say that the TimecodeScale comes later, but in EBML it
    // doesn't actually matter.
    written += tc_scale_.write(output);
    if (have_uid_)
    {
        written += uid_.write(output);
    }
    if (have_seg_fn_)
    {
        written += seg_fn_.write(output);
    }
    if (have_prev_uid_)
    {
        written += prev_uid_.write(output);
    }
    if (have_prev_fn_)
    {
        written += prev_fn_.write(output);
    }
    if (have_next_uid_)
    {
        written += next_uid_.write(output);
    }
    if (have_next_fn_)
    {
        written += next_fn_.write(output);
    }
    if (have_seg_fam_)
    {
        written += seg_fam_.write(output);
    }
    if (have_duration_)
    {
        written += duration_.write(output);
    }
    if (have_date_)
    {
        written += date_.write(output);
    }
    if (have_title_)
    {
        written += title_.write(output);
    }
    if (have_muxer_)
    {
        written += muxer_.write(output);
    }
    if (have_writer_)
    {
        written += writer_.write(output);
    }
    return written;
}


std::streamsize SegmentInfo::read_body(std::istream& input,
        std::streamsize size)
{
    // Reset to defaults
    reset();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::SegmentUID:
                read_bytes += uid_.read(input);
                have_uid_ = true;
                break;
            case ids::SegmentFileName:
                read_bytes += seg_fn_.read(input);
                have_seg_fn_ = true;
                break;
            case ids::PrevUID:
                read_bytes += prev_uid_.read(input);
                have_prev_uid_ = true;
                break;
            case ids::PrevFileName:
                read_bytes += prev_fn_.read(input);
                have_prev_fn_ = true;
                break;
            case ids::NextUID:
                read_bytes += next_uid_.read(input);
                have_next_uid_ = true;
                break;
            case ids::NextFileName:
                read_bytes += next_fn_.read(input);
                have_next_fn_ = true;
                break;
            case ids::SegmentFamily:
                read_bytes += seg_fam_.read(input);
                have_seg_fam_ = true;
                break;
            case ids::TimecodeScale:
                read_bytes += tc_scale_.read(input);
                break;
            case ids::Duration:
                read_bytes += duration_.read(input);
                have_duration_ = true;
                break;
            case ids::DateUTC:
                read_bytes += date_.read(input);
                have_date_ = true;
                break;
            case ids::Title:
                read_bytes += title_.read(input);
                have_title_ = true;
                break;
            case ids::MuxingApp:
                read_bytes += muxer_.read(input);
                have_muxer_ = true;
                break;
            case ids::WritingApp:
                read_bytes += writer_.read(input);
                have_writer_ = true;
                break;
            default:
                throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                    // The cast here makes Apple's LLVM compiler happy
                    err_pos(static_cast<std::streamsize>(input.tellg()) -
                            id_res.second);
        }
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////////////////////////////////////////////

void SegmentInfo::reset()
{
    uid_ = std::vector<char>();
    have_uid_ = false;
    seg_fn_ = "";
    have_seg_fn_ = false;
    prev_uid_ = std::vector<char>();
    have_prev_uid_ = false;
    prev_fn_ = "";
    have_prev_fn_ = false;
    next_uid_ = std::vector<char>();
    have_next_uid_ = false;
    next_fn_ = "";
    have_next_fn_ = false;
    seg_fam_ = std::vector<char>();
    have_seg_fam_ = false;
    tc_scale_ = 1000000;
    duration_ = 1;
    have_duration_ = false;
    date_ = 0;
    have_date_ = false;
    title_ = "";
    have_title_ = false;
    muxer_ = "";
    have_muxer_ = false;
    writer_ = "";
    have_writer_ = false;
}

