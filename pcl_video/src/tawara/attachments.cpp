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

#include <tawara/attachments.h>

#include <boost/foreach.hpp>
#include <tawara/el_ids.h>
#include <tawara/exceptions.h>

using namespace tawara;

///////////////////////////////////////////////////////////////////////////////
// AttachedFile constructors and destructors
///////////////////////////////////////////////////////////////////////////////

AttachedFile::AttachedFile()
    : MasterElement(ids::AttachedFile), desc_(ids::FileDescription, ""),
    name_(ids::FileName, ""), mime_(ids::FileMimeType, ""),
    uid_(ids::FileUID, 1)
{
}


AttachedFile::AttachedFile(std::string const& name,
        std::string const& mime_type, FileData::Ptr data, uint64_t uid)
    : MasterElement(ids::AttachedFile), desc_(ids::FileDescription, ""),
    name_(ids::FileName, name),
    mime_(ids::FileMimeType, mime_type),
    data_(data), uid_(ids::FileUID, uid)
{
    if (uid_ == 0)
    {
        throw ValueOutOfRange() << err_id(ids::FileUID) << err_par_id(id_);
    }

    if (!data_ || data_->value().empty())
    {
        throw NoAttachedData();
    }
}


void AttachedFile::uid(uint64_t uid)
{
    if (uid == 0)
    {
        throw ValueOutOfRange() << err_id(ids::FileUID) << err_par_id(id_);
    }
    uid_ = uid;
}


void AttachedFile::data(FileData::Ptr& data)
{
    if (!data || data->value().empty())
    {
        throw NoAttachedData();
    }
    data_ = data;
}


///////////////////////////////////////////////////////////////////////////////
// AttachedFile operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(AttachedFile const& lhs, AttachedFile const& rhs)
{
    bool data_eq(false);
    if (lhs.data_ && rhs.data_)
    {
        data_eq = lhs.data_ == rhs.data_;
    }
    else if (!lhs.data_ && !rhs.data_)
    {
        data_eq = true;
    }
    return lhs.desc_ == rhs.desc_ &&
        lhs.name_ == rhs.name_ &&
        lhs.mime_ == rhs.mime_ &&
        lhs.uid_ == rhs.uid_ &&
        data_eq;
}


///////////////////////////////////////////////////////////////////////////////
// AttachedFile Element interface implementation
///////////////////////////////////////////////////////////////////////////////

std::streamsize AttachedFile::body_size() const
{
    std::streamsize size(0);

    size = name_.size() + mime_.size() + data_->size() + uid_.size();
    if (!desc_.value().empty())
    {
        size += desc_.size();
    }
    return size;
}


std::streamsize AttachedFile::write_body(std::ostream& output)
{
    assert(data_);
    assert(!data_->value().empty());
    assert(uid_ != 0);

    std::streamsize written(0);

    if (!desc_.value().empty())
    {
        written += desc_.write(output);
    }
    written += name_.write(output);
    written += mime_.write(output);
    written += data_->write(output);
    written += uid_.write(output);
    return written;
}


std::streamsize AttachedFile::read_body(std::istream& input,
        std::streamsize size)
{
    // Reset to defaults
    reset();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    bool have_name(false), have_mime(false), have_data(false), have_uid(false);
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        switch(id)
        {
            case ids::FileDescription:
                read_bytes += desc_.read(input);
                break;
            case ids::FileName:
                read_bytes += name_.read(input);
                have_name = true;
                break;
            case ids::FileMimeType:
                read_bytes += mime_.read(input);
                have_mime = true;
                break;
            case ids::FileData:
                data_.reset(new FileData(std::vector<char>()));
                read_bytes += data_->read(input);
                if (!data_ || data_->value().empty())
                {
                    throw NoAttachedData();
                }
                have_data = true;
                break;
            case ids::FileUID:
                read_bytes += uid_.read(input);
                if (uid_ == 0)
                {
                    throw ValueOutOfRange() << err_id(ids::FileUID) <<
                        err_par_id(id_) << err_pos(offset_);
                }
                have_uid = true;
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
    if (!have_name)
    {
        throw MissingChild() << err_id(ids::FileName) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (!have_mime)
    {
        throw MissingChild() << err_id(ids::FileMimeType) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (!have_data)
    {
        throw MissingChild() << err_id(ids::FileData) << err_par_id(id_) <<
            err_pos(offset_);
    }
    if (!have_uid)
    {
        throw MissingChild() << err_id(ids::FileUID) << err_par_id(id_) <<
            err_pos(offset_);
    }

    return read_bytes;
}


///////////////////////////////////////////////////////////////////////////////
// AttachedFile private functions
///////////////////////////////////////////////////////////////////////////////

void AttachedFile::reset()
{
    desc_ = "";
    name_ = "";
    mime_ = "";
    data_.reset(new FileData(std::vector<char>()));
    uid_ = 0;
}


///////////////////////////////////////////////////////////////////////////////
// Attachments constructors and destructors
///////////////////////////////////////////////////////////////////////////////

Attachments::Attachments()
    : MasterElement(ids::Attachments)
{
}


///////////////////////////////////////////////////////////////////////////////
// Attachments operators
///////////////////////////////////////////////////////////////////////////////

bool tawara::operator==(Attachments const& lhs, Attachments const& rhs)
{
    return lhs.files_ == rhs.files_;
}


///////////////////////////////////////////////////////////////////////////////
// Attachments Element interface implementation
///////////////////////////////////////////////////////////////////////////////

std::streamsize Attachments::body_size() const
{
    std::streamsize size(0);

    BOOST_FOREACH(AttachedFile const& file, files_)
    {
        size += file.size();
    }
    return size;
}


std::streamsize Attachments::write_body(std::ostream& output)
{
    if (files_.empty())
    {
        throw NoAttachments();
    }

    std::streamsize written(0);

    BOOST_FOREACH(AttachedFile& file, files_)
    {
        written += file.write(output);
    }
    return written;
}


std::streamsize Attachments::read_body(std::istream& input,
        std::streamsize size)
{
    files_.clear();

    std::streamsize read_bytes(0);
    // Read elements until the body is exhausted
    while (read_bytes < size)
    {
        // Read the ID
        ids::ReadResult id_res = ids::read(input);
        ids::ID id(id_res.first);
        read_bytes += id_res.second;
        if (id != ids::AttachedFile)
        {
            throw InvalidChildID() << err_id(id) << err_par_id(id_) <<
                // The cast here makes Apple's LLVM compiler happy
                err_pos(static_cast<std::streamsize>(input.tellg()) -
                        id_res.second);
        }
        AttachedFile file;
        read_bytes += file.read(input);
        files_.push_back(file);
    }
    if (read_bytes != size)
    {
        // Read more than was specified by the body size value
        throw BadBodySize() << err_id(id_) << err_el_size(size) <<
            err_pos(offset_);
    }
    if (files_.empty())
    {
        throw NoAttachments();
    }

    return read_bytes;
}

