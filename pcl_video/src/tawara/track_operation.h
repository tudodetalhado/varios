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

#if !defined(TAWARA_TRACK_OPERATION_H_)
#define TAWARA_TRACK_OPERATION_H_

#include <boost/operators.hpp>
#include <boost/smart_ptr.hpp>
#include <tawara/el_ids.h>
#include <tawara/master_element.h>
#include <tawara/uint_element.h>
#include <tawara/win_dll.h>
#include <vector>

/// \addtogroup elements Elements
/// @{

namespace tawara
{
    /** \brief Abstract base class for individual track operations.
     *
     * All track operations must inherit from this base class.
     */
    class TAWARA_EXPORT TrackOperationBase : public MasterElement
    {
        public:
            /// \brief Constructor - this must be called to set the Class ID.
            TrackOperationBase(ids::ID id)
                : MasterElement(id)
            {}

            /// \brief Desctructor.
            virtual ~TrackOperationBase() {}

            /** \brief Get the type of operation to be performed.
             *
             * \return A string describing the operation.
             */
            virtual std::string type() const = 0;

            /// \brief Base type of a track operation pointer.
            typedef boost::shared_ptr<TrackOperationBase> Ptr;
    }; // class TrackOperationBase


    /** \brief JoinBlocks track operation.
     *
     * This track operation is used to join the blocks of the source tracks
     * into a single virtual track. Usually it is used to join tracks that are
     * distinct in time. Joining tracks overlapped in time will lead to
     * interleaved blocks of data at best and undefined results at worst, and
     * should be avoided.
     */
    class TAWARA_EXPORT TrackJoinBlocks : public TrackOperationBase,
            public boost::equality_comparable<TrackJoinBlocks>
    {
        public:
            /// \brief Construct a new JoinBlocks operation.
            TrackJoinBlocks();

            /// \brief Destructor.
            ~TrackJoinBlocks() {}

            /// \brief Get the type of operation to be performed.
            std::string type() const
            {
                return "joinblocks";
            }

            /** \brief Append a new UID to this operation.
             *
             * \param[in] uid The UID to append.
             * \throw ValueOutOfRange if a zero-value UID is appended.
             */
            void append(uint64_t uid);

            /** \brief Remove a UID.
             *
             * \param[in] pos The position of the UID to remove.
             * \return The removed UID.
             */
            uint64_t remove(unsigned int pos);

            /** \brief Const subscript operator.
             *
             * Gets the UID at the specified position.
             */
            uint64_t operator[](unsigned int pos) const;

            /// \brief Get the number of UIDs stored.
            unsigned int count() const { return uids_.size(); }

            /// TODO: Proper vector model for UIDs.

            /** \brief Element body writing.
             *
             * \throw ValueOutOfRange if a zero-value UID is written.
             */
            virtual std::streamsize write_body(std::ostream& output);

            /// \brief Equality operator.
            friend bool operator==(TrackJoinBlocks const& lhs,
                    TrackJoinBlocks const& rhs);

        protected:
            std::vector<UIntElement> uids_;

            /// \brief Get the size of the body of this element.
            virtual std::streamsize body_size() const;

            /// \brief Element body loading.
            virtual std::streamsize read_body(std::istream& input,
                    std::streamsize size);
    }; // class TrackJoinBlocks


    /// Equality operator for TrackJoinBlocks.
    inline bool operator==(TrackJoinBlocks const& lhs,
            TrackJoinBlocks const& rhs)
    {
        return lhs.uids_ == rhs.uids_;
    }
}; // namespace tawara

/// @}
// group elements

#endif // TAWARA_TRACK_OPERATION_H_

