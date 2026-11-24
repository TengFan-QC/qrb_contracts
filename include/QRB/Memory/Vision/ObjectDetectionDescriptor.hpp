// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_OBJECT_DETECTION_DESCRIPTOR_HPP
#define QRB_OBJECT_DETECTION_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/** @brief Maximum length of a class ID string (including null terminator). */
#ifndef QRB_DETECTION_MAX_CLASS_ID_LEN
#define QRB_DETECTION_MAX_CLASS_ID_LEN 64
#endif

/** @brief Maximum length of a detection tracking ID string (including null terminator). */
#ifndef QRB_DETECTION_MAX_ID_LEN
#define QRB_DETECTION_MAX_ID_LEN 64
#endif

/** @brief Maximum number of hypothesis results per detection. */
#ifndef QRB_DETECTION_MAX_RESULTS
#define QRB_DETECTION_MAX_RESULTS 8
#endif

/**
 * @struct ObjectHypothesisRaw_t
 * @brief POD representation of a single class hypothesis stored in the DMA buffer,
 *        matching ROS 2 vision_msgs/ObjectHypothesisWithPose.
 *
 * Shared by the 2-D and 3-D detection descriptors. The nested @c hypothesis of the ROS
 * message (class ID and score) is flattened into this struct.
 *
 * @param classId     Null-terminated class identifier string.
 * @param score       Confidence score in [0, 1].
 * @param pose        6-DOF pose with covariance of the hypothesised object.
 */
typedef struct ObjectHypothesisRaw
{
    char                 classId[QRB_DETECTION_MAX_CLASS_ID_LEN];
    double               score;
    PoseWithCovariance_t pose;
} ObjectHypothesisRaw_t;

static_assert( std::is_trivially_copyable<ObjectHypothesisRaw_t>::value,
               "ObjectHypothesisRaw_t must be trivially copyable for DMA buffer storage" );

/**
 * @struct DetectionRaw
 * @brief POD record of a single detection stored in the DMA buffer, matching ROS 2
 *        vision_msgs/Detection2D and vision_msgs/Detection3D without their @c header.
 *
 * The 2-D and 3-D records differ only in the bounding box type, so they share this layout;
 * see @c Detection2dRaw_t and @c Detection3dRaw_t. The @c header of the ROS message is carried
 * once per buffer by @c ObjectDetectionDescriptor_t::timestampNs and @c frameId.
 *
 * @tparam BoundingBoxT  Bounding box type of the detection.
 *
 * @param id           Null-terminated tracking identifier. May be empty.
 * @param bbox         Bounding box surrounding the object.
 * @param resultCount  Number of valid entries in @c results.
 * @param results      Class hypotheses; only the first @c resultCount entries are valid.
 */
template <typename BoundingBoxT>
struct DetectionRaw
{
    char                  id[QRB_DETECTION_MAX_ID_LEN];
    BoundingBoxT          bbox;
    uint32_t              resultCount;
    ObjectHypothesisRaw_t results[QRB_DETECTION_MAX_RESULTS];
};

/**
 * @struct ObjectDetectionDescriptor_t
 * @brief Common base of the robotics object detection buffer descriptors.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a list of detections packed into a DMA
 * buffer as a contiguous array of fixed-size POD records. This base holds everything the
 * 2-D and 3-D descriptors share: the buffer type tag, the stamp, the frame, the detection
 * count and the assignment logic. The record layout, and with it the accessors, belong to
 * the concrete descriptors:
 *   - @c ObjectDetection2dDescriptor_t  mirrors @c vision_msgs/Detection2DArray
 *   - @c ObjectDetection3dDescriptor_t  mirrors @c vision_msgs/Detection3DArray
 *
 * Buffer layout (@c Record is the @c Detection2dRaw_t or @c Detection3dRaw_t of the
 * concrete descriptor):
 * @code
 *   pBuf → [ Record[0] | Record[1] | ... | Record[objCount-1] ]
 * @endcode
 *
 * The class is abstract: instantiate one of the concrete descriptors.
 *
 * @note All detection descriptors share the buffer type tag @c QC_BUFFER_TYPE_CUSTOM_2, so
 * tell them apart by their C++ type (@c dynamic_cast to a concrete descriptor), never by
 * @c type. A @c dynamic_cast to this base matches both dimensions.
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_2 for object detection).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid detection data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs  Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                     matching ROS 2 std_msgs/Header.stamp.
 * @param frameId      Coordinate frame in which all detections are expressed
 *                     (e.g. "camera_link", "lidar_link"),
 *                     matching ROS 2 std_msgs/Header.frame_id. A @c FrameId_t, a fixed-capacity
 *                     string of at most QRB_FRAME_ID_MAX_LEN characters.
 * @param objCount     Number of valid detection records in the DMA buffer.
 *
 * @note The descriptor has no dynamic length member (no std::string / std::vector), so copying it
 * never allocates, as required for buffer descriptors. The detections themselves are stored in the
 * DMA buffer as fixed-size POD structs.
 */
typedef struct ObjectDetectionDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    ~ObjectDetectionDescriptor() override = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; detection-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    ObjectDetectionDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     *
     * Pure virtual: what is copied depends on the concrete descriptor, which only accepts
     * its own kind as a detection descriptor. See @c AssignFromBase.
     */
    ObjectDetectionDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override = 0;

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t  timestampNs; /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t frameId;     /**< Coordinate frame identifier. */
    uint32_t  objCount;    /**< Number of valid detection records in the DMA buffer. */

protected:
    ObjectDetectionDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 ), objCount( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_2;
    }

    ObjectDetectionDescriptor( const ObjectDetectionDescriptor &other ) = default;
    ObjectDetectionDescriptor( ObjectDetectionDescriptor &&other ) = default;

    /**
     * @brief Copy-assign from another @c ObjectDetectionDescriptor_t.
     *
     * Protected so that the detection-specific fields cannot be copied between two different
     * concrete descriptors, which would label 2-D records as 3-D. The concrete descriptors expose
     * the copy assignment of their own type; assigning any other descriptor goes through
     * @c operator=(const QCBufferDescriptorBase_t&) and copies the buffer fields only.
     */
    ObjectDetectionDescriptor &operator=( const ObjectDetectionDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c ObjectDetectionDescriptor_t. See the copy assignment.
     */
    ObjectDetectionDescriptor &operator=( ObjectDetectionDescriptor &&other ) noexcept;

    /**
     * @brief Shared body of the @c QCBufferDescriptorBase_t assignment of the concrete descriptors.
     *
     * @tparam DescriptorT  The concrete descriptor that is being assigned to.
     *
     * - @p other is a @c DescriptorT: copy everything.
     * - @p other is any other @c BufferDescriptor_t, including the other detection dimension:
     *   copy the buffer fields only, so a buffer of 2-D records is never labelled as 3-D.
     * - otherwise: defer to the base-class assignment.
     */
    template <typename DescriptorT>
    void AssignFromBase( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
    {
        const DescriptorT *pDetDesc = dynamic_cast<const DescriptorT *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pDetDesc )
        {
            ObjectDetectionDescriptor::operator=( static_cast<const ObjectDetectionDescriptor &>( *pDetDesc ) );
        }
        else if ( nullptr != pBufDesc )
        {
            ObjectDetectionDescriptor::operator=( *pBufDesc );
        }
        else
        {
            QC::Memory::BufferDescriptor_t::operator=( other );
        }
    }

} ObjectDetectionDescriptor_t;

inline ObjectDetectionDescriptor &ObjectDetectionDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_2;
    return *this;
}

inline ObjectDetectionDescriptor &ObjectDetectionDescriptor::operator=( const ObjectDetectionDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_2;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->objCount = other.objCount;
    }
    return *this;
}

inline ObjectDetectionDescriptor &ObjectDetectionDescriptor::operator=( ObjectDetectionDescriptor &&other ) noexcept
{
    // No heap buffer to steal: moving is a copy of the fixed-size members.
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_2;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->objCount = other.objCount;
    }
    return *this;
}

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_OBJECT_DETECTION_DESCRIPTOR_HPP
