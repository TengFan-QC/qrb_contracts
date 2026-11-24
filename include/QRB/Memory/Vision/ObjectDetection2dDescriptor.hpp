// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_OBJECT_DETECTION_2D_DESCRIPTOR_HPP
#define QRB_OBJECT_DETECTION_2D_DESCRIPTOR_HPP

#include "QRB/Memory/Vision/ObjectDetectionDescriptor.hpp"

#include <cstddef>
#include <cstdint>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/**
 * @struct Detection2dRaw_t
 * @brief POD representation of a single 2-D detection stored in the DMA buffer,
 *        matching ROS 2 vision_msgs/Detection2D.
 *
 * @param id           Null-terminated tracking identifier. May be empty.
 * @param bbox         2-D bounding box (pixels) surrounding the object.
 * @param resultCount  Number of valid entries in @c results.
 * @param results      Class hypotheses; only the first @c resultCount entries are valid.
 */
typedef DetectionRaw<BoundingBox2d_t> Detection2dRaw_t;

static_assert( std::is_trivially_copyable<Detection2dRaw_t>::value,
               "Detection2dRaw_t must be trivially copyable for DMA buffer storage" );

/**
 * @struct ObjectDetection2dDescriptor_t
 * @brief Descriptor for a robotics 2-D object detection buffer.
 *
 * Carries a list of 2-D detections packed into a DMA buffer as a contiguous array of
 * @c Detection2dRaw_t. The layout mirrors ROS 2 @c vision_msgs/Detection2DArray.
 * The stamp, frame, detection count and the other common members are inherited from
 * @c ObjectDetectionDescriptor_t.
 *
 * Buffer layout:
 * @code
 *   pBuf → [ Detection2dRaw_t[0] | Detection2dRaw_t[1] | ... | Detection2dRaw_t[objCount-1] ]
 * @endcode
 */
typedef struct ObjectDetection2dDescriptor : public ObjectDetectionDescriptor_t
{
public:
    ObjectDetection2dDescriptor() = default;
    ~ObjectDetection2dDescriptor() override = default;

    ObjectDetection2dDescriptor( const ObjectDetection2dDescriptor &other ) = default;
    ObjectDetection2dDescriptor( ObjectDetection2dDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from another @c ObjectDetection2dDescriptor_t.
     */
    ObjectDetection2dDescriptor &operator=( const ObjectDetection2dDescriptor &other ) = default;

    /**
     * @brief Move-assign from another @c ObjectDetection2dDescriptor_t.
     */
    ObjectDetection2dDescriptor &operator=( ObjectDetection2dDescriptor &&other ) = default;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; detection-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    ObjectDetection2dDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override
    {
        ObjectDetectionDescriptor_t::operator=( other );
        return *this;
    }

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     *
     * A @c ObjectDetection2dDescriptor_t source is copied entirely. Any other
     * @c BufferDescriptor_t, including an @c ObjectDetection3dDescriptor_t, only has its
     * base-class fields copied.
     */
    ObjectDetection2dDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override
    {
        if ( this != &other )
        {
            AssignFromBase<ObjectDetection2dDescriptor>( other );
        }
        return *this;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Returns a pointer to the @p index -th detection in the DMA buffer.
     *
     * No bounds checking is performed; the caller must ensure
     * @p index < @c objCount.
     *
     * @param index  Zero-based detection index.
     * @return Pointer to the requested @c Detection2dRaw_t, or @c nullptr if
     *         @c pBuf is @c nullptr.
     */
    Detection2dRaw_t *GetDetectionPtr( uint32_t index ) const
    {
        if ( pBuf == nullptr )
        {
            return nullptr;
        }
        return reinterpret_cast<Detection2dRaw_t *>( GetDataPtr() ) + index;
    }

    /**
     * @brief Returns the minimum DMA buffer size required to hold @p count detections.
     *
     * @param count  Number of detections to accommodate.
     * @return Required buffer size in bytes.
     */
    static size_t CalcBufferSize( uint32_t count )
    {
        return static_cast<size_t>( count ) * sizeof( Detection2dRaw_t );
    }

} ObjectDetection2dDescriptor_t;

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_OBJECT_DETECTION_2D_DESCRIPTOR_HPP
