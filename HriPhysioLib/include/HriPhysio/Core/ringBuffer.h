/* ================================================================================
 * Copyright: (C) 2020, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Core {
        template <class T> 
        class RingBuffer;
    }
}

template <class T> 
class hriPhysio::Core::RingBuffer {
private:
    /* ============================================================================
    **  Member Variables.
    ** ============================================================================ */ 

    //-- The main container of the data members.
    std::unique_ptr<T[]> buffer;

    //-- Meta-data for controlling the container.
    std::size_t buffer_length;
    std::size_t buffer_head;
    std::size_t buffer_tail;
    std::size_t buffer_size;

    //-- Mutex for ensuring atomicity.
    std::mutex lock;

    //-- Enable/Disable buffer warnings.
    bool warnings = false;


public:
    /* ============================================================================
    **  Main Constructor.
    ** 
    ** @param length    Number of indices to allocate. [Optional arg]
    ** ============================================================================ */
    RingBuffer();
    RingBuffer(const std::size_t length);


    /* ============================================================================
    **  Main Destructor.
    ** ============================================================================ */
    ~RingBuffer();


    /* ============================================================================
    **  Set Warnings to the passed in value (default is false).
    **
    ** @param value    Enable/Disable warnings.
    ** ============================================================================ */
   void setWarnings(bool value);


    /* ============================================================================
    **  Resize the internal buffer to the specified length.
    **    Note: Destroys any data left in the buffer.
    **
    ** @param length    Size of buffer to allocate.
    ** ============================================================================ */
    void resize(const std::size_t length);
    

    /* ============================================================================
    **  Clear the buffer and set all members to the default value of type.
    ** ============================================================================ */
    void clear();


    /* ============================================================================
    **  Enqueue a single piece of data into the buffer.
    **
    ** @param item    Reference to a single data to insert into the buffer.
    **
    ** @return Success/Failure of the insertion.
    ** ============================================================================ */
    bool enqueue(const T& item);


    /* ============================================================================
    **  Enqueue multiple pieces of data into the buffer.
    **
    ** @param items     Array of data to insert into the buffer.
    ** @param length    Length of the array attempting to insert.
    **
    ** @return Success/Failure of the insertion.
    ** ============================================================================ */
    bool enqueue(const T* items, const std::size_t length);


    /* ============================================================================
    **  Dequeue a single piece of data from the buffer.
    **
    ** @param item    Reference to a single data to pop from the buffer.
    **
    ** @return Success/Failure of the withdraw.
    ** ============================================================================ */
    bool dequeue(T& item);


    /* ============================================================================
    **  Dequeue multiple pieces of data from the buffer.
    **
    ** @param items     Array of data to write to from the buffer.
    ** @param length    Length of the array attempting to withdraw.
    **
    ** @return Success/Failure of the withdraw.
    ** ============================================================================ */
    bool dequeue(T* items, const std::size_t length);


    /* ============================================================================
    **  Copy a single piece of data from the buffer without popping them.
    **
    ** @param item    Reference to a single data to copy from the buffer.
    **
    ** @return Success/Failure of the copy.
    ** ============================================================================ */
    bool front(T& item);


    /* ============================================================================
    **  Copy multiple pieces of data from the buffer without popping them.
    **
    ** @param items     Array of data to copy to from the buffer.
    ** @param length    Length of the array attempting to copy.
    **
    ** @return Success/Failure of the copy.
    ** ============================================================================ */
    bool front(T* items, const std::size_t length);


    /* ============================================================================
    **  Get a pointer to the first element.
    **    Warning: Once you have the pointer does not guarantee atomicity!!
    **  
    ** @return The pointer to the first element (or NULL if unallocated).
    ** ============================================================================ */
    inline T* data();
    inline const T* data() const;


    /* ============================================================================
    **  Method to get if the buffer has data stored.
    **
    ** @return true if buffer is empty, false otherwise.
    ** ============================================================================ */
    bool empty() const;


    /* ============================================================================
    **  Method to get if the buffer is full.
    **
    ** @return true if buffer is full, false otherwise.
    ** ============================================================================ */
    bool full() const;


    /* ============================================================================
    **  Method to get the current number of elements stored in the buffer.
    **
    ** @return number of elements stored.
    ** ============================================================================ */
    std::size_t size() const;


    /* ============================================================================
    **  Method to get the max number of elements the buffer can store.
    **
    ** @return number of elements that can be stored.
    ** ============================================================================ */
    std::size_t length() const;


private:
    /* ============================================================================
    **  Private method to handle allocating and changing internal buffer.
    **
    ** @param length    Size of buffer to allocate.
    ** ============================================================================ */
    void bufferInit(const std::size_t length);
};

#endif /* RING_BUFFER_H */
