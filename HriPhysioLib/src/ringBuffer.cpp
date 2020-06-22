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

#include <HriPhysio/Core/ringBuffer.h>

using namespace hriPhysio::Core;


template <class T> 
RingBuffer<T>::RingBuffer() : 
    buffer_length(0),
    buffer_head(0),
    buffer_tail(0),
    buffer_size(0) {
    
    bufferInit(buffer_length);
}


template <class T> 
RingBuffer<T>::RingBuffer(const std::size_t length) : 
    buffer_length(length),
    buffer_head(0),
    buffer_tail(0),
    buffer_size(0) {
    
    bufferInit(buffer_length);
}


template <class T> 
RingBuffer<T>::~RingBuffer() {
    
    //-- Delete managed data.
    buffer.reset();
}


template <class T>
void RingBuffer<T>::setWarnings(bool value) {
    
    //-- Set the warnings flag.
    warnings = value;

    return;
}


template <class T> 
void RingBuffer<T>::resize(const std::size_t length) {

    //-- Update length, Update buffer.
    buffer_length = length;
    bufferInit(buffer_length);

    return;
}


template <class T> 
void RingBuffer<T>::clear() {

    //-- Create a ``zeroed`` instance of template type.
    T emptyVar;

    //-- Clear out the data.
    for (std::size_t idx = 0; idx < buffer_length; ++idx) {
        buffer[idx] = emptyVar;
    }

    //-- Set the meta-data to default.
    buffer_size = 0;
    buffer_head = 0;
    buffer_tail = 0;

    return; 
}


template <class T> 
bool RingBuffer<T>::enqueue(const T& item) {
    
    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If buffer space is not allocated, exit.
    if (buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] No buffer allocated to insert into." << std::endl;            
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- If buffer is full, overwrite old data.
    if (full()) {
        
        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Buffer Overflow!! Overwriting old data." << std::endl;
        }

        //-- ``Delete`` old data.
        buffer_head = (buffer_head + 1) % buffer_length;
        --buffer_size;
    }

    //-- Insert item at the ``back``.
    buffer[buffer_tail] = item;
    
    //-- Update tail and size.
    buffer_tail = (buffer_tail + 1) % buffer_length;
    ++buffer_size;

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
bool RingBuffer<T>::enqueue(const T* items, const std::size_t length) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If length of items exceeds allocated space, exit.
    if (length > buffer_length || buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Not enough buffer space allocated to insert into." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- If buffer is full, overwrite old data.
    if (buffer_size + length >= buffer_length) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Buffer Overflow!! Overwriting old data." << std::endl;
        }

        //-- ``Delete`` old data.
        buffer_head  = (buffer_head + length) % buffer_length;
        buffer_size -= length;
    }

    //-- Insert the items to the ``back``.
    for (std::size_t idx = 0; idx < length; ++idx) {

        //-- Copy the item.
        buffer[buffer_tail] = items[idx];

        //-- Update tail and size.
        buffer_tail = (buffer_tail + 1) % buffer_length;
        ++buffer_size;
    }

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
bool RingBuffer<T>::dequeue(T& item) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If buffer space is not allocated, exit.
    if (buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] No buffer allocated to pop from." << std::endl;            
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- If buffer is empty, can't pop.
    if (empty()) {
        
        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Buffer Empty!! Cannot pop." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- Copy item at the ``front``.
    item = buffer[buffer_head];

    //-- Update head and size.
    buffer_head = (buffer_head + 1) % buffer_length;
    --buffer_size;

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
bool RingBuffer<T>::dequeue(T* items, const std::size_t length) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If length of items exceeds allocated space, exit.
    if (length > buffer_length || buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Length requested exceeds available buffer space." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }
    
    //-- If requesting more items than what's available, exit.
    if (length > buffer_size) {
        
        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Not enough items in the buffer to satisfy requested length." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }


    //-- Copy items at the ``front``.
    for (std::size_t idx = 0; idx < length; ++idx) {

        //-- Copy the item.
        items[idx] = buffer[buffer_head];

        //-- Update head and size.
        buffer_head = (buffer_head + 1) % buffer_length;
        --buffer_size;
    }

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
bool RingBuffer<T>::front(T& item) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If buffer space is not allocated, exit.
    if (buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] No buffer allocated to copy from." << std::endl;            
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- If buffer is empty, can't copy.
    if (empty()) {
        
        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Buffer Empty!! Cannot copy." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }
    
    //-- Copy item at the ``front`` without changing head or size.
    item = buffer[buffer_head];

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
bool RingBuffer<T>::front(T* items, const std::size_t length) {

    //-- Lock the mutex to ensure read/write atomicity.
    lock.lock();

    //-- If length of items exceeds allocated space, exit.
    if (length > buffer_length || buffer_length == 0) {

        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Length requested exceeds available buffer space." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- If requesting more items than what's available, exit.
    if (length > buffer_size) {
        
        //-- Throw a warning if enabled.
        if (warnings) {
            std::cerr << "[DEBUG] Not enough items in the buffer to satisfy requested length." << std::endl;
        }

        //-- Unlock the mutex and return.
        lock.unlock();
        return false;
    }

    //-- Make a copy of the real head.
    std::size_t temp_head = buffer_head;

    //-- Copy items at the ``front``.
    for (std::size_t idx = 0; idx < length; ++idx) {

        //-- Copy the item.
        items[idx] = buffer[temp_head];

        //-- Update temporary head without touching size.
        temp_head = (temp_head + 1) % buffer_length;
    }

    //-- Unlock the mutex and return.
    lock.unlock();
    return true;
}


template <class T> 
inline T* RingBuffer<T>::data() {
    //-- get a pointer to the unique_ptr if allocated.
    return (buffer_length != 0) ? buffer.get() : 0 /* NULL */;
}


template <class T> 
inline const T* RingBuffer<T>::data() const {
    //-- get a pointer to the unique_ptr if allocated.
    return (buffer_length != 0) ? buffer.get() : 0 /* NULL */;
}


template <class T> 
bool RingBuffer<T>::empty() const {
    return buffer_size == 0;
}


template <class T> 
bool RingBuffer<T>::full() const {
    //-- Should never actually be larger, but
    //--   better safe than 3am debug sessions.
    return buffer_size >= buffer_length;
}


template <class T> 
std::size_t RingBuffer<T>::size() const {
    return buffer_size;
}


template <class T> 
std::size_t RingBuffer<T>::length() const {
    return buffer_length;
}


template <class T>
void RingBuffer<T>::bufferInit(const std::size_t length) {
    //-- Delete managed data and initiallize new sized array.
    buffer.reset(new T[ length ]);
}


//--
//-- Important Note!!
//--
//-- Putting  definitions of  a template class  outside of  the 
//-- header  file requires  us  to explicitly  state what  type 
//-- parameters are supported before hand. If this ever changes 
//-- move these definitions into the hearder file.
//--
//-- https://stackoverflow.com/questions/1022623/
//--

template class RingBuffer<int>;
template class RingBuffer<long>;
template class RingBuffer<double>;
template class RingBuffer<float>;
template class RingBuffer<std::size_t>;
