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

#ifndef HRI_PHYSIO_DEV_DEVICE_INTERFACE_H
#define HRI_PHYSIO_DEV_DEVICE_INTERFACE_H

#include <iostream>
#include <memory>
#include <mutex>

#include <HriPhysio/Core/ringBuffer.h>
#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Dev {
        class DeviceInterface;

        template <class T> 
        class DeviceTemplate;
    }
}

class hriPhysio::Dev::DeviceInterface {
public:
    virtual ~DeviceInterface();
    virtual void read()  = 0;
    virtual void write() = 0;
    virtual void deviceInit() = 0;
};

template <class T> 
class hriPhysio::Dev::DeviceTemplate : public hriPhysio::Dev::DeviceInterface {
protected:
    /* ============================================================================
    **  Member Variables that get Inherited by Child class.
    ** ============================================================================ */ 

    //-- The main container of the data members.
    hriPhysio::Core::RingBuffer<T> buffer;

    //std::unique_ptr<int[]> intermediary;

public:
    
    DeviceTemplate();

    void read();

    virtual void deviceInit() = 0;


protected:
    
    virtual void write() = 0;
    

};

#endif /* HRI_PHYSIO_DEV_DEVICE_INTERFACE_H */
