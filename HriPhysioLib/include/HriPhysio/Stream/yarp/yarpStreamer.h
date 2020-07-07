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

#ifndef HRI_PHYSIO_STREAM_YARP_STREAMER_H
#define HRI_PHYSIO_STREAM_YARP_STREAMER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Stream {
        class YarpStreamer;
    }
}

class hriPhysio::Stream::YarpStreamer {

private:
    int temp;

public:
    YarpStreamer();

private:
    void tempfunc();

};

#endif /* HRI_PHYSIO_STREAM_YARP_STREAMER_H */
