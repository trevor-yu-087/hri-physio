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

#ifndef STREAMER_CORE_H
#define STREAMER_CORE_H

namespace hriPhysio {
    namespace Core {
        class StreamerCore;
    }
}

class hriPhysio::Core::StreamerCore {

private:
    int temp;

public:
    StreamerCore();

private:
    void tempfunc();

};

#endif /* STREAMER_CORE_H */
