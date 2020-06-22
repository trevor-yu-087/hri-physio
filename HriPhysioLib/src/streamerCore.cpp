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

#include <HriPhysio/Core/streamerCore.h>

using namespace hriPhysio::Core;

StreamerCore::StreamerCore() {
    temp = 5; 
}

void StreamerCore::tempfunc() {
    temp = 10;
}
