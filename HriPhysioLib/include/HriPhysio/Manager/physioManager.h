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

#ifndef HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H
#define HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H

#include <iostream>
#include <memory>
#include <mutex>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class PhysioManager;
    }
}

class hriPhysio::Manager::PhysioManager {

private:
    int temp;

public:
    PhysioManager();

private:
    void tempfunc();

};

#endif /* HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H */
