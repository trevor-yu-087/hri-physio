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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <HriPhysio/Dev/deviceInterface.h>
#include <HriPhysio/Stream/streamerInterface.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class PhysioManager;
    }
}

class hriPhysio::Manager::PhysioManager {
private:
    
    hriPhysio::Dev::DeviceInterface* dev;
    hriPhysio::Stream::StreamerInterface* stream;

    double period_read;
    double period_publish;
    
    std::atomic<bool> run_read;
    std::atomic<bool> run_publish;

    std::thread thread_read;
    std::thread thread_publish;


public:
    PhysioManager();

    void start();
    void step();
    void stop();

private:
    void deviceLoop();
    void streamLoop();

};

#endif /* HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H */
