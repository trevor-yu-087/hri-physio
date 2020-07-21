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

#include <HriPhysio/Manager/physioManager.h>

using namespace hriPhysio::Manager;


PhysioManager::PhysioManager() {

}


void PhysioManager::start() {

    if (!run_read) {
        thread_read = std::thread(&PhysioManager::deviceLoop, this);
    }

    if (!run_publish) {
        thread_publish = std::thread(&PhysioManager::streamLoop, this);
    }

}


void PhysioManager::step() {

}


void PhysioManager::stop() {

    //-- Release the loops, so that return can be reached.
    run_read = false;
    run_publish = false;
    
    //-- Join the threads.
    thread_read.join();
    thread_publish.join();

    return;
}


void PhysioManager::deviceLoop() {

    run_read = true;
    while (run_read) {

        dev->read();
        
        std::this_thread::sleep_for(
            std::chrono::duration<double>(period_read) //seconds.
        );
    }
}


void PhysioManager::streamLoop() {

    run_publish = true;
    while (run_publish) {
        
        bool success = dev->write();

        if (success) {
            stream->publish();
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(period_publish) //seconds.
        );
    }
}
