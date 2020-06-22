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

#include <doctest.h>

#include <HriPhysio/Core/ringBuffer.h>

TEST_CASE("Successful Test Example") {

    hriPhysio::Core::RingBuffer <int> b(10);

    int a = 5;
    CHECK(a == 5);
}

TEST_CASE("Failing Test Examples") {

    CHECK(true == false);
}

