//

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <doctest.h>

#include <HriPhysio/Core/streamerCore.h>

TEST_CASE("Successful Test Example") {
    
    hriPhysio::Core::streamerCore testCore;

    int a = 5;
    CHECK(a == 5);
}

TEST_CASE("Failing Test Examples") {

    hriPhysio::Core::streamerCore testCore;

    CHECK(true == false);
}

