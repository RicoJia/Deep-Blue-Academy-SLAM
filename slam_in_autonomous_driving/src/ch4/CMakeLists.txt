find_package(GTest REQUIRED)          # you already have this

# ------------------------------------------------------------------
# If CMake already created a target called 'gmock' but did not
# give it a library path, patch it so the linker can find the .a.
# ------------------------------------------------------------------
if(TARGET gmock)
    # Check whether IMPORTED_LOCATION is empty
    get_target_property(_gmock_loc gmock IMPORTED_LOCATION)
    if(NOT _gmock_loc)
        # Tell CMake where the static lib installed by libgmock-dev lives
        set_target_properties(gmock PROPERTIES
            IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libgmock.a
            INTERFACE_INCLUDE_DIRECTORIES /usr/include
        )
    endif()
else()
    # Very old CMake versions may not create the stub at all
    add_library(gmock STATIC IMPORTED)
    set_target_properties(gmock PROPERTIES
        IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libgmock.a
        INTERFACE_INCLUDE_DIRECTORIES /usr/include
    )
endif()



add_library(${PROJECT_NAME}.ch4
        gins_pre_integ.cc
        imu_preintegration.cc
        g2o_types.cc
        )

ADD_EXECUTABLE(test_preintegration test_preintegration.cc)
ADD_TEST(test_preintegration test_preintegration)

target_link_libraries(test_preintegration
        gtest pthread glog gflags ${PROJECT_NAME}.ch4 ${PROJECT_NAME}.ch3 ${PROJECT_NAME}.common
        )

add_executable(run_gins_pre_integ run_gins_pre_integ.cc)
target_link_libraries(run_gins_pre_integ
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch4
        ${g2o_libs}
        )