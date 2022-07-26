FIND_PATH(g2o_INCLUDE_DIR
        NAMES g2o/config.h
        HINTS /home/wfram/vslam2_ws/devel/include)

if (g2o_INCLUDE_DIR)
    message("CMake Modules found g2o:  ${g2o_INCLUDE_DIR}")
    find_library(g2o_LIBRARY g2o
            HINTS ${g2o_INCLUDE_DIR}/../lib/g2o)
    if (g2o_LIBRARY)
        message("CMake Modules g2o LIB:  ${g2o_LIBRARY}")
    else ()
        set(g2o_LIBRARY /home/wfram/vslam2_ws/devel/lib/libg2o.so)
        message("CMake Modules g2o LIB not found | bdc HARDCODING")
        message("CMake Modules g2o LIB:  ${g2o_LIBRARY}")
    endif ()
else ()
    message("CMake Modules g2o not found | bdc HARDCODING")
    set(g2o_INCLUDE_DIR /home/wfram/vslam2_ws/devel/include/g2o)
    set(g2o_LIBRARY /home/wfram/vslam2_ws/devel/lib/g2o/libg2o.so)
    message("CMake Modules g2o INCLUDE:  ${g2o_INCLUDE_DIR}")
    message("CMake Modules g2o LIB    :  ${g2o_LIBRARY}")
endif ()

set(g2o_DIRS ${g2o_DIR})
set(g2o_INCLUDE_DIRS ${g2o_INCLUDE_DIR})
set(g2o_LIBRARIES ${g2o_LIBRARY})
