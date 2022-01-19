set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wextra -g")

add_library(pipeline SHARED 
            pipeline.cpp
            )
