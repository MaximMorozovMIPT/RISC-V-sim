set(BINARY tests)

file(GLOB SOURCES LIST_DIRECTORIES false *.h *.cpp)

add_executable(${BINARY} ${SOURCES})

add_test(NAME ${BINARY} COMMAND ${BINARY})

target_link_libraries(${BINARY} PUBLIC pipeline gtest)