cmake_minimum_required(VERSION 3.8)
project(soemtest)


project(eRob_test LANGUAGES C CXX)
find_package(Threads REQUIRED)

include_directories(/home/erobman/SOEM_ethercat/install/SOEM/include/soem)
link_directories(/home/erobman/SOEM_ethercat/install/SOEM/lib)

set(SOURCES simple_test.c)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem)
install(TARGETS simple_test DESTINATION bin)



add_executable(eRob_test eRob_test.cpp)
target_link_libraries(eRob_test soem)
install(TARGETS eRob_test DESTINATION bin)


add_executable(github_test github.cpp)
target_link_libraries(github_test soem)
install(TARGETS github_test DESTINATION bin)