cmake_minimum_required(VERSION 3.0)
project(mpegts_parser)

if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -static-libgcc")
endif()

if(WIN32)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_definitions(-D_WIN32_WINNT=0x0501)
	add_definitions(-DNOMINMAX)
endif()

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC_RUNTIME ON)
find_package(Boost COMPONENTS system filesystem REQUIRED)

add_executable(mpegts_parser
  main.cpp
  mpegts.cpp
  mpegts.hpp
)

if(UNIX)
	target_link_libraries(mpegts_parser ${Boost_LIBRARIES} pthread)
else()
	target_link_libraries(mpegts_parser ${Boost_LIBRARIES})
endif()

install(TARGETS mpegts_parser RUNTIME DESTINATION bin)
