cmake_minimum_required ( VERSION 3.14)
project( XRSandbox VERSION 0.1.0
					DESCRIPTION "A sandbox for XR"
					LANGUAGES CXX)
set(NAME XRSandbox)

add_subdirectory( thirdparty/DiligentEngine/DiligentCore )
add_subdirectory( thirdparty/DiligentEngine/DiligentTools )
add_subdirectory( thirdparty/DiligentEngine/DiligentFx )
add_subdirectory( thirdparty/OpenXR-SDK )

add_compile_options( "/std:c++17" )

add_subdirectory( projects )

