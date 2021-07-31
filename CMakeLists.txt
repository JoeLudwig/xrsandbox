cmake_minimum_required ( VERSION 3.14)
project( XRSandbox VERSION 0.1.0
					DESCRIPTION "A sandbox for XR"
					LANGUAGES CXX)
set(NAME XRSandbox)

add_subdirectory( thirdparty/DiligentEngine/DiligentCore )
add_subdirectory( thirdparty/DiligentEngine/DiligentTools )
add_subdirectory( thirdparty/DiligentEngine/DiligentFx )
add_subdirectory( thirdparty/OpenXR-SDK )

set( PHYSX_ROOT_DIR ${CMAKE_SOURCE_DIR}/thirdparty/PhysX/physx )
set( PX_BUILDSNIPPETS True )
set( PX_BUILDPUBLICSAMPLES False )
set( PX_GENERATE_STATIC_LIBRARIES False )
set( NV_USE_STATIC_WINCRT False )
set( NV_USE_DEBUG_WINCRT True )
set( PX_FLOAT_POINT_PRECISE_MATH False )
set( PX_CMAKE_SUPPRESS_REGENERATION False )
set( PX_OUTPUT_LIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/physx/lib )
set( PX_OUTPUT_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/physx/bin )
set( TARGET_BUILD_PLATFORM windows )
set( ENV{PHYSX_ROOT_DIR} ${CMAKE_SOURCE_DIR}/thirdparty/PhysX/physx )
set( ENV{PM_VSWHERE_PATH} "${PHYSX_ROOT_DIR}/../externals/VsWhere" )
set( ENV{PM_CMAKEMODULES_PATH} "${PHYSX_ROOT_DIR}/../externals/CMakeModules" )
set( ENV{PM_PXSHARED_PATH} "${PHYSX_ROOT_DIR}/../pxshared" )
set( ENV{PM_TARGA_PATH} "${PHYSX_ROOT_DIR}/../externals/targa" )
set( ENV{PM_PATHS} "$ENV{PM_CMAKEMODULES_PATH};$ENV{PM_TARGA_PATH}" )

set( CMAKE_INSTALL_PREFIX "build/install/vc15win64/PhysX" )
add_subdirectory( "thirdparty/PhysX/physx/compiler/public" )

add_compile_options( "/std:c++17" )

add_subdirectory( projects )

