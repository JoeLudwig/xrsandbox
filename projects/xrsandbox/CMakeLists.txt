cmake_minimum_required (VERSION 3.6)

add_executable(xrsandbox 
	WIN32 
		src/xrs_app.cpp
		../xrbase/public/main_windows.cpp
)

set_property( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT xrsandbox )

add_dependencies( xrsandbox xrbase )

target_link_libraries(xrsandbox
PRIVATE
	xrbase

	PhysXExtensions 
	PhysX 
	PhysXPvdSDK 
	PhysXVehicle 
	PhysXCharacterKinematic 
	PhysXCooking 
	PhysXCommon 
	PhysXFoundation
)

if(PLATFORM_WIN32 OR PLATFORM_LINUX)
	# Copy assets to target folder
	add_custom_command(TARGET xrsandbox POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			"${CMAKE_CURRENT_SOURCE_DIR}/assets"
			"\"$<TARGET_FILE_DIR:xrsandbox>\"")
endif()

if( PLATFORM_WIN32 )
	set_target_properties(xrsandbox PROPERTIES 
		VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets"
	)
endif()

copy_required_dlls(xrsandbox)

function(copy_physx_dlls TARGET_NAME)
    list(APPEND PHYSX_DLLS PhysX)
    list(APPEND PHYSX_DLLS PhysXCharacterKinematic)
    list(APPEND PHYSX_DLLS PhysXCommon)
    list(APPEND PHYSX_DLLS PhysXCooking)
    list(APPEND PHYSX_DLLS PhysXExtensions)
    list(APPEND PHYSX_DLLS PhysXFoundation)
    list(APPEND PHYSX_DLLS PhysXPvdSDK)
    list(APPEND PHYSX_DLLS PhysXTask)
    list(APPEND PHYSX_DLLS PhysXVehicle)

    foreach(DLL ${PHYSX_DLLS})
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "\"$<TARGET_FILE:${DLL}>\""
                "\"$<TARGET_FILE_DIR:${TARGET_NAME}>\"")
    endforeach(DLL)
endfunction()

copy_physx_dlls(xrsandbox)