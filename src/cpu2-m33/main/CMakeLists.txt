add_executable(main
	Init.cpp
	RemoteLoggerTask.cpp
	RISAFMonitorTask.cpp
	TestCLISessionContext.cpp
	vectors.cpp)

set(LINKER_SCRIPT "${PROJECT_ROOT}/stm32-cpp/devices/link/stm32mp257-cpu2-fsbl.ld")

target_link_options(main
	PUBLIC "-ffunction-sections"
	PUBLIC "-Wl,--gc-sections"
	PUBLIC "-Wl,-T${LINKER_SCRIPT}"
	PUBLIC "-L${PROJECT_ROOT}/stm32-cpp/devices/link/")

target_link_libraries(main
	common-embedded-platform-core
	common-embedded-platform-multicore
	embedded-cli
	bsp
	embedded-utils
	#microkvs
	stm32-cpp
	)

# Binary depends on the linker script
set_property(TARGET main APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
set_property(TARGET main APPEND PROPERTY LINK_DEPENDS "${PROJECT_ROOT}/stm32-cpp/devices/link/stm32mp257-base.ld")

# Calculate flash usage and generate binary file
common_postbuild(main)
