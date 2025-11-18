add_executable(main
	Init.cpp
	vectors.cpp)

target_link_options(main
	PUBLIC "-ffunction-sections"
	PUBLIC "-Wl,--gc-sections"
	PUBLIC "-Wl,-T${PROJECT_ROOT}/stm32-cpp/devices/link/stm32mp257-cpu2-ramload.ld"
	PUBLIC "-L${PROJECT_ROOT}/stm32-cpp/devices/link/")

target_link_libraries(main
	common-embedded-platform-core
	embedded-cli
	bsp
	embedded-utils
	#microkvs
	stm32-cpp
	)

# Calculate flash usage and generate binary file
common_postbuild(main)
