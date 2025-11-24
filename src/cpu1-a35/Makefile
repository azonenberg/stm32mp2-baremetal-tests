all:

	# Compile
	aarch64-none-elf-g++ \
		-g \
		-fno-exceptions -fno-rtti -fno-threadsafe-statics \
		-nostdlib -nostartfiles \
		-mcpu=cortex-a35 \
		-Wl,-T../../stm32-cpp/devices/link/stm32mp257-cpu1-fsbl.ld \
		-L../../stm32-cpp/devices/link/ \
		-O0 \
		-static \
		-o main \
		startup.S \
		mainloop.cpp

	# Extract flat binary image
	../../common-embedded-platform/postbuild/stm32mp257-cpu1.sh main

	# Add FSBL image headers
	mkfsbl a main.bin fsbl.bin
