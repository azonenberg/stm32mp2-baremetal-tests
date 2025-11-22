all:
	aarch64-linux-gnu-g++ \
		-g \
		-fno-exceptions -fno-rtti -fno-threadsafe-statics \
		-nostdlib -nostartfiles \
		-mcpu=cortex-a35 \
		-Wl,-T../../stm32-cpp/devices/link/stm32mp257-cpu1-fsbl.ld \
		-L../../stm32-cpp/devices/link/ \
		-O0 \
		-o test.elf \
		main.S

	../../common-embedded-platform/postbuild/stm32mp257-cpu1.sh test.elf

#--specs=nano.specs
