all:
	arm-none-eabi-g++ \
		-g \
		-fno-exceptions -fno-rtti --specs=nano.specs -fno-threadsafe-statics \
		-nostartfiles \
		-mcpu=cortex-m33 -mfloat-abi=hard \
		-T stm32mp257-cpu2-ramload.ld \
		-O0 \
		-o test.elf \
		main.S
