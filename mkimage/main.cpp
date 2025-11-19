#include <stdio.h>
#include <stdint.h>
#include <string.h>

int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		fprintf(stderr, "Usage: mkimage image.bin\n");
		return 1;
	}

	FILE* fp = fopen(argv[1], "rb");
	if(!fp)
	{
		fprintf(stderr, "fail to open %s\n", argv[1]);
		return 1;
	}

	FILE* fout = fopen("out.bin", "wb");
	if(!fout)
	{
		fprintf(stderr, "fail to open out.bin\n");
		return 1;
	}

	//Read the input file
	fseek(fp, 0, SEEK_END);
	uint32_t len = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	uint8_t* image = new uint8_t[len];
	fread(image, 1, len, fp);
	printf("Read %u bytes from input file\n", len);

	//Rebase the image to the desired mapping of RETRAM the bootrom wants
	uint32_t* preset = reinterpret_cast<uint32_t*>(image + 4);
	uint32_t entry = *preset;
	/*entry = (entry & 0xffff) | 0x0e080000;
	*preset = entry;*/

	//Magic number
	const uint8_t magic[] = { 'S', 'T', 'M', 0x32};
	fwrite(magic, 1, sizeof(magic), fout);

	//ECDSA image signature (leave blank since we're not doing secure boot
	uint8_t signature[64] = {0};
	fwrite(signature, 1, sizeof(signature), fout);

	//Image checksum
	uint32_t checksum = 0;
	for(size_t i=0; i<len; i++)
		checksum += image[i];
	printf("Image checksum: %08x\n", checksum);
	fwrite(&checksum, 1, sizeof(checksum), fout);

	//Header version
	uint32_t headerVersion = 0x00020200;
	fwrite(&headerVersion, 1, sizeof(headerVersion), fout);

	//Image length
	fwrite(&len, 1, sizeof(len), fout);

	//Image entry point
	printf("Entry point: %08x\n", entry);
	fwrite(&entry, 1, sizeof(entry), fout);

	//4 byte reserved/padding
	uint8_t padding[4] = {0};
	fwrite(&padding, 1, sizeof(padding), fout);

	//Load address, not used by ROM but stm32wrapper4dbg wants it
	uint32_t load_address = 0x0e080000;
	fwrite(&load_address, 1, sizeof(load_address), fout);

	//4 byte reserved/padding
	fwrite(&padding, 1, sizeof(padding), fout);

	//Image version for anti-rollback (start at 0x0)
	uint32_t imageVersion = 0;
	fwrite(&imageVersion, 1, sizeof(imageVersion), fout);

	//Option flags: header padding enabled
	uint32_t options = 0x8000'0000;
	fwrite(&options, 1, sizeof(options), fout);

	//Header extension length
	uint32_t headerExtLen = 512 - 128;
	fwrite(&headerExtLen, 1, sizeof(headerExtLen), fout);

	//Binary type, set to 0x30 for FSBL-M image
	uint32_t binaryType = 0x30;
	fwrite(&binaryType, 1, sizeof(binaryType), fout);

	//Padding at end of header
	uint8_t padding2[16];
	fwrite(&padding2, 1, sizeof(padding2), fout);
	printf("Length at end of main header: %d bytes\n", ftell(fout));

	//Padding extension
	uint8_t extensionType[] = { 'S', 'T', 0xff, 0xff };
	fwrite(extensionType, 1, sizeof(extensionType), fout);

	//Extension length
	uint32_t extensionLength = headerExtLen;
	fwrite(&extensionLength, 1, sizeof(extensionLength), fout);

	//Actual padding
	//this shouldn't matter but ST example image seems to write 0xad here
	uint8_t padding3[512];
	memset(padding3, 0xad, sizeof(padding3));
	fwrite(padding3, 1, extensionLength - 8, fout);
	printf("Padded header: %d bytes\n", ftell(fout));

	//Actual firmware image
	fwrite(image, 1, len, fout);

	//Clean up
	delete[] image;
	fclose(fout);
	fclose(fp);
	return 0;
}

