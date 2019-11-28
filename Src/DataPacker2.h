#ifndef DATAPACKER_h
#define DATAPACKER_h

/*
* ---DataPackerV2---
* Packs data into byte arrays for fast transfering to peripherals,
* or to save/read from EEPROM
* Provides CRC16 checking for data consistency assurance
*
* Array structure:
*    [data]       + cypher seed   +   CRC
*    n bytes      +   1 byte      +  2 bytes = packetSize
*
* Note: CRC calculates all bytes (except for CRC bytes itself), including not occupied bytes
* ------------------
* author: maisonsmd
* contact: maisonsmd@gmail.com
*/

#include "crc16.h"

template <uint8_t packetSize = 32>
class DataPacker2
{
private:
	// buffer
	uint8_t data[packetSize];
	// map of data sizes
	uint8_t map[packetSize];
	// number of data member occupied
	uint8_t occupiedMembers = 0;
	// length in bytes occupied
	uint8_t occupiedBytes = 0;
public:
	int16_t(*cipherAlgorithm)(int16_t index, int16_t seed) = nullptr;

	DataPacker2() {
		clear();
	}

	void clone(uint8_t * src, uint8_t srcOffset = 0, uint8_t len = packetSize) {
		if (len > packetSize)
			len = packetSize;
		memcpy(data, src + srcOffset, len);
	}

	void setSizeMap(uint8_t * map, uint8_t count) {
		this->occupiedMembers = 0;
		this->occupiedBytes = 0;
		if (map == nullptr || count == 0 || count > packetSize - 3)
			return;
		memcpy(this->map, map, count);
		for (uint8_t i = 0; i < count; ++i) {
			if (this->occupiedBytes + map[i] > packetSize - 3)
				break;
			this->occupiedBytes += map[i];
			this->occupiedMembers++;
		}
	}

	inline uint8_t * getBuffer() {
		return data;
	}

	void clear() {
		memset(data, 0, packetSize);
		occupiedMembers = 0;
		occupiedBytes = 0;
	}

	inline uint8_t size() {
		return packetSize;
	}

	inline uint8_t length() {
		return occupiedBytes;
	}

	/// push numbers / structs / classes to the end of packet, return index of newly added number
	template<typename T>
	int16_t push(T value) {
		uint8_t typeSize = sizeof(T);
		if (occupiedBytes + typeSize + 3 > packetSize)
			return -1;

		memcpy(data + occupiedBytes, &value, typeSize);

		map[occupiedMembers++] = typeSize;
		occupiedBytes += typeSize;
		return occupiedMembers - 1;
	}

	/// put numbers / structs / classes at predetermined index
	template<typename T>
	int16_t putAt(uint8_t index, T value) {
		if (index >= occupiedMembers)
			return -1;

		uint8_t typeSize = sizeof(T);

		uint8_t offset = 0;
		for (uint8_t i = 0; i < index; ++i)
			offset += map[i];

		if (offset + map[index] + 3 > packetSize)
			return -1;

		memcpy(data + offset, &value, typeSize);
		return index;
	}

	/// push char array to the end of packet
	int16_t pushString(const char * str, uint8_t fixedSize = 0) {
		uint8_t strSize = fixedSize > 0 ? fixedSize : ((uint8_t)strlen(str) + 1);
		if (occupiedBytes + strSize + 3 > packetSize)
			return -1;

		memccpy(data + occupiedBytes, str, 0, strSize);

		map[occupiedMembers++] = strSize;
		occupiedBytes += strSize;
		return occupiedMembers - 1;
	}

	/// put char array at predeterminied index, the length of the array is set in setSizeMap
	int16_t putStringAt(uint8_t index, const char * str) {
		if (index >= occupiedMembers)
			return -1;

		uint8_t offset = 0;
		for (uint8_t i = 0; i < index; ++i)
			offset += map[i];

		if (offset + map[index] + 3 > packetSize)
			return -1;

		memccpy(data + offset, str, 0, map[index]);

		return index;
	}

	/// get numbers / structs / classes
	template<typename T>
	T get(uint8_t index) {
		if (index >= occupiedMembers || map[index] == 0)
			return T();

		uint8_t offset = 0;
		for (uint8_t i = 0; i < index; ++i)
			offset += map[i];

		// this simple casting raises GCC warning: [-Wstrict-aliasing]
		// return *(reinterpret_cast<T*>(&data[offset]));

		union
		{
			uint8_t * buff;
			T * val;
		} temp;
		temp.buff = data + offset;
		return *(temp.val);

		/*T value;
		memcpy(&value, data + offset, sizeof(T));
		return value;*/
	}

	/// get char array and copy it to 'buffer'
	/// return null pointer if failed, and return buffer pointer if succedded
	char * getString(uint8_t index, char * buffer, uint8_t bufferSize) {
		if (index >= occupiedMembers || map[index] == 0)
			return nullptr;

		uint8_t offset = 0;
		for (uint8_t i = 0; i < index; ++i)
			offset += map[i];

		memccpy(buffer, data + offset, 0, bufferSize);
		buffer[bufferSize-1] = '\0';

		return buffer;
	};

	/// calculate CRC from packet
	uint16_t calcCRC() {
		return calculateCRC16(data, packetSize - 3);
	}

	/// get CRC embedded in packet
	uint16_t getCRC() {
		union
		{
			uint8_t * buff;
			uint16_t * val;
		} temp;
		temp.buff = data + (packetSize - 2);
		return *(temp.val);
	}

	/// calculate CRC and then put it to the end of the packet
	void putCRC() {
		uint16_t crc = calcCRC();
		memcpy(data + (packetSize - 2), &crc, 2);
	}

	/// check data for consistency
	bool checkCRC() {
		return getCRC() == calcCRC();
	}

	void cipher(uint8_t seed) {
		if (cipherAlgorithm)
			for (uint8_t i = 0; i < packetSize - 3; ++i) {
				int16_t shuffle = data[i];
				shuffle += cipherAlgorithm(i, seed);
				while (shuffle > 255)
					shuffle -= 255;
				data[i] = shuffle;
			}
		data[packetSize - 3] = seed;
	}

	void decipher() {
		uint8_t seed = data[packetSize - 3];
		if (cipherAlgorithm)
			for (uint8_t i = 0; i < packetSize - 3; ++i) {
				int16_t shuffle = data[i];
				shuffle -= cipherAlgorithm(i, seed);
				while (shuffle < 0)
					shuffle += 255;
				data[i] = shuffle;
			}
	}

#ifdef USE_DUMPER
	char * dump() {
		static char textBuffer[packetSize * 3 + 4];
		const auto toHex = [](uint8_t val)->char {
			if (val > 0x0F)
				return '?';
			static const char * hexChar = "0123456789ABCDEF";
			return hexChar[val];
		};
		uint16_t currentIndex = 0;
		textBuffer[currentIndex++] = '[';
		textBuffer[currentIndex++] = ' ';

		for (uint16_t i = 0; i < packetSize; i++) {
			if (data[i] < 0x10) {
				textBuffer[currentIndex++] = '0';
				textBuffer[currentIndex++] = toHex(data[i]);//to ascii
			} else {
				textBuffer[currentIndex++] = toHex(data[i] >> 4);
				textBuffer[currentIndex++] = toHex(data[i] & 0x0F);//to ascii
			}
			textBuffer[currentIndex++] = ' ';
		}
		textBuffer[currentIndex++] = ']';
		textBuffer[currentIndex] = '\0';
		return textBuffer;
}
#endif
};

#endif