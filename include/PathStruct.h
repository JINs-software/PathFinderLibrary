#pragma once

using BYTE = unsigned char;
using int8 = __int8;
using int16 = __int16;
using int32 = __int32;
using int64 = __int64;
using uint8 = unsigned __int8;
using uint16 = unsigned __int16;
using uint32 = unsigned __int32;
using uint64 = unsigned __int64;

#define BYTE_BIT	8

template<typename T>
struct PathPosition {
	bool operator==(PathPosition& other) {
		return y == other.y && x == other.x;
	}
	bool operator==(const PathPosition& other) const {
		return y == other.y && x == other.x;
	}
	bool operator!=(PathPosition& other) {
		return !(*this == other);
	}
	bool operator<(const PathPosition& other) const
	{
		if (y != other.y)
			return y < other.y;
		return x < other.x;
	}
	PathPosition operator+(PathPosition& other) {
		PathPosition ret;
		ret.y = y + other.y;
		ret.x = x + other.x;
		return ret;
	}
	PathPosition& operator+=(PathPosition& other)
	{
		y += other.y;
		x += other.x;
		return *this;
	}

	T y;
	T x;
};
template<typename T>
using Pos = PathPosition<T>;

template<typename T>
struct Path {
private:
	Pos<T>* path;
	T capacity;
	T len;
	T offset;
public:
	Path() {
		offset = -1;
		len = 0;
		capacity = 2;
		path = new Pos<T>[capacity];
	}
	Path(Path&& src) {
		path = src.path;
		src.path = nullptr;

		capacity = src.capacity;
		len = src.len;
		offset = src.offset;
	}
	void operator=(Path&& src) {
		path = src.path;
		src.path = nullptr;

		capacity = src.capacity;
		len = src.len;
		offset = src.offset;
	}
	Path& operator<<(const Pos<T>& point) {
		if (len >= capacity) {
			// TO DO: overflow üũ
			Pos<T>* temp = new Pos<T>[capacity * 2];
			memcpy(temp, path, sizeof(Pos<T>) * capacity);
			delete[] path;
			path = temp;
			capacity *= 2;
		}
		*path[len++] = point;
		offset++;
	}

	void Add(const Pos<T>& point) {
		if (len >= capacity) {
			// TO DO: overflow üũ
			Pos<T>* temp = new Pos<T>[capacity * 2];
			memcpy(temp, path, sizeof(Pos<T>) * capacity);
			delete[] path;
			path = temp;
			capacity *= 2;
		}
		path[len++] = point;
		offset++;
	}

	Pos<T>* Now() {
		if (offset == -1) {
			return nullptr;
		}

		return path + offset;
	}
	Pos<T>* Next() {
		if (offset == 0) {
			return nullptr;
		}

		return path + (--offset);
	}
};


template<typename T>
struct PathNode {
	bool operator<(const PathNode& other) const {
		return f < other.f;
	}
	bool operator>(const PathNode& other) const {
		return f > other.f;
	}
	bool operator==(const PathNode& other) const {
		if (f == other.f && g == other.g && h == other.g && pos == other.pos && parentPos == other.parentPos) {
			return true;
		}
		else {
			return false;
		}
	}
	bool operator!=(const PathNode& other) const {
		if (f == other.f && g == other.g && h == other.g && pos == other.pos && parentPos == other.parentPos) {
			return false;
		}
		else {
			return true;
		}
	}

	T f;	// = g + h
	T g;
	T h;

	Pos<T> pos;
	Pos<T> parentPos;
};


BYTE bitMask[8] = {
	0b1000'0000,
	0b0100'0000,
	0b0010'0000,
	0b0001'0000,
	0b0000'1000,
	0b0000'0100,
	0b0000'0010,
	0b0000'0001
};
BYTE reverseMask[8] = {
	0b0111'1111,
	0b1011'1111,
	0b1101'1111,
	0b1110'1111,
	0b1111'0111,
	0b1111'1011,
	0b1111'1101,
	0b1111'1110
};

BYTE FindSetAbitPos(BYTE byte) {
	BYTE len = 8;
	BYTE leftPart;
	BYTE rigthPart;
	BYTE idx = 0;

	while ((byte & 1) == 0) {
		leftPart = byte >> (len / 2);
		//rigthPart = ((byte) << (len / 2)) >> (len / 2);

		if (leftPart != 0) {
			idx += (len / 2);
			byte = leftPart;
		}
		else {
			byte = (byte << (len / 2)) >> (len / 2);
		}

		len /= 2;
	}

	return 7 - idx;
}

// find LSB
BYTE GetLSBIdx(BYTE byte) {
	if (byte == 0) {
		return 8;
	}

	byte = byte & ~(byte - 1);
	return FindSetAbitPos(byte);
}
// find MSB
BYTE GetMSBIdx(BYTE byte) {
	if (byte == 0) {
		return 8;
	}
	if ((byte & 0b1000'0000) != 0) {
		return 0;
	}

	byte |= byte >> 1;
	byte |= byte >> 2;
	byte |= byte >> 4;

	byte = byte + 1;
	byte = byte >> 1;
	return FindSetAbitPos(byte);
}


// 1111 1111
// -> 0000 0000
// 1101 0111
//  ~ -> 0010 1000
// 
// find
BYTE GetLISBIdx(BYTE byte) {
	if (byte == 0) {
		return 7;
	}
	else if (byte == 255) {
		return 8;
	}

	return GetLSBIdx(~byte);
}
BYTE GetMISBIdx(BYTE byte) {
	if (byte == 0) {
		return 0;
	}
	else if (byte == 255) {
		return 8;
	}

	return GetMSBIdx(~byte);
}