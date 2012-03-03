
#ifndef COLORTABLE_H
#define COLORTABLE_H

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <vector>

namespace rtabmap
{
class RTABMAP_EXP ColorTable
{
public:
	ColorTable(int size);
	virtual ~ColorTable() {}

	static unsigned char INDEXED_TABLE_8[24];
	static unsigned char INDEXED_TABLE_16[48];
	static unsigned char INDEXED_TABLE_32[96];
	static unsigned char INDEXED_TABLE_64[192];
	static unsigned char INDEXED_TABLE_128[384];
	static unsigned char INDEXED_TABLE_256[768];
	static unsigned char INDEXED_TABLE_512[1536];
	static unsigned char INDEXED_TABLE_1024[3076];
	static unsigned char INDEXED_TABLE_65536[196608];

	int size() const {return _size;}
	unsigned short getIndex(unsigned char r, unsigned char g, unsigned char b) const;
	void getRgb(unsigned short index, unsigned char & r, unsigned char & g, unsigned char & b) const;

	unsigned short getNNIndex(unsigned char r, unsigned char g, unsigned char b) const;
	void getNNRgb(unsigned short index, unsigned char & r, unsigned char & g, unsigned char & b) const;

private:
	int _size;
	std::vector<unsigned short> _rgb2indexed;
	unsigned char * _indexedTable;
};

} // namespace rtabmap

#endif // COLORTABLE_H
