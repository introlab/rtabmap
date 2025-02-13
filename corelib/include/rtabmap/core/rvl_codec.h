// The following code is a C++ wrapper of the code presented by
// Andrew D. Wilson in "Fast Lossless Depth Image Compression" at SIGCHI'17.
// The original code is licensed under the MIT License.

#ifndef RVL_CODEC_H_
#define RVL_CODEC_H_

#include <cstdint>
#include "rtabmap/core/rtabmap_core_export.h"

namespace rtabmap
{

class RTABMAP_CORE_EXPORT RvlCodec {
public:
    RvlCodec();
    // Compress input data into output. The size of output can be equal to (1.5 * numPixels + 4) in the worst case.
    int CompressRVL(const uint16_t * input, unsigned char * output, int numPixels);
    // Decompress input data into output. The size of output must be equal to numPixels.
    void DecompressRVL(const unsigned char * input, uint16_t * output, int numPixels);

private:
    RvlCodec(const RvlCodec &);
    RvlCodec & operator=(const RvlCodec &);

    void EncodeVLE(int value);
    int DecodeVLE();

    int *buffer_;
    int *pBuffer_;
    int word_;
    int nibblesWritten_;
};

}  // namespace rtabmap

#endif  // RVL_CODEC_H_
