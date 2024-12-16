// The following code is a C++ wrapper of the code presented by
// Andrew D. Wilson in "Fast Lossless Depth Image Compression" at SIGCHI'17.
// The original code is licensed under the MIT License.

#include <rtabmap/core/rvl_codec.h>

namespace rtabmap
{

RvlCodec::RvlCodec() {}

void RvlCodec::EncodeVLE(int value)
{
    do
    {
        int nibble = value & 0x7;  // lower 3 bits
        if (value >>= 3)
            nibble |= 0x8;  // more to come
        word_ <<= 4;
        word_ |= nibble;
        if (++nibblesWritten_ == 8)  // output word
        {
            *pBuffer_++ = word_;
            nibblesWritten_ = 0;
            word_ = 0;
        }
    } while (value);
}

int RvlCodec::DecodeVLE()
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do
    {
        if (!nibblesWritten_)
        {
            word_ = *pBuffer_++;  // load word
            nibblesWritten_ = 8;
        }
        nibble = word_ & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word_ <<= 4;
        nibblesWritten_--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int RvlCodec::CompressRVL(const uint16_t * input, unsigned char * output, int numPixels)
{
    buffer_ = pBuffer_ = reinterpret_cast<int *>(output);
    nibblesWritten_ = 0;
    const uint16_t * end = input + numPixels;
    uint16_t previous = 0;
    while (input != end)
    {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++) {}
        EncodeVLE(zeros);  // number of zeros
        for (const uint16_t * p = input; (p != end) && *p++; nonzeros++) {}
        EncodeVLE(nonzeros);  // number of nonzeros
        for (int i = 0; i < nonzeros; i++)
        {
            uint16_t current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(positive);  // nonzero value
            previous = current;
        }
    }
    if (nibblesWritten_)  // last few values
        *pBuffer_++ = word_ << 4 * (8 - nibblesWritten_);
    return static_cast<int>((unsigned char *)pBuffer_ - (unsigned char *)buffer_);  // num bytes
}

void RvlCodec::DecompressRVL(const unsigned char * input, uint16_t * output, int numPixels)
{
    buffer_ = pBuffer_ = const_cast<int *>(reinterpret_cast<const int *>(input));
    nibblesWritten_ = 0;
    uint16_t current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode)
    {
        int zeros = DecodeVLE();  // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--) 
            *output++ = 0;
        int nonzeros = DecodeVLE();  // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--)
        {
            int positive = DecodeVLE();  // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}

}  // namespace rtabmap
