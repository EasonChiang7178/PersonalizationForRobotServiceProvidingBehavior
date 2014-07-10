/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __FaceDetectionLcm_hpp__
#define __FaceDetectionLcm_hpp__



class FaceDetectionLcm
{
    public:
        int32_t    face_direction;

    public:
        inline int encode(void *buf, int offset, int maxlen) const;
        inline int getEncodedSize() const;
        inline int decode(const void *buf, int offset, int maxlen);
        inline static int64_t getHash();
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int FaceDetectionLcm::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int FaceDetectionLcm::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int FaceDetectionLcm::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t FaceDetectionLcm::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* FaceDetectionLcm::getTypeName()
{
    return "FaceDetectionLcm";
}

int FaceDetectionLcm::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->face_direction, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int FaceDetectionLcm::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->face_direction, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int FaceDetectionLcm::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t FaceDetectionLcm::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0x86f63f5537e4f474LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif