#include <string.h>
#include <assert.h>

#include "asn1crt_encoding_acn.h"



void Acn_AlignToNextByte(BitStream* pBitStrm, flag bEncode)
{
	CHECK_BIT_STREAM_PRE(pBitStrm, (NO_OF_BITS_IN_BYTE -
		pBitStrm->currentBit) & (NO_OF_BITS_IN_BYTE - 1));

	if (pBitStrm->currentBit != 0)
	{
		pBitStrm->currentBit = 0;
		pBitStrm->currentByte++;
		if (bEncode)
			bitstream_push_data_if_required(pBitStrm);
		else
			bitstream_fetch_data_if_required(pBitStrm);
		CHECK_BIT_STREAM(pBitStrm);
	}
}





/*ACN Integer functions*/
void Acn_Enc_Int_PositiveInteger_ConstSize(BitStream* pBitStrm, asn1SccUint intVal, int encodedSizeInBits)
{
	int nBits = 0;
	if (encodedSizeInBits == 0)
		return;
	/* Get number of bits*/
	nBits = GetNumberOfBitsForNonNegativeInteger(intVal);
	/* put required zeros*/
	BitStream_AppendNBitZero(pBitStrm, encodedSizeInBits - nBits);
	/*Encode number */
	BitStream_EncodeNonNegativeInteger(pBitStrm, intVal);

	CHECK_BIT_STREAM(pBitStrm);
}
void Acn_Enc_Int_PositiveInteger_ConstSize_8(BitStream* pBitStrm, asn1SccUint intVal)
{
	BitStream_AppendByte0(pBitStrm, (byte)intVal);
	CHECK_BIT_STREAM(pBitStrm);
}


















flag Acn_Dec_Int_PositiveInteger_ConstSize(BitStream* pBitStrm, asn1SccUint* pIntVal, int encodedSizeInBits)
{
	asn1SccUint tmp = 0;
	if (BitStream_DecodeNonNegativeInteger(pBitStrm, &tmp, encodedSizeInBits))
	{
		*pIntVal = tmp;
		return TRUE;
	}
	return FALSE;

}


flag Acn_Dec_Int_PositiveInteger_ConstSize_8(BitStream* pBitStrm, asn1SccUint* pIntVal)
{
	byte v = 0;
	if (!BitStream_ReadByte(pBitStrm, &v))
		return FALSE;
	*pIntVal = v;
	return TRUE;
}














































































//return values is in nibbles


















//encoding puts an 'F' at the end















void getIntegerDigits(asn1SccUint intVal, byte digitsArray100[], byte* totalDigits);

void getIntegerDigits(asn1SccUint intVal, byte digitsArray100[], byte* totalDigits) {
	int i = 0;
	*totalDigits = 0;
	byte reversedDigitsArray[100];
	memset(reversedDigitsArray, 0x0, 100);
	memset(digitsArray100, 0x0, 100);
	if (intVal > 0) {
		while (intVal > 0 && *totalDigits < 100) {
			reversedDigitsArray[*totalDigits] = '0' + (byte)(intVal % 10);
			(*totalDigits)++;
			intVal /= 10;
		}
		for (i = *totalDigits - 1; i >= 0; i--) {
			digitsArray100[(*totalDigits - 1) - i] = reversedDigitsArray[i];
		}
	}
	else {
		digitsArray100[0] = '0';
		*totalDigits = 1;
	}
}



























/* Boolean Decode */

flag BitStream_ReadBitPattern(BitStream* pBitStrm, const byte* patternToRead, int nBitsToRead, flag* pBoolValue)
{
	int nBytesToRead = nBitsToRead / 8;
	int nRemainingBitsToRead = nBitsToRead % 8;
	byte curByte;
	int i = 0;

	*pBoolValue = TRUE;
	for (i = 0; i<nBytesToRead; i++) {
		if (!BitStream_ReadByte(pBitStrm, &curByte))
			return FALSE;
		if (curByte != patternToRead[i])
			*pBoolValue = FALSE;
	}

	if (nRemainingBitsToRead > 0) {
		if (!BitStream_ReadPartialByte(pBitStrm, &curByte, (byte)nRemainingBitsToRead))
			return FALSE;
		if (curByte != patternToRead[nBytesToRead] >> (8 - nRemainingBitsToRead))
			*pBoolValue = FALSE;
	}

	return TRUE;
}

flag BitStream_ReadBitPattern_ignore_value(BitStream* pBitStrm, int nBitsToRead)
{
	int nBytesToRead = nBitsToRead / 8;
	int nRemainingBitsToRead = nBitsToRead % 8;
	byte curByte;
	int i = 0;

	for (i = 0; i<nBytesToRead; i++) {
		if (!BitStream_ReadByte(pBitStrm, &curByte))
			return FALSE;
	}

	if (nRemainingBitsToRead > 0) {
		if (!BitStream_ReadPartialByte(pBitStrm, &curByte, (byte)nRemainingBitsToRead))
			return FALSE;
	}

	return TRUE;

}

/*Real encoding functions*/
typedef union _float_tag
{
	float f;
	byte b[sizeof(float)];
} _float;

typedef union _double_tag
{
	double f;
	byte b[sizeof(double)];
} _double;


#define Acn_enc_real_big_endian(type)       \
    int i;                      \
    _##type dat1;               \
    dat1.f = (type)realValue;   \
    if (!RequiresReverse()) {   \
        for(i=0;i<(int)sizeof(dat1);i++)        \
            BitStream_AppendByte0(pBitStrm,dat1.b[i]);  \
    } else {    \
        for(i=(int)(sizeof(dat1)-1);i>=0;i--)   \
            BitStream_AppendByte0(pBitStrm,dat1.b[i]);  \
    }   \


#define Acn_dec_real_big_endian(type)   \
    int i;                  \
    _##type dat1;           \
    dat1.f=0.0;             \
    if (!RequiresReverse()) {       \
        for(i=0;i<(int)sizeof(dat1);i++) {  \
            if (!BitStream_ReadByte(pBitStrm, &dat1.b[i]))  \
                return FALSE;       \
        }                           \
    } else {                        \
        for(i=(int)(sizeof(dat1)-1);i>=0;i--) {         \
            if (!BitStream_ReadByte(pBitStrm, &dat1.b[i]))      \
                return FALSE;           \
        }       \
    }       \
    *pRealValue = dat1.f;   \
    return TRUE;            \
















#define Acn_enc_real_little_endian(type)        \
    int i;                      \
    _##type dat1;               \
    dat1.f = (type)realValue;   \
    if (RequiresReverse()) {    \
        for(i=0;i<(int)sizeof(dat1);i++)        \
            BitStream_AppendByte0(pBitStrm,dat1.b[i]);  \
    } else {    \
        for(i=(int)(sizeof(dat1)-1);i>=0;i--)   \
            BitStream_AppendByte0(pBitStrm,dat1.b[i]);  \
    }   \


#define Acn_dec_real_little_endian(type)    \
    int i;                  \
    _##type dat1;           \
    dat1.f=0.0;             \
    if (RequiresReverse()) {        \
        for(i=0;i<(int)sizeof(dat1);i++) {  \
            if (!BitStream_ReadByte(pBitStrm, &dat1.b[i]))  \
                return FALSE;       \
        }                           \
    } else {                        \
        for(i=(int)(sizeof(dat1)-1);i>=0;i--) {         \
            if (!BitStream_ReadByte(pBitStrm, &dat1.b[i]))      \
                return FALSE;           \
        }       \
    }       \
    *pRealValue = dat1.f;   \
    return TRUE;            \















/* String functions*/

































































/* Length Determinant functions*/


















flag Acn_Dec_Int_PositiveInteger_ConstSize_8UInt8(BitStream* pBitStrm, uint8_t* pIntVal) {
	asn1SccUint v;
	flag ret = Acn_Dec_Int_PositiveInteger_ConstSize_8(pBitStrm, &v);
	*pIntVal = (uint8_t)v;
	return ret;
}

















































































































































































































