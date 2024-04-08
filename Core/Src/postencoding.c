#include "asn1crt.h"
#include "asn1crt_encoding.h"
#include "asn1crt_encoding_acn.h"
#include "stm32g0xx.h"
#include "postencoding.h"

extern CRC_HandleTypeDef hcrc;

asn1SccUint calc_crc_32(byte *buf, long size) {
	return ~HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, size);
}

void my_encoding_patcher(const Packet_Type *pPacket, BitStream *pStartBitStrm,
		Packet_Type_extension_function_positions *pNullPos, BitStream *pEndBitStrm) {

	(void) pPacket;
	(void) pStartBitStrm;
	(void) pEndBitStrm;

	//this is the start position of the body-length-in-bytes field
	asn1SccUint startPosInBits = pNullPos->Packet_Type_body_length_in_bytes.currentByte * 8
			+ pNullPos->Packet_Type_body_length_in_bytes.currentBit;

	//this is the start position of the packet-crc32 field
	asn1SccUint endPosIBits = pNullPos->Packet_Type_packet_crc32.currentByte * 8
			+ pNullPos->Packet_Type_packet_crc32.currentBit;

	//the length of the body is difference between in positions between body-length-in-bytes field and packet-crc32
	// minus the length of the packet-crc32 field (16).
	asn1SccUint lengthInBytes = (endPosIBits - startPosInBits - 16) / 8;

	//use the ACN function to encode the length value. Please note that we use the Packet_Type_packet_length_in_bytes field in the
	// Packet_Type_extension_function_positions to encode the length in the correct position.
	Acn_Enc_Int_PositiveInteger_ConstSize(&pNullPos->Packet_Type_body_length_in_bytes, lengthInBytes, 16);

	//calculate crc
	asn1SccUint crc32 = calc_crc_32(pNullPos->Packet_Type_packet_crc32.buf,
			pNullPos->Packet_Type_packet_crc32.currentByte);
	//encode crc32 in the correct position
	Acn_Enc_Int_PositiveInteger_ConstSize(&pNullPos->Packet_Type_packet_crc32, crc32, 32);

}

flag my_crc_validator(const Packet_Type *pPacket, BitStream *pStartBitStrm,
		Packet_Type_extension_function_positions *pNullPos, BitStream *pEndBitStrm, int *pErrCode) {

	(void) pPacket;
	(void) pStartBitStrm;
	(void) pEndBitStrm;
	flag ret1;
	flag ret2;
	asn1SccUint startPosInBits = pNullPos->Packet_Type_body_length_in_bytes.currentByte * 8
			+ pNullPos->Packet_Type_body_length_in_bytes.currentBit;

	asn1SccUint endPosIBits = pNullPos->Packet_Type_packet_crc32.currentByte * 8
			+ pNullPos->Packet_Type_packet_crc32.currentBit;

	asn1SccUint lengthInBytes = (endPosIBits - startPosInBits - 16) / 8;

	asn1SccUint decodeLengthInBytes = 0;
	//use the ACN function to decode the length value. Please note that we use the Packet_Type_packet_length_in_bytes field in the
	// Packet_Type_extension_function_positions to encode the length in the correct position.
	ret1 = Acn_Dec_Int_PositiveInteger_ConstSize(&pNullPos->Packet_Type_body_length_in_bytes, &decodeLengthInBytes, 16);

	//calculate crc
	asn1SccUint crc32 = calc_crc_32(pNullPos->Packet_Type_packet_crc32.buf,
			pNullPos->Packet_Type_packet_crc32.currentByte);
	asn1SccUint decodeCrc32;
	//decode crc32 from the correct position
	ret2 = Acn_Dec_Int_PositiveInteger_ConstSize(&pNullPos->Packet_Type_packet_crc32, &decodeCrc32, 32);

	flag ret = ret1 && ret2 && (lengthInBytes == decodeLengthInBytes) && (crc32 == decodeCrc32);

	*pErrCode = ret ? 0 : 99999; //custom error code
	return ret;
}
