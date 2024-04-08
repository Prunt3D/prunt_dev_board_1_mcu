#ifndef POSTENCODING_H
#define POSTENCODING_H
#include "asn1crt.h"
#include "mcu_comms.h"

void my_encoding_patcher(const Packet_Type *pPacket, BitStream *pStartBitStrm,
		Packet_Type_extension_function_positions *pNullPos, BitStream *pEndBitStrm);

flag my_crc_validator(const Packet_Type *pPacket, BitStream *pStartBitStrm,
		Packet_Type_extension_function_positions *pNullPos, BitStream *pEndBitStrm, int *pErrCode);

#endif // POSTENCODING_H
