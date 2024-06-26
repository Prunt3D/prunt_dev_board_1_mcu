#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>

#include "asn1crt.h"

#if WORD_SIZE==8
const asn1SccUint64 ber_aux[] = { 
    0xFF,
    0xFF00,
    0xFF0000,
    0xFF000000,
    0xFF00000000ULL,
    0xFF0000000000ULL,
    0xFF000000000000ULL,
    0xFF00000000000000ULL };
#else
const asn1SccUint32 ber_aux[] = {
    0xFF,
    0xFF00,
    0xFF0000,
    0xFF000000 };
#endif

asn1SccUint int2uint(asn1SccSint v) {
    asn1SccUint ret = 0;
    if (v < 0) {
        ret = (asn1SccUint)(-v - 1);
        ret = ~ret;
    }
    else {
        ret = (asn1SccUint)v;
    };
    return ret;
}

asn1SccSint uint2int(asn1SccUint v, int uintSizeInBytes) {
    int i;
    asn1SccUint tmp = 0x80;
    flag bIsNegative = (v & (tmp << ((uintSizeInBytes - 1) * 8)))>0;
    if (!bIsNegative)
        return (asn1SccSint)v;
    for (i = WORD_SIZE - 1; i >= uintSizeInBytes; i--)
        v |= ber_aux[i];
    return -(asn1SccSint)(~v) - 1;
}





/*

#######                                      ###
#     # #####       # ######  ####  #####     #  #####  ###### #    # ##### # ###### # ###### #####
#     # #    #      # #      #    #   #       #  #    # #      ##   #   #   # #      # #      #    #
#     # #####       # #####  #        #       #  #    # #####  # #  #   #   # #####  # #####  #    #
#     # #    #      # #      #        #       #  #    # #      #  # #   #   # #      # #      #####
#     # #    # #    # #      #    #   #       #  #    # #      #   ##   #   # #      # #      #   #
####### #####   ####  ######  ####    #      ### #####  ###### #    #   #   # #      # ###### #    #

Object Identifier

*/








