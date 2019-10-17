/*
 *  This file contains some general purpose function
 */

#include "util_functions.h"

uint32_t convert_uint8_array_to_uint32(const uint8_t* array){
	uint32_t out = *(uint32_t*)array;
	return out;
}

uint16_t convert_uint8_array_to_uint16(const uint8_t* array){
	uint16_t out = *(uint16_t*)array;
	return out;
}
