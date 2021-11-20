#pragma once
#define P__Z6kernelPlS_S__dfg_0_sub0_v0_0_ 0
#define P__Z6kernelPlS_S__dfg_0_sub0_v1_0_ 1

#define P__Z6kernelPlS_S__dfg_0_sub0_v3_ 1

// -------- EDGE:0, extra_lat = 0 -------- 
//	config SW0
//		route input port 0 to output port 2
//	config FU0
//		add extra delay 0 for operand 0
//		route input port 0 to operand 0
//		set current opcode to 1 means Add_I64
// -------- EDGE:1, extra_lat = 0 -------- 
//	config SW1
//		route input port 2 to output port 2
//	config FU0
//		add extra delay 0 for operand 1
//		route input port 1 to operand 1
//		set current opcode to 1 means Add_I64
// -------- EDGE:2, extra_lat = 0 -------- 
//	config SW3
//		route input port 2 to output port 1
//	config FU0
//		route result 0 to output port 0

#define _Z6kernelPlS_S__dfg_0_size 7

char _Z6kernelPlS_S__dfg_0_config[_Z6kernelPlS_S__dfg_0_size] = {
		0b00000000, // RoCC L1D memory ignore first
		0b0000000000000000100000000000000000010000001000110010000001000111, //FU0
		0b0100000000000000000000000000000000000000000000000000000000000001, //SW0
		0b0100000000000001000000000000000000000000010010010010010010010010, //SW0
		0b0100000001000000000000000000000000000000000000000000000000000001, //SW1
		0b0100000001000001000000000000000000000000110110110110110110110110, //SW1
		0b0100000011000000000000000000000111111111111111100000000000000001, //SW3
};