/*******************************************************************
*
* Module: shift.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Shift left by one.
*
**********************************************************************/

`timescale 1ns / 1ps

module shift #(parameter N=32)(
input [N-1:0] in,
output [N-1:0] out
);
    assign out = in * 2;
    
endmodule