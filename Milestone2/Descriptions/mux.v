/*******************************************************************
*
* Module: mux.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: two input multiplexer.
*
**********************************************************************/

`timescale 1ns / 1ps


module mux # (parameter  N= 32)(
input [N-1:0] i0,
input [N-1:0] i1,
input sel,
output [N-1:0] out
);
    assign out = sel ? i1:i0 ;
endmodule
