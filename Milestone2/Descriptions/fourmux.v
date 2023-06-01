/*******************************************************************
*
* Module: fourmux.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Four input multiplexer.
*
**********************************************************************/

`timescale 1ns / 1ps

module fourmux # (parameter  N= 32)(
input [N-1:0] i0,
input [N-1:0] i1,
input [N-1:0] i2,
input [N-1:0] i3,
input [1:0] sel,
output [N-1:0] out
);
    
    assign out = sel[1] ? (sel[0] ? i3:i2) : (sel[0] ? i1:i0);
    
endmodule
