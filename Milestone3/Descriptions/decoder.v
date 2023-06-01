`timescale 1ns / 1ps
/*******************************************************************
*
* Module: DataMemory.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: decoder for the RISC-V processor.
*
**********************************************************************/


module decoder(
    input clk, 
    input [31:0] i,
    output  reg [31:0] a,
    output  reg [31:0] b
    );
    
    always @(*) begin
    if (clk) a=i; else b=i;
    end
    
endmodule
