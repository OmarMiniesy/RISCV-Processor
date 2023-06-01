`timescale 1ns / 1ps
/*******************************************************************
*
* Module: Register.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: register module to be used for the registers and program
*                counter.
*
**********************************************************************/


module Register #(parameter N=6 )(
input clk, 
input reset, 
input load, 
input [N-1:0] data, 
output reg [N-1:0] Q
);
    
    always @(posedge (clk) or posedge(reset))begin
        if (reset == 1'b1)
            Q <= {N{1'b0}};
        else begin
            if(load == 1'b1)
               Q <= data;
        end
    end 
    
endmodule
