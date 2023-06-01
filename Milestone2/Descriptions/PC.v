`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/07/2023 09:51:01 AM
// Design Name: 
// Module Name: PC
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module PC #(parameter N=6 )(
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
