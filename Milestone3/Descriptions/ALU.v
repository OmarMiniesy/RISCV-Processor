/*******************************************************************
*
* Module: ALU.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: ALU unit for the RISC-V processor.
*
**********************************************************************/
`timescale 1ns / 1ps

module prv32_ALU(
input   wire [31:0] a, b,
input   wire [4:0]  shamt,
output  reg  [31:0] r,
output  wire        cf, zf, vf, sf,
input   wire [4:0]  alufn
);

    wire [31:0] add, sub, op_b;
    wire cfa, cfs;
    
    assign op_b = (~b);
    
    assign {cf, add} = alufn[0] ? (a + op_b + 1'b1) : (a + b);
    
    assign zf = (add == 0);
    assign sf = add[31];
    assign vf = (a[31] ^ (op_b[31]) ^ add[31] ^ cf);
    
    wire[31:0] sh;
    shiftermod shifter0(.a(a), .shamt(shamt), .type(alufn[1:0]),  .r(sh));
    
    reg [63:0] multiplication;
    
    always @ * begin
        r = 0;
        (* parallel_case *)
        case (alufn)
            // arithmetic
            5'b000_00 : r = add;
            5'b000_01 : r = add;
            5'b000_11 : r = b;
            // logic
            5'b001_00:  r = a | b;
            5'b001_01:  r = a & b;
            5'b001_11:  r = a ^ b;
            // shift
            5'b010_00:  r=sh;
            5'b010_01:  r=sh;
            5'b010_10:  r=sh;
            // slt & sltu
            5'b011_01:  r = {31'b0,(sf != vf)}; 
            5'b011_11:  r = {31'b0,(~cf)};  
            //Multiplication
            5'b100_00:  r = $signed(a) * $signed(b);
            5'b100_01: begin multiplication = $signed(a) * $signed(b); r = multiplication[63:32];end
            5'b100_10: begin multiplication = $signed(a) * b; r= multiplication[63:32];end
            5'b100_11: begin multiplication = a * b; r = multiplication[63:32];end
            //Division
            5'b110_00: r = $signed(a) / $signed(b);
            5'b110_01: r = a / b;
            5'b110_10: r = $signed(a) % $signed(b);
            5'b110_11: r = a % b;	
        endcase
    end
endmodule
