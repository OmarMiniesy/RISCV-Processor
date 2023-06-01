/*******************************************************************
*
* Module: instructionmem.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Instruction memory.
*
**********************************************************************/
`timescale 1ns / 1ps

module instructionmem(
input [31:0] addr,
output [31:0] data_out
);
    
    reg [7:0] mem [0:63];
    assign data_out = {mem[addr], mem[addr+1], mem[addr+2], mem[addr+3] };
    
    initial begin
// mem[0]=32'b000000000000_00000_010_00001_0000011 ; //lw x1, 0(x0)
// mem[1]=32'b000000000100_00000_010_00010_0000011 ; //lw x2, 4(x0)
// mem[2]=32'b000000001000_00000_010_00011_0000011 ; //lw x3, 8(x0)
// mem[3]=32'b0000000_00010_00001_110_00100_0110011 ; //or x4, x1, x2
// mem[4]=32'b0_000000_00011_00100_000_0100_0_1100011; //beq x4, x3, 4
// mem[5]=32'b0000000_00010_00001_000_00011_0110011 ; //add x3, x1, x2
// mem[6]=32'b0000000_00010_00011_000_00101_0110011 ; //add x5, x3, x2
// mem[7]=32'b0000000_00101_00000_010_01100_0100011; //sw x5, 12(x0)
// mem[8]=32'b000000001100_00000_010_00110_0000011 ; //lw x6, 12(x0)
// mem[9]=32'b0000000_00001_00110_111_00111_0110011 ; //and x7, x6, x1
// mem[10]=32'b0100000_00010_00001_000_01000_0110011 ; //sub x8, x1, x2
// mem[11]=32'b0000000_00010_00001_000_00000_0110011 ; //add x0, x1, x2
// mem[12]=32'b0000000_00001_00000_000_01001_0110011 ; //add x9, x0, x1
    
    $readmemh("C://Users//Omarminiesy//Desktop//Project test 1//project_1//project_1.srcs//sources_1//new//stype_test.mem", mem);
    
    end 

    
endmodule
