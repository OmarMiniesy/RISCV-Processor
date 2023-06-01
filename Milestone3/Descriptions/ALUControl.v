/*******************************************************************
*
* Module: ALUControl.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Control unit for the ALU that produces the correct flags
*              for the ALU.
*
**********************************************************************/

`timescale 1ns / 1ps
`include "defines.v"

module ALUControl(
input [2:0] ALUOp,
input [2:0] Inst14_12,
input Inst30,
input immbit,
output reg [4:0] ALUSelection
);
    
    wire [7:0] concat;
    assign concat = {ALUOp, Inst14_12, Inst30, immbit};

    always @ (*) begin
        casex(concat)
        
            7'b000xxxxx: ALUSelection = `ALU_ADD;    // lw and sw and jalr add
            7'b001xxxxx: ALUSelection = `ALU_SUB;    // Branches SUB
            
            // R - TYPE
            {3'b010, `F3_ADD, 1'b1, 1'b0}: ALUSelection = `ALU_SUB;
            {3'b010, `F3_ADD, 1'b0, 1'b0}: ALUSelection = `ALU_ADD;
            {3'b010, `F3_AND, 1'b0, 1'b0}: ALUSelection = `ALU_AND;
            {3'b010, `F3_OR, 1'b0, 1'b0}: ALUSelection = `ALU_OR;
            {3'b010, `F3_XOR, 1'b0, 1'b0}: ALUSelection = `ALU_XOR;
            {3'b010, `F3_SLL, 1'b0, 1'b0}: ALUSelection = `ALU_SLL;
            {3'b010, `F3_SRL, 1'b0, 1'b0}: ALUSelection = `ALU_SRL;
            {3'b010, `F3_SRL, 1'b1, 1'b0}: ALUSelection = `ALU_SRL;
            {3'b010, `F3_SLT, 1'b0, 1'b0}: ALUSelection = `ALU_SLT;
            {3'b010, `F3_SLTU, 1'b0, 1'b0}: ALUSelection = `ALU_SLTU;

            // MULTIPLICATION
            {3'b010, 3'b000, 1'b0, 1'b1}: ALUSelection = `ALU_MUL;
            {3'b010, 3'b001, 1'b0, 1'b1}: ALUSelection = `ALU_MULH;
            {3'b010, 3'b010, 1'b0, 1'b1}: ALUSelection = `ALU_MULHSU;
            {3'b010, 3'b011, 1'b0, 1'b1}: ALUSelection = `ALU_MULHU;
            {3'b010, 3'b100, 1'b0, 1'b1}: ALUSelection = `ALU_DIV;
            {3'b010, 3'b101, 1'b0, 1'b1}: ALUSelection = `ALU_DIVU;
            {3'b010, 3'b110, 1'b0, 1'b1}: ALUSelection = `ALU_REM;
            {3'b010, 3'b111, 1'b0, 1'b1}: ALUSelection = `ALU_REMU;
            
             
            // I - TYPE
            {3'b011, `F3_ADD, 1'bx, 1'bx}: ALUSelection = `ALU_ADD;
            {3'b011, `F3_XOR, 1'bx, 1'bx}: ALUSelection = `ALU_XOR;
            {3'b011, `F3_OR, 1'bx, 1'bx}: ALUSelection = `ALU_OR;
            {3'b011, `F3_AND, 1'bx, 1'bx}: ALUSelection = `ALU_AND;
            {3'b011, `F3_SLL, 1'b0, 1'bx}: ALUSelection = `ALU_SLL;
            {3'b011, `F3_SRL, 1'b0, 1'bx}: ALUSelection = `ALU_SRL;
            {3'b011, `F3_SRL, 1'b1, 1'bx}: ALUSelection = `ALU_SRA;
            {3'b011, `F3_SLT, 1'bx, 1'bx}: ALUSelection = `ALU_SLT;
            {3'b011, `F3_SLTU, 1'bx, 1'bx}: ALUSelection = `ALU_SLTU;


            {3'b100, 3'bxxx, 2'bx}: ALUSelection = `ALU_PASS;  // LUI
            {3'b000, 3'bxxx, 2'bx}: ALUSelection = `ALU_PASS;  // auipc
            

            
            default :ALUSelection=`ALU_PASS;
        endcase    
    end
endmodule
