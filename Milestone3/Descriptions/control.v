/*******************************************************************
*
* Module: control.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Control unit for the RISC-V processor.
*
**********************************************************************/

`timescale 1ns / 1ps


module control(
input immbit,
input [4:0] inst,
output reg flush,
output reg Branch,
output reg MemRead,
output reg MemtoReg,
output reg [2:0] ALUOp,
output reg MemWrite,
output reg ALUsrc,
output reg RegWrite,
output reg auipc,
output reg jump,
output reg [1:0] srcPC,
output reg pcload
);
    
    always @ (*) begin
        if(inst == 5'b01100)begin       //r-type
            Branch = 0; 
            MemRead = 0;
            MemtoReg =0; 
            ALUOp = 3'b010; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0; 
            flush = 0;
        end 
        else if (inst == 5'b00000)begin  //load
            Branch = 0; 
            MemRead = 1; 
            MemtoReg =1; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0; 
            flush = 0;
        end 
        else if (inst == 5'b01000)begin  //stores
            Branch = 0; 
            MemRead = 0; 
            ALUOp = 3'b000; 
            MemWrite = 1; 
            ALUsrc = 1; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0;
            flush = 0; 
        end  
        else if (inst == 5'b11000)begin  //branch 
            Branch = 1; 
            MemRead = 0; 
            ALUOp = 3'b001; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b01; 
            pcload = 0;
            flush = 0; 
        end  
        else if (inst == 5'b00100)begin  //i-type
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b011; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0;
            flush = 0; 
        end  
        else if (inst == 5'b01101)begin  //LUI
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b100; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0;
            flush = 0; 
        end   
        else if (inst == 5'b00101)begin  //auipc
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 1; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0;
            flush = 0; 
        end   
        else if (inst == 5'b11001)begin  // jalr
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 1; 
            srcPC =2'b10; 
            pcload = 0;
            flush = 1; 
        end  
        else if (inst == 5'b11011)begin  //jal
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 1; 
            RegWrite = 1; 
            auipc = 0; 
            jump = 1; 
            srcPC =2'b01; 
            pcload = 0; 
            flush = 1;
        end  
        else if (inst == 5'b11100 && immbit==1'b1)begin //ebreak
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 1;
            flush = 1; 
        end  
        else if (inst == 5'b11100 && immbit==1'b0)begin //ecall
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b11; 
            pcload = 0;
            flush = 1; 
        end  
        else if (inst == 5'b00011)begin  //fence
            Branch = 0; 
            MemRead = 0; 
            MemtoReg =0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b11; 
            pcload = 0;
            flush = 1; 
        end  
        else begin                       //default
            Branch = 0; 
            MemRead = 0; 
            ALUOp = 3'b000; 
            MemWrite = 0; 
            ALUsrc = 0; 
            RegWrite = 0; 
            auipc = 0; 
            jump = 0; 
            srcPC =2'b00; 
            pcload = 0;
            flush = 0;
        end 
    end
    
endmodule
