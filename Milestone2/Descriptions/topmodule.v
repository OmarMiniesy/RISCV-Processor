/*******************************************************************
*
* Module: topmodule.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Top module.
*
**********************************************************************/

`timescale 1ns / 1ps
`include "defines.v"

module topmodule(
input clk, 
input ssd_clock, 
input reset,  
input [1:0] ledSel, 
input [3:0] ssdSel,  
output reg [15:0] LED, 
output [3:0] Anode,
output [6:0] LED_out 
);
    
  reg [12:0] Switches ;
   bcd ssd(.clk(ssd_clock), .num(Switches), .Anode(Anode), .LED_out(LED_out));
    always @ (*) begin
        case(ssdSel)
            4'b0000: Switches = currentAddress[12:0];
            4'b0001: Switches = adderResult[12:0];
            4'b0010: Switches = Sum[12:0];
            4'b0011: Switches = nextAddress[12:0]; 
            4'b0100: Switches = ReadData1[12:0];
            4'b0101: Switches = ReadData2[12:0];
            4'b0110: Switches = writeData[12:0];
            4'b0111: Switches = immediate[12:0]; 
            4'b1000: Switches = shiftLeftOutput[12:0]; 
            4'b1001: Switches = ALUinput2[12:0]; 
            4'b1010: Switches = ALUResult[12:0]; 
            4'b1011: Switches = writeData[12:0];
        endcase 
    end 
   
    always @ (*) begin
        case(ledSel)
            2'b00: LED = Instruction[15:0]; 
            2'b01: LED = Instruction[31:16];
            2'b10: LED = {{1{1'b0}}, ALUop, ALUselection,  branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite, zeroFlag, selForPC };
            default: LED = 16'd0;
        endcase 
    end
    
    //PROGRAM COUNTER
    wire [31:0] currentAddress; //output beta3 el PC
    wire [31:0] nextAddress;    // input beta3 el PC
    PC #(32) pc (.clk(clk), .reset(reset), .load(pcload), .data(nextAddress), .Q(currentAddress));
    
    //INSTRUCTION MEMORY
    wire [31:0] Instruction;   // output beta3 el instruction memory
    wire [6:0] opcode;         //wire divisions from IM to control
    wire [4:0] rs1;          //wire divisions from IM to RF reg1
    wire [4:0] rs2;          //wire divisions from IM to RF reg2
    wire [4:0] rd;            //wire divisions from IM to RF rd;
    instructionmem InstructionMemory(.addr(currentAddress[31:0]), .data_out(Instruction));
    assign opcode =  Instruction [6:0];
    assign rs1= Instruction [19:15];
    assign rs2= Instruction [24:20];
    assign rd= Instruction [11:7];
     
    // CONTROL 
    wire branch;               
    wire MemRead;              //OUTPUTS
    wire MemtoReg;
    wire [2:0] ALUop;
    wire MemWrite;
    wire ALUsrc;
    wire RegWrite;
    wire auipc;
    wire jump;
    wire [1:0] srcPC;
    wire pcload;
    wire immbit = Instruction[26];
    control Control(.inst(Instruction[6:2]), .Branch(branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUop), .MemWrite(MemWrite), .ALUsrc(ALUsrc), .RegWrite(RegWrite), .auipc(auipc), .jump(jump), .srcPC(srcPC), .pcload(pcload), .immbit(immbit));
    
    // ALU CONTROL
    wire [3:0] ALUselection;   //output of ALUControl into ALU from Control
    ALUControl ALUControl(.ALUOp(ALUop), .Inst14_12(Instruction[14:12]), .Inst30(Instruction[30]), .ALUSelection(ALUselection));
    
    // IMMEDIATE GENERATOR
    wire signed [31:0] immediate;     //output of Immediate Generator
    rv32_ImmGen immediateGenerator(.IR(Instruction), .Imm(immediate)); 
    
    // SHIFT LEFT 1 
    wire signed [31:0] shiftLeftOutput;
    shift #(32) ShiftLeft (.in(immediate), .out(shiftLeftOutput));
    
    //MUX FOR REGISTER FILE TO CHOOSE BETWEEN AUIPC AND WRITE DATA, AND JALR/JAL
    wire [31:0] inputrd;
    wire [1:0] sel = {auipc, jump};
    fourmux #(32) inputRd (.i0(writeData), .i1(adderResult), .i2(Sum), .i3(32'd0), .sel(sel), .out(inputrd));
    
    // REGISTER FILE 
    wire signed [31:0] ReadData1; //outputs of the Register File
    wire signed [31:0] ReadData2;
    registerFile #(32) registerFile(.clk(clk), .rst(reset), .ReadReg1(rs1), .ReadReg2(rs2), .WriteReg(rd), .write(RegWrite), .data(inputrd), .reg1(ReadData1), .reg2(ReadData2));
    
    // MUX FOR CHOOSING THE INPUT OF THE ALU
    wire signed [31:0] ALUinput2;
    mux #(32) RFALU(.i0(ReadData2), .i1(immediate), .sel(ALUsrc), .out(ALUinput2));
    
    //ALU
    wire signed [31:0] ALUResult;
    wire zeroFlag, cf, vf, sf ;
    prv32_ALU  ALU(.alufn(ALUselection), .shamt(ALUinput2[4:0]), .a(ReadData1), .b(ALUinput2), .r(ALUResult), .zf(zeroFlag), .cf(cf), .vf(vf), .sf(sf));
    
    //DATA MEMORY
    wire [31:0] ReadData;
    DataMemory DataMemory(.clk(clk), .MemRead(MemRead), .MemWrite(MemWrite), .addr(ALUResult[7:0]), .data_in(ReadData2), .data_out(ReadData), .funct3(Instruction[14:12]));
    
    //MUX FOR WRITING BACK TO REGISTER FILE FROM DATA MEMORY
    wire [31:0] writeData;
    mux #(32) DMRF(.i0(ALUResult), .i1(ReadData), .sel(MemtoReg), .out(writeData));
    
    //PC + 4
    wire [32:0] temp1;
    wire [31:0] adderResult;
    adder #(32) PcAdder(.a(currentAddress), .b(32'd4), .sum(temp1));
    assign adderResult = temp1[31:0]; 
    
    //SHIFTED IMMEDIATE + PC
    wire [32:0] temp2;
    wire [31:0] Sum;
    adder #(32) PcAdderShift(.a(currentAddress), .b(immediate), .sum(temp2));
    assign Sum = temp2[31:0];
    
    //FINAL MUX FOR CHANGING VALUE OF PC
    wire selForPC;
    branchmod brancher(.zf(zeroFlag), .vf(vf), .sf(sf), .cf(cf), .funct3(Instruction[14:12]), .branch(branch), .branchResult(selForPC));
    
    wire selForBranchjal = selForPC | jump;
    wire [31:0] selForFinalPC;
    mux #(32) firstmux(.i0(adderResult), .i1(Sum), .sel(selForBranchjal), .out(selForFinalPC));
           
    fourmux #(32) ADDPCBRANCH(.i0(adderResult), .i1(selForFinalPC), .i2(ALUResult), .i3(32'd0), .sel(srcPC), .out(nextAddress));

    
endmodule
