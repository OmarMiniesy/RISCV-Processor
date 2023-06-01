/*******************************************************************
*
* Module: topmodule.v
* Project: Pipelined Risc-V Processor
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
    reg [1:0] counter;
    wire sclk;
    wire [31:0] currentAddress; //output beta3 el PC
    wire [31:0] nextAddress;    // input beta3 el PC
    wire [68:0] instructionMuxIn;
    wire [68:0] dataMuxIn;
    wire [68:0] muxOut;
    wire [31:0] IF_ID_PC, IF_ID_Inst;
    wire selForInstMux;
    wire [31:0] InstructionMuxOut;
    wire [3:0]funct3 ;
    wire [6:0] opcode;         //wire divisions from IM to control
    wire [4:0] rs1;          //wire divisions from IM to RF reg1
    wire [4:0] rs2;          //wire divisions from IM to RF reg2
    wire [4:0] rd;            //wire divisions from IM to RF module adder 
    wire forwardA;
    wire forwardB;
    wire flush;
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
    wire [14:0] newControl;
    wire immbit;
    wire immbitMult;
    wire [31:0] inputrd;
    wire [1:0] sel = {MEM_WB_Ctrl[4], MEM_WB_Ctrl[3]};  // auipc or jump
    wire signed [31:0] ReadData1; //outputs of the Register File
    wire signed [31:0] ReadData2;
    wire signed [31:0] immediate;     //output of Immediate Generator
    wire [14:0] controlOrNop;
    wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [14:0] ID_EX_Ctrl;
    wire [3:0] ID_EX_Func;
    wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
    wire ID_EX_immediateBit;
    wire [4:0] ALUselection;   //output of ALUControl into ALU from Control
    wire signed [31:0] shiftLeftOutput;
    wire [31:0] alufirstmuxinput;
    wire signed [31:0] ALUinput2;
    wire [31:0] ALUinput1;
    wire signed [31:0] ALUResult;
    wire zeroFlag, cf, vf, sf ;
    wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_PC;
    wire [14:0] EX_MEM_Ctrl;
    wire [4:0] EX_MEM_Rd;
    wire [3:0] EX_MEM_func;
    wire EX_MEM_Zero, EX_MEM_cf, EX_MEM_vf, EX_MEM_sf;
    wire [31:0] ReadData;
    wire [31:0] memInstruction;
    wire [31:0] memData;
    wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out, MEM_WB_PC;
    wire [14:0] MEM_WB_Ctrl;
    wire [4:0] MEM_WB_Rd;
    wire [31:0] writeData;
    wire [32:0] temp1;
    wire [31:0] adderResult;
    wire [32:0] temp2;
    wire [31:0] Sum;
    wire selForPC;
    wire selForBranchjal = selForPC | EX_MEM_Ctrl[3];
    wire [31:0] selForFinalPC;

    
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
            2'b00: LED = memInstruction[15:0]; 
            2'b01: LED = memInstruction[31:16];
            2'b10: LED = {ALUop, ALUselection,  branch, MemRead, MemtoReg, 
            MemWrite, ALUsrc, RegWrite, zeroFlag, selForPC };
            default: LED = 16'd0;
        endcase 
    end
    
    initial begin counter = 2'b00; end
    always @(posedge clk) begin
        if (counter != 2'b11) counter=counter+1;
        else counter = 2'b00;
    end
    assign sclk = counter[1];
    
    
    //PROGRAM COUNTER
    Register #(32) pc (.clk(!clk/*sclk*/), .reset(reset),
    .load(~EX_MEM_Ctrl[0]), .data(nextAddress), .Q(currentAddress));
    
    // MUX BETWEEN NOP AND INSTRUCTION
    assign selForInstMux = selForPC | EX_MEM_Ctrl[14];
    mux #(32) instructionMuxNop (.i0(memInstruction), .i1(32'h33),
    .sel(selForInstMux), .out(InstructionMuxOut));
    
    // mux for memory to access instruction or data
    assign instructionMuxIn = {1'b1, 1'b0, 3'b010, currentAddress, 32'd0};
    assign dataMuxIn = {EX_MEM_Ctrl[12], EX_MEM_Ctrl[7], EX_MEM_func[2:0],
    EX_MEM_ALU_out, EX_MEM_RegR2};
    mux #(69) muxForMem(.i0(instructionMuxIn), .i1(dataMuxIn), .sel(~clk), .out(muxOut));
    
    // IF / ID REGISTER
    Register #(64) IF_ID (~clk/*sclk*/,reset,~EX_MEM_Ctrl[0],
    {currentAddress,  InstructionMuxOut},
    {IF_ID_PC,IF_ID_Inst} );

    assign opcode =  IF_ID_Inst [6:0];
    assign rs1= IF_ID_Inst [19:15];
    assign rs2= IF_ID_Inst [24:20];
    assign rd= IF_ID_Inst [11:7];
    assign funct3 = {IF_ID_Inst[30], IF_ID_Inst[14:12]}; 
     
   //FORWARDING UNIT
    forwardingUnit forwarding (.ID_EXE_rs1(ID_EX_Rs1), .ID_EXE_rs2(ID_EX_Rs2),
    .MEM_WB_regRd(MEM_WB_Rd), .MEM_WB_regWrite(MEM_WB_Ctrl[5]),
    .forwardA(forwardA), .forwardB(forwardB) );
      
    // CONTROL 
    assign immbit = IF_ID_Inst[20];
    assign immbitMult  = IF_ID_Inst[25];
    assign newControl = {flush, branch, MemRead, MemtoReg, ALUop,
    MemWrite, ALUsrc, RegWrite, auipc, jump, srcPC, pcload};
    control Control(.inst(IF_ID_Inst[6:2]), .Branch(branch),
    .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUop),
    .MemWrite(MemWrite), .ALUsrc(ALUsrc), .RegWrite(RegWrite),
    .auipc(auipc), .jump(jump), .srcPC(srcPC), .pcload(pcload),
    .immbit(immbit), .flush(flush));
    
    //MUX FOR REGISTER FILE TO CHOOSE BETWEEN AUIPC AND WRITE DATA, AND JALR/JAL
    fourmux #(32) inputRd (.i0(writeData), .i1(EX_MEM_PC + 32'd4),
    .i2(MEM_WB_ALU_out + MEM_WB_PC), .i3(32'd0), .sel(sel), .out(inputrd));
        
    // REGISTER FILE 
    registerFile #(32) registerFile(.clk(!clk), .rst(reset),
    .ReadReg1(rs1), .ReadReg2(rs2), .WriteReg(MEM_WB_Rd), .write(MEM_WB_Ctrl[5]),
    .data(inputrd), .reg1(ReadData1), .reg2(ReadData2)); 
    
    // IMMEDIATE GENERATOR
    rv32_ImmGen immediateGenerator(.IR(IF_ID_Inst), .Imm(immediate)); 
    
    // MUX FOR 0's in the control
    mux #(15) muxControl(.i0(newControl),.i1(15'd0),.sel(selForInstMux),.out(controlOrNop));   
    
    // ID / EXE REGISTER
    Register #(170) ID_EX (clk/*sclk*/,selForInstMux | reset,~EX_MEM_Ctrl[0],
    {controlOrNop,IF_ID_PC,ReadData1,ReadData2,immediate,
    funct3,rs1,rs2,rd,immbitMult},
    {ID_EX_Ctrl,ID_EX_PC,ID_EX_RegR1,ID_EX_RegR2,
    ID_EX_Imm, ID_EX_Func,ID_EX_Rs1,ID_EX_Rs2,ID_EX_Rd, ID_EX_immediateBit} );    
    // ALU CONTROL
    ALUControl ALUControl(.ALUOp(ID_EX_Ctrl[10:8]),
    .Inst14_12(ID_EX_Func[2:0]), .Inst30(ID_EX_Func[3]),
    .immbit(ID_EX_immediateBit), .ALUSelection(ALUselection));
    
    // SHIFT LEFT 1 
    shift #(32) ShiftLeft (.in(ID_EX_Imm), .out(shiftLeftOutput));
   
   // MUX FOR ALU INPUT 2   rs2/write data/ alu result(forward)
   mux #(32) input2mux(.i0(ID_EX_RegR2), .i1(inputrd), .sel(forwardB),
   .out(alufirstmuxinput));
   
    // MUX FOR CHOOSING THE INPUT2 OF THE ALU
    mux #(32) RFALU(.i0(alufirstmuxinput), .i1(ID_EX_Imm), .sel(ID_EX_Ctrl[6]),
    .out(ALUinput2));
    
    // MUF FOR alu input 1 (rs1, write data, alu result)
    mux #(32) ALU_input2_mux(.i0(ID_EX_RegR1), .i1(inputrd), .sel(forwardA),
    .out(ALUinput1)); 
    
    //ALU
    prv32_ALU  ALU(.alufn(ALUselection), .shamt(ALUinput2[4:0]),
    .a(ALUinput1), .b(ALUinput2), .r(ALUResult), .zf(zeroFlag),
    .cf(cf), .vf(vf), .sf(sf));
    
     // EX / MEM REGISTER
    Register #(170) EX_MEM (~clk/*sclk*/,reset,~EX_MEM_Ctrl[0],
    {ID_EX_Ctrl, Sum, zeroFlag, cf, vf, sf, ALUResult, alufirstmuxinput,
    ID_EX_Rd, ID_EX_Func, ID_EX_PC},
    {EX_MEM_Ctrl, EX_MEM_BranchAddOut, EX_MEM_Zero, EX_MEM_cf,
    EX_MEM_vf, EX_MEM_sf,  
    EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd, EX_MEM_func, EX_MEM_PC} );
    
    //MEMORY
    DataMemory memory(.clk(clk), .MemRead(muxOut[68]), .MemWrite(muxOut[67]),
    .funct3(muxOut[66:64]), .addr(muxOut[63:32]), .data_in(muxOut[31:0]),
    .data_out(ReadData));
    
    //Decoder for memory output
    decoder memDecoder(.clk(clk), .i(ReadData), .a(memInstruction), .b(memData));
    
    // MEM / WB  REGISTER
    Register #(130) MEM_WB (clk/*sclk*/,reset,1'b1,
    {EX_MEM_Ctrl, memData, EX_MEM_ALU_out, EX_MEM_Rd, EX_MEM_PC},
    {MEM_WB_Ctrl,MEM_WB_Mem_out, MEM_WB_ALU_out,
    MEM_WB_Rd, MEM_WB_PC} );
    
    //MUX FOR WRITING BACK TO REGISTER FILE FROM DATA MEMORY
    mux #(32) DMRF(.i0(MEM_WB_ALU_out), .i1(MEM_WB_Mem_out),
    .sel(MEM_WB_Ctrl[11]), .out(writeData));
    
    //PC + 4
    adder #(32) PcAdder(.a(currentAddress), .b(32'd4), .sum(temp1));
    assign adderResult = temp1[31:0]; 
    
    //SHIFTED IMMEDIATE + PC
    adder #(32) PcAdderShift(.a(ID_EX_PC), .b(ID_EX_Imm), .sum(temp2));
    assign Sum = temp2[31:0];
    
    //FINAL MUX FOR CHANGING VALUE OF PC
    branchmod brancher(.zf(EX_MEM_Zero), .vf(EX_MEM_vf), .sf(EX_MEM_sf), .cf(EX_MEM_cf), 
    .funct3(EX_MEM_func[2:0]), .branch(EX_MEM_Ctrl[13]), .branchResult(selForPC));
    

    mux #(32) firstmux(.i0(adderResult), .i1(EX_MEM_BranchAddOut),
    .sel(selForBranchjal), .out(selForFinalPC));
           
    fourmux #(32) ADDPCBRANCH(.i0(adderResult), .i1(selForFinalPC),
    .i2(EX_MEM_ALU_out), .i3(32'd0), .sel(EX_MEM_Ctrl[2:1]), .out(nextAddress));

    
endmodule
