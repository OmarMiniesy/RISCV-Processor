/*******************************************************************
*
* Module: DataMemory.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Data memory for the RISC-V processor.
*
**********************************************************************/
`timescale 1ns / 1ps



module DataMemory(
input clk, 
input MemRead, 
input MemWrite,
input [2:0] funct3, 
input [5:0] addr, 
input [31:0] data_in, 
output reg [31:0] data_out
);

    reg [7:0] mem [0:63];
    
    initial begin
        mem[0] =8'h4; 
        mem[1] =8'h0; 
        mem[2] =8'h0; 
        mem[3] =8'h0;
    
        mem[4] =8'h2; 
        mem[5] =8'h0; 
        mem[6] =8'h0; 
        mem[7] =8'h0;
    
        mem[8] =8'h1; 
        mem[9] =8'h0; 
        mem[10] =8'h0; 
        mem[11] =8'h0; 
    end 
    
    always @ (*) begin
        if(MemRead) begin
        case (funct3)
            3'b000:  //load byte
            data_out = $signed({mem[addr]});    
    
            3'b001: // load half word
            data_out = $signed({mem[addr+1], mem[addr]});  
    
            3'b010: // load word
            data_out = $signed({mem[addr+3],mem[addr+2],mem[addr+1],mem[addr]}); 
    
            3'b100: // load byte unsigned
            data_out = mem[addr];              
    
            3'b101: //load half word unsigned
            data_out = {mem[addr+1], mem[addr]};
    
            default: data_out = 32'hZZZZZZZZ;
        endcase
        end else begin
        data_out = 32'hZZZZZZZZ;
        end
    end
    
    always @ (posedge clk) begin
        if(MemWrite) begin
            case (funct3)
                3'b000: //store byte
                mem[addr] = data_in;
    
                3'b001: //store half word
                {mem[addr+1], mem[addr]} = data_in;    
    
                3'b010: //store word
                {mem[addr+3],mem[addr+2],mem[addr+1],mem[addr]}= data_in;     
    
                default: mem[addr] = mem[addr];
            endcase
        end else begin
            mem[addr] = mem[addr];
        end
    end

endmodule