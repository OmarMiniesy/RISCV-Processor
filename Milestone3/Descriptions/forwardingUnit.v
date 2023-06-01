/*******************************************************************
*
* Module: DataMemory.v
* Project: Pipelined RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Forwarding Unit for the RISC-V processor.
*
**********************************************************************/


module forwardingUnit (
input [4:0] ID_EXE_rs1,
input [4:0] ID_EXE_rs2,
input MEM_WB_regWrite,
input [4:0] MEM_WB_regRd,
output reg forwardA,
output reg forwardB
    );
    always @(*)
    begin

        if( (MEM_WB_regWrite) && (MEM_WB_regRd != 5'd0) && (MEM_WB_regRd == ID_EXE_rs1))
        begin  
		forwardA = 1'b1; 
	end
        else 
        	forwardA =1'b0; 
    
        if( (MEM_WB_regWrite) && (MEM_WB_regRd != 5'd0) && (MEM_WB_regRd == ID_EXE_rs2))
        begin 
		forwardB = 1'b1; 
	end
        else
       		forwardB = 1'b0;
        end   
endmodule
