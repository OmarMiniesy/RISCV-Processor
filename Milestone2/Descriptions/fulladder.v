/*******************************************************************
*
* Module: fulladder.v
* Project: Single Cycle RISC-V Processor
* Author: Omar Miniesy - Ziad Miniesy
* Description: Bit full adder.
*
**********************************************************************/
`timescale 1ns / 1ps

module fulladder(
a, b, cin, sum, cout
    );
    
    input a, b, cin;
    output sum, cout;

    wire w1, w2, w3;

    xor g1 (w1, a, b);
    xor g2 (sum, w1, cin);
    and g3 (w2, w1, cin);
    and g4 (w3, a, b);
    or g5 (cout, w2, w3);
    
endmodule
