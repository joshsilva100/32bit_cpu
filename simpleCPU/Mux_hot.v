`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/21/2024 04:24:06 PM
// Design Name: 
// Module Name: Mux_hot
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




// Define a parameterized hot-one (one-hot) multiplexer module. This type of multiplexer selects one
// of the multiple input signals to pass to the output based on a one-hot encoding scheme.
module Mux_hot #(
    parameter N = 8  // Parameter N defines the width of the data inputs and output. Default is 8 bits.
)(
    input [9:0] select,         // One-hot select signal, determines which input to pass to the output.
    input [N-1:0] s0,           // Input signal 0
    input [N-1:0] s1,           // Input signal 1
    input [N-1:0] s2,           // Input signal 2
    input [N-1:0] s3,           // Input signal 3
    input [N-1:0] s4,           // Input signal 4
    input [N-1:0] s5,           // Input signal 5
    input [N-1:0] s6,           // Input signal 6
    input [N-1:0] s7,           // Input signal 7
    input [N-1:0] din,          // Default input signal (used when no valid select is active)
    input [N-1:0] g,            // Additional input signal (for example purposes)
    output reg [N-1:0] result   // Output signal, selected from one of the inputs based on 'select'
);

   // Combinational logic to determine the output based on the 'select' signal
   always @(*)
   begin
       case (select)
           10'b0000000001: result <= din;  // When select line is 0000000001, output is 'din'
           10'b0000000010: result <= s0;   // When select line is 0000000010, output is 's0'
           10'b0000000100: result <= s1;   // When select line is 0000000100, output is 's1'
           10'b0000001000: result <= s2;   // When select line is 0000001000, output is 's2'
           10'b0000010000: result <= s3;   // When select line is 0000010000, output is 's3'
           10'b0000100000: result <= s4;   // When select line is 0000100000, output is 's4'
           10'b0001000000: result <= s5;   // When select line is 0001000000, output is 's5'
           10'b0010000000: result <= s6;   // When select line is 0010000000, output is 's6'
           10'b0100000000: result <= s7;   // When select line is 0100000000, output is 's7'
           10'b1000000000: result <= g;    // When select line is 1000000000, output is 'g'
           default:        result <= din;  // Default case, output is 'din' if no valid select
       endcase
   end   
endmodule




