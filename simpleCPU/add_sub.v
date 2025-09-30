`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/21/2024 04:23:32 PM
// Design Name: 
// Module Name: add_sub
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


// Module definition for a parameterized adder-subtractor.
// This module can perform either addition or subtraction based on the addsub control signal.
module add_sub #( 
    parameter N = 8  // Parameter N defines the width of the data inputs and output. Default is 8 bits.
)(
    input addsub,            // Control input: Determines operation mode (0 for addition, 1 for subtraction).
    input [N-1:0] x0,        // First input operand.
    input [N-1:0] x1,        // Second input operand.
    output reg [N-1:0] result  // Output: Result of the operation, either x0 + x1 or x0 - x1.
);

    // Combinational logic block: Sensitive to changes in the inputs.
    always @(*) begin
        if (addsub == 0)
            result = x1 + x0;  // Perform addition when addsub is 0.
        else
            result = x0 - x1;  // Perform subtraction when addsub is 1.    
    end
    
endmodule




