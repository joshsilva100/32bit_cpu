`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/21/2024 04:23:02 PM
// Design Name: 
// Module Name: reg_enabled
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


// Define a parameterized register module with enable control.
// This module can be used for registers of any size as specified by the parameter N.
module reg_enabled #(
    parameter N=8  // Parameter N defines the width of the data input and output. The default is 8 bits.
)(
    input clk,          // Clock input: Used to synchronize the data transfer on the rising edge.
    input rst,          // Reset input: When high (positive edge detected), it resets the output to 0.
    input enable,       // Enable input: When high, allows data to be latched from din to dout.
    input [N-1:0] din,  // Data input: The data to be latched into the register when enabled.
    output reg [N-1:0] dout  // Data output: The current value stored in the register.
);

    // Sequential logic block: Sensitive to the rising edge of the clock or reset signal.
    always @(posedge clk or posedge rst)
    begin
        if(rst)  // Check if reset signal is high.
            dout <= 0;  // Asynchronously reset the output to 0.
        else if (clk == 1) begin  // Check if the clock signal is high (redundant due to posedge clk trigger).
            if(enable)  // Check if the enable signal is high.
                dout <= din;  // Latch the input data into the output register.
        end
    end
endmodule
























