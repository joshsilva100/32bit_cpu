`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/21/2024 04:26:12 PM
// Design Name: 
// Module Name: system_processor
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




// Define a system processor module with parameterizable width N.
// This module simulates a simple processing unit with various registers and operations.
module system_processor #(
	parameter N = 16  // Define the data width for the system.
)(
	input						clk, 		// Clock input for synchronization.
	input 						rst, 		// Global reset signal.
	input 						reg_rst, 	// Register reset signal.
	input 						Run, 		// Control signal to start the processor.
	input		 [N-1:0]    DIN, 		// Data input port.
	output           [N-1:0]    reg0,		// Output from register 0.
	output           [N-1:0]    reg1,  	// Output from register 1.
	output           [N-1:0]    reg2,   	// Output from register 2.
	output		 [N-1:0] Bus_data, 	// Data bus used for internal and external data communication.
	output 						Done		// Signal indicating completion of operation.
);

    // Internal wire declarations.
    wire [7:0] save_instruction;  // Wire to hold instruction from instruction register.
    wire [N-1:0] reg3;            // Additional internal registers.
    wire [N-1:0] reg4;
    wire [N-1:0] reg5;
    wire [N-1:0] reg6;
    wire [N-1:0] reg7;
    
    wire [N-1:0] a;               // Wire for temporary storage used in operations.
    wire [N-1:0] g;               // General purpose register or calculation result storage.
    wire [N-1:0] add_result;      // Result of addition or subtraction operation.
    
    wire add_ctrl;                // Control signal for addition or subtraction.
    
    wire [10:0] reg_ctrl;         // Control signals for register enable.
    wire [9:0] select_mux;        // Multiplexer selection signal.

    // Module instantiations
    // Instruction Register
    reg_enabled #(.N(8)
    ) IR (
        .clk(clk),
        .rst(reg_rst),
        .enable(reg_ctrl[0]),
        .din(DIN[7:0]),
        .dout(save_instruction)
    );
    
    // Data Registers instantiation for storing and manipulating data
    // Each Data_X module represents a register which can be used to store data from the Bus_data.
    reg_enabled #(.N(8)) Data_1 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[1]), .din(Bus_data), .dout(reg0));
    reg_enabled #(.N(8)) Data_2 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[2]), .din(Bus_data), .dout(reg1));
    // Additional data registers Data_3 to Data_8 follow similar pattern as Data_1 and Data_2
    reg_enabled #(.N(8)) Data_3 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[3]), .din(Bus_data), .dout(reg2));
    reg_enabled #(.N(8)) Data_4 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[4]), .din(Bus_data), .dout(reg3));
    reg_enabled #(.N(8)) Data_5 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[5]), .din(Bus_data), .dout(reg4));
    reg_enabled #(.N(8)) Data_6 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[6]), .din(Bus_data), .dout(reg5));
    reg_enabled #(.N(8)) Data_7 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[7]), .din(Bus_data), .dout(reg6));
    reg_enabled #(.N(8)) Data_8 (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[8]), .din(Bus_data), .dout(reg7));

    // Arithmetic Control Register - Controls input to the arithmetic unit.
    reg_enabled #(.N(8)) A_ctrl (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[9]), .din(Bus_data), .dout(a));
    
    // General register - Could be used for various purposes, here for storing arithmetic results.
    reg_enabled #(.N(8)) g_ctrl (.clk(clk), .rst(reg_rst), .enable(reg_ctrl[10]), .din(add_result), .dout(g));
    
    // Arithmetic Unit - Performs addition or subtraction based on add_ctrl signal.
    add_sub #(.N(N)) addition (.addsub(add_ctrl), .x0(a), .x1(Bus_data), .result(add_result));
    
    // Multiplexer - Selects one of several input data sources to output onto the Bus_data based on select_mux.
    Mux_hot #(.N(N)) Mux (.select(select_mux), .s0(reg0), .s1(reg1), .s2(reg2), .s3(reg3), .s4(reg4), .s5(reg5), .s6(reg6), .s7(reg7), .din(DIN), .g(g), .result(Bus_data));
 
    // Control Unit - Controls the processor's operation including instruction decoding and generating control signals.
    CTRL_unit #( .N(N)) Ctrl_test (.clk(clk), .rst(rst), .run(Run), .instruction(save_instruction), .select_mux(select_mux), .select_reg(reg_ctrl), .A(add_ctrl), .done(Done));
  
endmodule






