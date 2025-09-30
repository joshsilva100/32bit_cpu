`timescale 1ns / 1ps
// Define a module for a button-controlled run interface, integrating with a system processor.
module btn_run #(
	parameter N = 8  // Define the data width for the system.
)(
	input						clk, 		// Clock input for synchronization.
	input 						rst, 		// Global reset signal.
	input 						reg_rst, 	// Register reset signal.
	input 						key, 		// Input signal from a physical button.
	input                       run,		// Control signal to start the processor.
	input		     [N-1:0]    DIN, 		// Data input port.
	output           [N-1:0]    reg0,		// Output from register 0.
	output           [N-1:0]    reg1,  	// Output from register 1.
	output           [N-1:0]    reg2,   	// Output from register 2.
	output		[N-1:0] Bus_data, 	// Data bus used for internal and external data communication.
	output 						Done		// Signal indicating completion of operation.
);
   
    // Internal signal declarations for debouncing logic.
    wire      key_pulse;        // Pulse signal generated after key debounce.
 
    reg [7:0] key_buffer;       // Buffer to store the state of the key for debouncing.
    reg key_reg;                // Register to hold the debounced key value.
    
    // Debounce logic for the key input.
    always @(posedge clk or negedge rst) begin
        if (!rst)
            key_reg <= 1'd0;  // Reset the key register.
        else begin
            if (key == 1'd1)
                key_reg <= 1'd1;  // Set the key register if the key input is high.
            else 
                key_reg <= 1'd0;  // Clear the key register if the key input is low.
        end       
    end

integer i;
    // Buffer to hold the debounced key value for detecting edges.
    always @(posedge clk or negedge rst) begin
        if (!rst)
            key_buffer <= 8'd0;  // Reset the key buffer.
        else begin
            for (i = 0; i < 8; i = i + 1) begin
                // Fill the buffer based on the debounced key value.
                if (key_reg == 1'd1)
                    key_buffer[i] <= 1'd1;  // Set buffer bits if key is pressed.
                else
                    key_buffer[i] <= 1'd0;  // Clear buffer bits if the key is not pressed.
            end            
        end  
    end
    
    // Generate a single cycle pulse when the key is pressed.
    assign key_pulse = key_buffer & 8'hff;  
    
    // Instantiate the system_processor module with the debounced key_pulse signal.
    system_processor #(.N(8)
    ) System(
        .clk(key_pulse),         // Use the debounced key pulse as the clock input.
        .rst(rst),
        .reg_rst(reg_rst),
        .Run(run),
        .DIN(DIN),
        .reg0(reg0),
        .reg1(reg1),
        .reg2(reg2),
        .Bus_data(Bus_data),
        .Done(Done)
    );     
endmodule


