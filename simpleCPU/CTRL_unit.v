`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/21/2024 04:25:14 PM
// Design Name: 
// Module Name: CTRL_unit
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


// Module definition for a control unit with parameterizable width, N.
module CTRL_unit #(
    parameter N = 8  // Bit width of the instruction and data paths.
)(
    input clk,                       // Clock input.
    input rst,                       // Asynchronous reset input.
    input run,                       // Input signal to start the operation.
    input [N-1:0] instruction,       // Instruction input, determining the operation.
    output reg [9:0] select_mux,     // Control signal for multiplexer selection.
    output reg [10:0] select_reg,    // Control signal for register selection.
    output reg A,                    // Auxiliary control signal, usage depends on context.
    output reg done                  // Signal indicating completion of an operation.
);

    // State machine variables and declarations.
    reg [3:0] state, next_state;  // Current and next state definitions.
    localparam idle    = 2'd0,    // State definitions for readability.
               R_M     = 2'd1,
               add_sub = 2'd2,
               last    = 2'd3;
    
    // State transition logic: triggered on clock or reset.
    always @(posedge clk or negedge rst) begin
        if (!rst)
            state <= idle;       // Reset to idle state.
        else
            state <= next_state; // Transition to the next state.
    end
    
    // Next state logic and output logic based on current state and inputs.
    always @(*) begin
        case (state)
            idle: begin
                // In idle state, transition to R_M state if 'run' is high, else stay in idle.
                if (run == 1'b1)
                    next_state = R_M;
                else
                    next_state = idle;
            end
            R_M: begin
                // In R_M state, decide next state based on the instruction's upper bits.
                if (instruction[7:6] == 2'b10 || instruction[7:6] == 2'b11)
                    next_state = add_sub;
                else
                    next_state = idle;
            end
            add_sub: begin
                // In add_sub state, always transition to last state next.
                next_state = last;
            end
            last: begin
                // In last state, transition back to idle.
                next_state = idle;                
            end
            default: begin
                // Default case: go back to idle if an unknown state is encountered.
                next_state = idle;
            end
        endcase
    end
// Combinational logic for next state determination and control signal generation.
    always @(*) begin
        case (state)
            idle: begin
                // In idle state, set default control signals and check if 'run' is activated.
                select_mux = 10'b0000000001;  // Default selection for multiplexer.
                select_reg = 11'b00000000001; // Default selection for register or functional unit.
                A = 1'b0;                     // Default state for auxiliary control signal A.
                done = 1'b0;                  // Indicate that the operation is not done.
                  end
            R_M:
            begin
                if(instruction[7:6]==2'b00) //the first instruction mvi
                begin
                       A = 1'b0;
                       done = 1'b1;
                       case(instruction[5:3])
                        3'b000:         select_reg = 11'b00000000010;
                        3'b001: 	select_reg = 11'b00000000100;
                        3'b010: 	select_reg = 11'b00000001000;
                        3'b011: 	select_reg = 11'b00000010000;
                        3'b100: 	select_reg = 11'b00000100000;
                        3'b101: 	select_reg = 11'b00001000000;
                        3'b110: 	select_reg = 11'b00010000000;
                        3'b111: 	select_reg = 11'b00100000000;
                        default: select_reg = 11'b00000000000;
                        endcase
                        select_mux = 11'b0000000001;
                end
                else if(instruction[7:6]==2'b01)// the second instruction cpy
                begin
                    A = 1'b0;
                    done = 1'b1;
                   case(instruction[5:3])
                        3'b000:         select_reg = 11'b00000000010;
                        3'b001: 	select_reg = 11'b00000000100;
                        3'b010: 	select_reg = 11'b00000001000;
                        3'b011: 	select_reg = 11'b00000010000;
                        3'b100: 	select_reg = 11'b00000100000;
                        3'b101: 	select_reg = 11'b00001000000;
                        3'b110: 	select_reg = 11'b00010000000;
                        3'b111: 	select_reg = 11'b00100000000;
                        default: select_reg = 11'b00000000000;
                    endcase
                    case(instruction[2:0])
                        3'b000:        select_mux = 10'b0000000010;
                        3'b001: 	select_mux = 10'b0000000100;
                        3'b010: 	select_mux = 10'b0000001000;
                        3'b011: 	select_mux = 10'b0000010000;
                        3'b100: 	select_mux = 10'b0000100000;
                        3'b101: 	select_mux = 10'b0001000000;
                        3'b110: 	select_mux = 10'b0010000000;
                        3'b111: 	select_mux = 10'b0100000000;
                        default: select_mux = 10'b0000000000;
                    endcase
                 
                end
                
                else if(instruction[7:6]==2'b10) // the add instruction
                begin
                    A = 1'b0;
                    done = 1'b0;
                    select_reg = 11'b01000000000;
                       case(instruction[5:3])
                        3'b000:         select_mux = 10'b0000000010;
                        3'b001: 	select_mux = 10'b0000000100;
                        3'b010: 	select_mux = 10'b0000001000;
                        3'b011: 	select_mux = 10'b0000010000;
                        3'b100: 	select_mux = 10'b0000100000;
                        3'b101: 	select_mux = 10'b0001000000;
                        3'b110: 	select_mux = 10'b0010000000;
                        3'b111: 	select_mux = 10'b0100000000;
                        default: select_mux = 10'b0000000000;
                    endcase
                
                end
                
                else // the sub instruction
                begin
                     A = 1'b1;
                    done = 1'b0;
                    select_reg = 11'b01000000000;
                       case(instruction[5:3])
                        3'b000:         select_mux = 10'b0000000010;
                        3'b001: 	select_mux = 10'b0000000100;
                        3'b010: 	select_mux = 10'b0000001000;
                        3'b011: 	select_mux = 10'b0000010000;
                        3'b100: 	select_mux = 10'b0000100000;
                        3'b101: 	select_mux = 10'b0001000000;
                        3'b110: 	select_mux = 10'b0010000000;
                        3'b111: 	select_mux = 10'b0100000000;
                        default: select_mux = 10'b0000000000;
                    endcase
                
                end
            
            end
        
        add_sub: // this state corespond to the Save A +/- RY to G part
            begin
                if(instruction[7:6]==2'b10)
                begin
                        A = 1'b0;
                        done = 1'b0;
                        select_reg = 11'b10000000000;
                       case(instruction[2:0])
                        3'b000:         select_mux = 10'b0000000010;
                        3'b001: 	select_mux = 10'b0000000100;
                        3'b010: 	select_mux = 10'b0000001000;
                        3'b011: 	select_mux = 10'b0000010000;
                        3'b100: 	select_mux = 10'b0000100000;
                        3'b101: 	select_mux = 10'b0001000000;
                        3'b110: 	select_mux = 10'b0010000000;
                        3'b111: 	select_mux = 10'b0100000000;
                        default: select_mux = 10'b0000000000;
                    endcase
                end
                
                else
                begin
                         A = 1'b1;
                        done = 1'b0;
                       select_reg = 11'b10000000000;
                       case(instruction[2:0])
                        3'b000:     select_mux = 10'b0000000010;
                        3'b001: 	select_mux = 10'b0000000100;
                        3'b010: 	select_mux = 10'b0000001000;
                        3'b011: 	select_mux = 10'b0000010000;
                        3'b100: 	select_mux = 10'b0000100000;
                        3'b101: 	select_mux = 10'b0001000000;
                        3'b110: 	select_mux = 10'b0010000000;
                        3'b111: 	select_mux = 10'b0100000000;
                        default: select_mux = 10'b0000000000;
                    endcase
                
                end
                          
            end
        
        last: // this state corespond to the Save G to RX
            begin
                 if(instruction[7:6]==2'b10)
                    A = 1'b0;
                 else
                    A = 1'b1;                      
                 done = 1'b1;
                
                   case(instruction[5:3])
                        3'b000:         select_reg = 11'b00000000010;
                        3'b001: 	select_reg = 11'b00000000100;
                        3'b010: 	select_reg = 11'b00000001000;
                        3'b011: 	select_reg = 11'b00000010000;
                        3'b100: 	select_reg = 11'b00000100000;
                        3'b101: 	select_reg = 11'b00001000000;
                        3'b110: 	select_reg = 11'b00010000000;
                        3'b111: 	select_reg = 11'b00100000000;
                        default: select_reg = 11'b00000000000;
                   endcase
                  select_mux = 10'b1000000000;  //choose the G register         
            end       
        default:
               begin
                    select_mux = 10'b0000000000;  // Reset multiplexer selection.
                    select_reg = 11'b00000000000; // Reset register selection.
                    A = 1'b0;                     // Reset auxiliary control signal.
                    done = 1'b0;                  // Reset operation completion indicator.
                   
                end   
    endcase
 
   end  

endmodule






