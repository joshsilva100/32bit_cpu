/* Pipeline Registers: Effectively create each Pipeline stage. 
                       Take in and out signals from single cycle design. Separating previous components into the stages by making them 
                       take their inputs from the pipeline registers instead of directly from previous modules.

    Control Signals (Take from MIPS decoder):
        EX: ALUSrc (i_type), ALUOp, RegDst (Sel signal for rdNum, for now include both i_type and load_ra)
        MEM: MemRead (?, read memory everytime. Or just data from data memory?), MemWrite (word_we and byte_we)
        WB: RegWrite(wr_enable), MemToReg (mem_read)
*/ 

// IF/ID Register: Handle transition from IF (Fetch) to ID (Decode) stages. 
//
// INPUTS:
//
// clk     (input) - the clock signal
// flush   (input) - set to 1 to set all registers to zero, set to 0 for normal execution. Decide if we keep this or not
// IF/ID Write (input) - From the Hazard Unit. Determines if register is updated
// inst (input) - MIPS Instruction from the Instruction Memory.  
// link_data (input) - Link ALU output. Take from PC
// 
// OUTPUTS: 
//
// out_inst (output) - MIPS Instruction from the Instruction Memory. Hold onto until ID stage or stall ends 
// out_link_data (output) - Link ALU output. Take from PC and store for next stage

module if_id_reg(clk, flush, 
             if_id_write, inst, link_data,
             out_inst, out_link_data); 

    input clk, flush, if_id_write; 
    input [31:0] inst, link_data;

    output reg [31:0] out_inst, out_link_data; 

    always @(posedge clk or posedge flush) begin
		// Clear register
        if (flush)
			out_inst <= 32'h00000000; 
            out_link_data <= 32'h00000000;  

        // Normal Operation
		else if (if_id_write)
            out_inst <= inst; 
            out_link_data <= link_data; 

        // Data Hazard! Stall until hazard unit says otherwise
        else 
            out_inst <= out_inst; 
            out_link_data <= out_link_data;
	end

endmodule 


// ID/EX Register: Handle transition from ID (Decode) to EX (Execute) stages. 
//
// INPUTS: 
//
// clk     (input) - the clock signal
// flush   (input) - set to 1 to set all registers to zero, set to 0 for normal execution. Decide if we keep this or not  
//
// WB Control Group
// write (input) - Write Enable signal for the Register file, determines whether data is written. From the Mips Decoder 
// mem_read (input) - "MemToReg" sel signal. Determines whether data is read from EX stage or from Data Memory. From the MIPS Decoder. 
//
// M Control Group
// word_we (input) - Half of "MemWrite." Word Enable Control Signal for Data Memory from the MIPS Decoder
// byte_we (input) - Other Half of "MemWrite." Byte Enable Control Signal for Data Memory from the MIPS Decoder 
// byte_load (input) - Decides if loading a byte or whole word.
//
// EX Control Group
// i_type (input) - Acts as ALUSrc and part of RegDst. Determines whether data for Primary ALU Source 2 is from rtData or from Sign Extender. From the MIPS Decoder 
// load_ra (input) - Acts as part of RegDst. Determines if data is from $RA or from Rd/Rt
// alu_op (input) - Determines operation of primary ALU 
// slt (input) - SLT control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage.
// shift (input) - SHIFT control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage.
// lui (input) - LUI control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage.
// link (input) - LINK control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage.
// 
// Other Signals: ID Stage
// rsData (input) - Data from register RS from the Register File
// rtData (input) - Data from register RT from the Register File 
// imm32 (input) - Sign Extender output from inst[15:0] 
// rt (input) - Rt Register from instruction. Will help decide rdNum in EX stage
// rd (input) - Rd Register from instruction. Will help decide rdNum in EX stage
// rs (input) - Rs Register from instruction.
// mux_shift_v_A (input) - First input into the mux_shift_v mux NOTE: Make this the input .A({ 27'b0, inst[10:6] }). 
// lui_data (input) - The upper immediate data. Will be acted upon in EX stage
// link_data (input) - Link ALU output. Take from PC, and will be acted upon in EX stage
// 
// OUTPUTS: 
//
// WB Control Group
// out_write (output) - Write Enable signal for the Register file, determines whether data is written. Sending to EX stage
// out_mem_read (output) - "MemToReg" sel signal. Determines whether data is read from EX stage or from Data Memory. Sending to EX stage 
// 
// M Control Group
// out_word_we (output) - Half of "MemWrite." Word Enable Control Signal for Data Memory. Sending/Holding for M stage
// out_byte_we (output) - Other Half of "MemWrite." Byte Enable Control Signal for Data Memory. Sending/Holding for M stage 
// out_byte_load (input) - Decides if loading a byte or whole word. Sending/Holding for M stage
// 
// EX Control Group
// out_i_type (output) - Acts as ALUSrc. Determines whether data for Primary ALU Source 2 is from rtData or from Sign Extender. Sending to EX stage 
// out_load_ra (input) - Acts as part of RegDst. Determines if data is from $RA or from Rd/Rt. Send to EX stage
// out_alu_op (output) - Determines operation of primary ALU. Send to EX stage 
// out_slt (input) - SLT control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage. Send to EX stage 
// out_shift (input) - SHIFT control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage. Send to EX stage 
// out_lui (input) - LUI control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage. Send to EX stage 
// out_link (input) - LINK control signal from MIPS decoder. Will decide between ALU_out or other control signals in EX stage. Send to EX stage 
// 
// Other Signals: ID Stage to EX Stage
// out_rsData (output) - Data from register RS from the Register File. Sending to EX stage
// out_rtData (output) - Data from register RT from the Register File. Sending to EX stage
// out_imm32 (output) - Sign Extender output from inst[15:0]. Sending to EX stage
// out_rt (output) - Rt Register from instruction. Will help decide rdNum in EX stage. Sending to EX stage
// out_rd (output) - Rd Register from instruction. Will help decide rdNum in EX stage. Sending to EX stage
// out_rs (output) - Rs Register from instruction. Sending to EX stage
// out_mux_shift_v_A (output) - First input into the mux_shift_v mux. Sending to EX stage (NOTE: Make this the input .A({ 27'b0, inst[10:6] })) 
// out_lui_data (input) - The upper immediate data. Sending to EX stage
// out_link_data (input) - Link ALU output. Take from PC. Sending to EX stage

module id_ex_reg(clk, flush, 
             write, mem_read, //WB
             word_we, byte_we, byte_load, //M
             i_type, load_ra, alu_op, slt, shift, lui, link, //EX
             rsData, rtData, imm32, rt, rd, rs, mux_shift_v_A, lui_data, link_data, //Other 
             out_write, out_mem_read, //WB
             out_word_we, out_byte_we, out_byte_load, //M
             out_i_type, out_load_ra, out_alu_op, out_slt, out_shift, out_lui, out_link, //EX
             out_rsData, out_rtData, out_imm32, out_rt, out_rd, out_rs, out_mux_shift_v_A, out_lui_data, out_link_data //Other
            ); 

    input clk, flush, write, mem_read, word_we, byte_we, byte_load, i_type, load_ra, slt, shift, lui, link; 
    input [2:0] alu_op;
    input [31:0] rsData, rtData, imm32, mux_shift_v_A, lui_data, link_data; 
    input [4:0] rt, rd, rs;

    output reg out_write, out_mem_read, out_word_we, out_byte_we, out_byte_load, out_i_type, out_load_ra, out_slt, out_shift, out_lui, out_link; 
    output reg [2:0] out_alu_op;
    output reg [31:0] out_rsData, out_rtData, out_imm32, out_mux_shift_v_A, out_lui_data, out_link_data; 
    output reg [4:0] out_rt, out_rd, out_rs;

    always @(posedge clk or posedge flush) begin
		// Clear register
        if (flush) 
            out_write <= 0; 
            out_mem_read <= 0; 
            out_word_we <= 0; 
            out_byte_we <= 0; 
            out_byte_load <= 0;
            out_i_type <= 0; 
            out_load_ra <= 0; 
            out_slt <= 0; 
            out_shift <= 0; 
            out_lui <= 0; 
            out_link <= 0; 

            out_alu_op <= 3'h0;

            out_rsData <= 32'h00000000; 
            out_rtData <= 32'h00000000; 
            out_imm32 <= 32'h00000000; 
            out_mux_shift_v_A <= 32'h00000000; 
            out_lui_data <= 32'h00000000; 
            out_link_data <= 32'h00000000; 

            out_rt <= 5'h00;
            out_rd <= 5'h00; 
            out_rs <= 5'h00;

        // Normal Operation
        else 
            out_write <= write; 
            out_mem_read <= mem_read; 
            out_word_we <= word_we; 
            out_byte_we <= byte_we; 
            out_byte_load <= byte_load;
            out_i_type <= i_type; 
            out_load_ra <= load_ra; 
            out_slt <= slt; 
            out_shift <= shift; 
            out_lui <= lui; 
            out_link <= link; 

            out_alu_op <= alu_op;

            out_rsData <= rsData; 
            out_rtData <= rtData; 
            out_imm32 <= imm32; 
            out_mux_shift_v_A <= mux_shift_v_A; 
            out_lui_data <= lui_data; 
            out_link_data <= link_data; 

            out_rt <= rt;
            out_rd <= rd; 
            out_rs <= rs;
	end

endmodule 

// EX/MEM Register: Handle transition from EX (Execute) to MEM (Memory) stages. 
//
// clk     (input) - the clock signal
// flush   (input) - set to 1 to set all registers to zero, set to 0 for normal execution. Decide if we keep this or not 
// 
// INPUTS: 
//
// WB Control Group 
// write (input) - Write Enable for the Register File to write. WB signal from ID/EX
// mem_read (input) - Mux Select to determine if output is from EX stage or MEM stage. WB signal from ID/EX 
// 
// M Control Group 
// word_we (input) - Half of "MemWrite." Word Enable Control Signal for Data Memory from the MIPS Decoder
// byte_we (input) - Other Half of "MemWrite." Byte Enable Control Signal for Data Memory from the MIPS Decoder 
// byte_load (input) - Decides if loading a byte or whole word
// 
// Other Signals
// rdNum (input) - Register Number for Register File to write to. 
// aluOut (input) - Result output from the EX stage.  
// rtData (input) - Data from the RT register that could be written to Data Memory. 
// 
// OUTPUTS: 
//
// WB Control Group
// out_write (output) - Write Enable for the Register File to write. Sending/Holding for WB stage
// out_mem_read (output) - Mux Select to determine if output is from EX stage or MEM stage. Sending/Holding for WB stage 
// 
// M Control Group
// out_word_we (output) - Half of "MemWrite." Word Enable Control Signal for Data Memory. Sending/Holding for M stage
// out_byte_we (output) - Other Half of "MemWrite." Byte Enable Control Signal for Data Memory. Sending/Holding for M stage 
// out_byte_load (input) - Decides if loading a byte or whole word. Sending/Holding for M stage
// 
// Other Signals
// out_rdNum (output) - Register Number for Register File to write to. Sending/Holding for WB stage
// out_aluOut (output) - Result output from the EX stage. Sending/Holding for WB stage 
// out_rtData (output) - Data from the RT register that could be written to Data Memory. Sending/Holding for MEM stage

module ex_mem_reg(clk, flush, 
              write, mem_read, //WB
              word_we, byte_we, byte_load, //M
              rdNum, aluOut, rtData, //Other 
              out_write, out_mem_read, //WB
              out_word_we, out_byte_we, out_byte_load, //M
              out_rdNum, out_aluOut, out_rtData //Other
            ); 

    input clk, flush, write, mem_read, word_we, byte_we, byte_load; 
    input [4:0] rdNum; 
    input [31:0] aluOut, rtData;

    output reg out_write, out_mem_read, out_word_we, out_byte_we, out_byte_load; 
    output reg [4:0] out_rdNum; 
    output reg [31:0] out_aluOut, out_rtData;

    always @(posedge clk or posedge flush) begin
		// Clear register
        if (flush) 
            out_write <= 0; 
            out_mem_read <= 0;
            out_word_we <= 0;
            out_byte_we <= 0; 
            out_byte_load <= 0;
            
            out_rdNum <= 5'h00; 

            out_aluOut <= 32'h00000000; 
            out_rtData <= 32'h00000000;

        // Normal Operation 
        else 
            out_write <= write; 
            out_mem_read <= mem_read;
            out_word_we <= word_we;
            out_byte_we <= byte_we; 
            out_byte_load <= byte_load;
            
            out_rdNum <= rdNum; 

            out_aluOut <= aluOut; 
            out_rtData <= rtData;
	end

endmodule

// MEM/WB Register: Handle transition from MEM (Memory) to WB (Write) stages. 
//
// INPUTS: 
//
// clk     (input) - the clock signal
// flush   (input) - set to 1 to set all registers to zero, set to 0 for normal execution. Decide if we keep this or not 
//
// WB Control Group
// write (input) - Write Enable for the Register File to write. WB signal from EX/MEM
// mem_read (input) - Mux Select to determine if output is from EX stage or MEM stage. WB signal from EX/MEM 
// 
// Other Signals
// mem_data (input) - Output from Memory Unit. MEM Stage
// rdNum (input) - Register Number for Register File to write to. From EX/MEM
// aluOut (input) - Result output from the EX stage. From EX/MEM 
// 
// OUTPUTS: 
//
// WB Control Group
// out_write (output) - Write Enable for the Register File to write. Sending/Holding for WB stage
// out_mem_read (output) - Mux Select to determine if output is from EX stage or MEM stage. Sending/Holding for WB stage 
//
// Other Signals
// out_mem_data (output) - Output from Memory Unit. Sending/Holding for WB stage
// out_rdNum (output) - Register Number for Register File to write to. Sending/Holding for WB stage
// out_aluOut (output) - Result output from the EX stage. Sending/Holding for WB stage

module mem_wb_reg(clk, flush, 
              write, mem_read, //WB
              mem_data, rdNum, aluOut, //Other
              out_write, out_mem_read, //WB
              out_mem_data, out_rdNum, out_aluOut //Other
); 

    input clk, flush, write, mem_read;
    input [4:0] rdNum;
    input [31:0] mem_data, aluOut; 

    output reg out_write, out_mem_read;
    output reg [4:0] out_rdNum; 
    output reg [31:0] out_mem_data, out_aluOut; 

    always @(posedge clk or posedge flush) begin
		// Clear register
        if (flush) 
            out_write <= 0; 
            out_mem_read <= 0;

            out_rdNum <= 5'h00; 

            out_mem_data <= 32'h00000000; 
            out_aluOut <= 32'h00000000; 
        
        // Normal Operation
        else 
            out_write <= write; 
            out_mem_read <= mem_read;

            out_rdNum <= rdNum; 

            out_mem_data <= mem_data; 
            out_aluOut <= alu_out;
	end

endmodule 