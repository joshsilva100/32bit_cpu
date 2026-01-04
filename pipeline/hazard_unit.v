// hazard_unit: Implement hazard detection to CPU. Stall if hazard detected
// id_ex.memRead (input) - MemRead from ID_EX stage
// id_ex.rt (input) - RT from ID_EX stage
// rs (input) - RS from ID stage
// rt (input) - RT from ID stage
// control_sel (output) - MUX sel to determine if control signals from MIPS decoder come through to ID/EX
// if_id_write (output) - Write for IF_ID stage
// pc_write (output) - Enable for Program Counter

module hazard_unit(mem_read_EX, rd_num_EX, rs_num_ID, rt_num_ID, branch_taken, jr, stall, flush); 

    input mem_read_EX, branch_taken, jr;
    input [4:0] rd_num_EX;
    input [4:0] rs_num_ID;
    input [4:0] rt_num_ID;
    output reg stall, flush;

    // We can detect a load hazard between the current instruction in its ID
    // stage and the previous instruction in the EX stage just like we detected
    // data hazards.
    // ▪ A hazard occurs if the previous instruction was LW...
    // ID/EX.MemRead = 1
    // ...and the LW destination is one of the current source registers.
    // ID/EX.RegisterRt = IF/ID.RegisterRs
    // or
    // ID/EX.RegisterRt = IF/ID.RegisterRt
    // ▪ The complete test for stalling is the conjunction of these two conditions.
    // if (ID/EX.MemRead = 1 and
    // ( ID/EX.RegisterRt = IF/ID.RegisterRs or
    // ID/EX.RegisterRt = IF/ID.RegisterRt))
    // then stall 

    always @(*) begin 

        // By default, do not stall or "flush"
        stall = 0; 
        flush = 0;

        // Check for hazard, stall if conditions met
        // Load Hazard
        if (mem_read_EX == 1) begin // If the EX stage plans to read from memory
            // and the instruction in EX wants to write to either register we are loading in ID stage
            if ((rd_num_EX == rs_num_ID) || (rd_num_EX == rt_num_ID)) begin 
                stall = 1;
            end
        end 

        // Branch or JR Hazard. Both require flushing IF/ID and ID/EX but no stalling required
        if (branch_taken || jr) begin 
            flush = 1;
        end
    end

endmodule
  