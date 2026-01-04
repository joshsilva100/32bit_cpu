// forward_unit: Implement forwarding to pipeline to avoid data hazards
// rt_num_EX (input) - RT from ID/EX Register
// rs_num_EX (input) - RS from ID/EX Register
// wr_enable_MEM (input) - Write Enable from the EX/MEM Register
// wr_enable_WB (input) - Write Enable from the MEM/WB Register 
// rd_num_MEM (input) - RD from EX_MEM Register
// rd_num_WB (input) - RD from MEM_WB Register
// forwardA (output) - Sel for ALU input 1 src (Read data 1)
// forwardB (output) - Sel for ALU input 2 src (Read data 2)

module forward_unit(rt_num_EX, rs_num_EX, wr_enable_MEM, wr_enable_WB, rd_num_MEM, rd_num_WB, 
                    forwardA, forwardB); 

    input [4:0] rt_num_EX, rs_num_EX, rd_num_MEM, rd_num_WB;
    input wr_enable_MEM, wr_enable_WB;
    output reg [1:0] forwardA, forwardB; 

    // EX/MEM data hazards 
    // An EX/MEM hazard occurs between the instruction currently in its EX
    // stage and the previous instruction if:
    // 1. The previous instruction will write to the register file, and
    // 2. The destination is one of the ALU source registers in the EX stage. 

    // if (EX/MEM.RegWrite = 1
    //     and EX/MEM.RegisterRd = ID/EX.RegisterRs)
    // then ForwardA = 2 

    // The second ALU source is similar.
    // if (EX/MEM.RegWrite = 1
    //     and EX/MEM.RegisterRd = ID/EX.RegisterRt)
    // then ForwardB = 2 

    // MEM/WB data hazards 
    // if (MEM/WB.RegWrite = 1
    //  and MEM/WB.RegisterRd = ID/EX.RegisterRs 
    //  and (EX/MEM.RegisterRd ≠ ID/EX.RegisterRs or EX/MEM.RegWrite = 0)
    // then ForwardA = 1

    //The second ALU operand is handled similarly.
    // if (MEM/WB.RegWrite = 1
    //  and MEM/WB.RegisterRd = ID/EX.RegisterRt 
    //  and (EX/MEM.RegisterRd ≠ ID/EX.RegisterRt or EX/MEM.RegWrite = 0)
    // then ForwardB = 1

    always @(*) begin  
        // Default values. No hazard unless detected in following conditions 
        forwardA = 2'h0; 
        forwardB = 2'h0; 

        // EX/MEM data hazards
        if (wr_enable_MEM) begin // If instruction in MEM stage wants to write to register file
            if (rd_num_MEM == rs_num_EX) begin // and the register it's writing is the same as rs
                forwardA = 2'h2; // load from the ALU out in MEM stage rather than rs
            end 

            if (rd_num_MEM == rt_num_EX) begin  // and the register it's writing is the same as rt
                forwardB = 2'h2; // load from the ALU out in MEM stage rather than rt
            end
        end 

        // MEM/WB data hazards
        if (wr_enable_WB) begin  // if the instruction in WB wants to write to register file
            // and the register it's writing is the same as rs
            // and MEM stage is not writing or MEM stage is writing to a different register
            // (to avoid overwrites)
            if ((rd_num_WB == rs_num_EX) && (rd_num_MEM != rs_num_EX || wr_enable_MEM == 0)) begin
                forwardA = 2'h1; // use rd_data instead of rs
            end 

            // and the register it's writing is the same as rd
            // and MEM stage is not writing or MEM stage is writing to a different register
            // (to avoid overwrites)
            if ((rd_num_WB == rt_num_EX) && (rd_num_MEM != rt_num_EX || wr_enable_MEM == 0)) begin 
                forwardB = 2'h1; // use rd_data instead of rt
            end
        end 
    end

endmodule