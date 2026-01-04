// branch_mod: Decides, in the EX stage, whether to take a branch or not. 
// 
// zero (input) - Output from the primary ALU. When its inputs result in zero, then true
// branch (input) - From the ID/EX pipeline register. Originally from the MIPS decoder, true when a branch instruction is given
// branch_taken (output) - Based on inputs, sends out a signal that the branch must be taken

module branch_mod(zero, negative, branch_control, branch_taken); 

    input zero, negative;
    input [2:0] branch_control;
    output reg branch_taken = 0;

    always @(*) begin
        case (branch_control)
            `BRANCH_NOP: begin
                branch_taken = 0;
            end

            `BRANCH_EQ: begin
                branch_taken = zero;
            end

            `BRANCH_NEQ: begin
                branch_taken = ~zero;
            end

            `BRANCH_LTZ: begin
                branch_taken = negative || zero;
            end

            `BRANCH_GTZ: begin
                branch_taken = ~(negative || zero);
            end
        endcase
    end

endmodule