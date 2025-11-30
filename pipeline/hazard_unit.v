// hazard_unit: Implement hazard detection to CPU
// id_ex.memRead (input) - MemRead from ID_EX stage
// id_ex.rt (input) - RT from ID_EX stage
// rs (input) - RS register
// rt (input) - RT register
// control_sel (output) - MUX sel to determine if control signals from MIPS decoder come through to ID/EX
// if_id_write (output) - Write for IF_ID stage
// pc_write (output) - Write for Program Counter

module hazard_unit(rs, rt, id_ex_memRead, id_ex_rt, control_sel, if_id_write, pc_write); 

    input [4:0] rs, rt, id_ex_rt;
    input id_ex_memRead;
    output control_sel, if_id_write, pc_write;

endmodule
  