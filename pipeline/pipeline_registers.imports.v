// Import for pipeline register 'if_id'.
// Wire assumptions:
// Any wire that crosses the register will use format name_STAGE i.e. data_out_MEM
// (For best results, generate wires with generate_pipelines.py)
// Clock signal is clk
// Reset signal is reset
// Enable wire is if_id_reg_enable
if_id_reg IF_ID(
    .clk(clk), .reset(reset), .enable(if_id_reg_enable), .flush(if_id_reg_flush),
    .inst_IN(inst_IF), .inst_OUT(inst_ID),
    .link_alu_out_IN(link_alu_out_IF), .link_alu_out_OUT(link_alu_out_ID),
    .plus4_alu_out_IN(plus4_alu_out_IF), .plus4_alu_out_OUT(plus4_alu_out_ID),
    .alu_op_IN(alu_op_IF), .alu_op_OUT(alu_op_ID),
    .wr_enable_IN(wr_enable_IF), .wr_enable_OUT(wr_enable_ID),
    .itype_IN(itype_IF), .itype_OUT(itype_ID),
    .except_IN(except_IF), .except_OUT(except_ID),
    .control_type_IN(control_type_IF), .control_type_OUT(control_type_ID),
    .lui_IN(lui_IF), .lui_OUT(lui_ID),
    .slt_IN(slt_IF), .slt_OUT(slt_ID),
    .byte_load_IN(byte_load_IF), .byte_load_OUT(byte_load_ID),
    .word_we_IN(word_we_IF), .word_we_OUT(word_we_ID),
    .byte_we_IN(byte_we_IF), .byte_we_OUT(byte_we_ID),
    .mem_read_IN(mem_read_IF), .mem_read_OUT(mem_read_ID),
    .shift_v_IN(shift_v_IF), .shift_v_OUT(shift_v_ID),
    .shift_op_IN(shift_op_IF), .shift_op_OUT(shift_op_ID),
    .shift_IN(shift_IF), .shift_OUT(shift_ID),
    .load_ra_IN(load_ra_IF), .load_ra_OUT(load_ra_ID),
    .link_IN(link_IF), .link_OUT(link_ID),
    .alu_b_zero_IN(alu_b_zero_IF), .alu_b_zero_OUT(alu_b_zero_ID),
    .branch_control_IN(branch_control_IF), .branch_control_OUT(branch_control_ID),
    .jr_IN(jr_IF), .jr_OUT(jr_ID)
);

// Import for pipeline register 'id_ex'.
// Wire assumptions:
// Any wire that crosses the register will use format name_STAGE i.e. data_out_MEM
// (For best results, generate wires with generate_pipelines.py)
// Clock signal is clk
// Reset signal is reset
// Enable wire is id_ex_reg_enable
id_ex_reg ID_EX(
    .clk(clk), .reset(reset), .enable(id_ex_reg_enable), .flush(id_ex_reg_flush),
    .inst_IN(inst_ID), .inst_OUT(inst_EX),
    .rs_data_IN(rs_data_ID), .rs_data_OUT(rs_data_EX),
    .rt_data_IN(rt_data_ID), .rt_data_OUT(rt_data_EX),
    .link_alu_out_IN(link_alu_out_ID), .link_alu_out_OUT(link_alu_out_EX),
    .rs_num_IN(rs_num_ID), .rs_num_OUT(rs_num_EX),
    .rt_num_IN(rt_num_ID), .rt_num_OUT(rt_num_EX),
    .plus4_alu_out_IN(plus4_alu_out_ID), .plus4_alu_out_OUT(plus4_alu_out_EX),
    .alu_op_IN(alu_op_ID), .alu_op_OUT(alu_op_EX),
    .wr_enable_IN(wr_enable_ID), .wr_enable_OUT(wr_enable_EX),
    .itype_IN(itype_ID), .itype_OUT(itype_EX),
    .except_IN(except_ID), .except_OUT(except_EX),
    .control_type_IN(control_type_ID), .control_type_OUT(control_type_EX),
    .lui_IN(lui_ID), .lui_OUT(lui_EX),
    .slt_IN(slt_ID), .slt_OUT(slt_EX),
    .byte_load_IN(byte_load_ID), .byte_load_OUT(byte_load_EX),
    .word_we_IN(word_we_ID), .word_we_OUT(word_we_EX),
    .byte_we_IN(byte_we_ID), .byte_we_OUT(byte_we_EX),
    .mem_read_IN(mem_read_ID), .mem_read_OUT(mem_read_EX),
    .shift_v_IN(shift_v_ID), .shift_v_OUT(shift_v_EX),
    .shift_op_IN(shift_op_ID), .shift_op_OUT(shift_op_EX),
    .shift_IN(shift_ID), .shift_OUT(shift_EX),
    .load_ra_IN(load_ra_ID), .load_ra_OUT(load_ra_EX),
    .link_IN(link_ID), .link_OUT(link_EX),
    .alu_b_zero_IN(alu_b_zero_ID), .alu_b_zero_OUT(alu_b_zero_EX),
    .branch_control_IN(branch_control_ID), .branch_control_OUT(branch_control_EX),
    .jr_IN(jr_ID), .jr_OUT(jr_EX)
);

// Import for pipeline register 'ex_mem'.
// Wire assumptions:
// Any wire that crosses the register will use format name_STAGE i.e. data_out_MEM
// (For best results, generate wires with generate_pipelines.py)
// Clock signal is clk
// Reset signal is reset
// Enable wire is ex_mem_reg_enable
ex_mem_reg EX_MEM(
    .clk(clk), .reset(reset), .enable(ex_mem_reg_enable), .flush(ex_mem_reg_flush),
    .alu_out_IN(alu_out_EX), .alu_out_OUT(alu_out_MEM),
    .rt_data_IN(rt_data_EX), .rt_data_OUT(rt_data_MEM),
    .rd_num_IN(rd_num_EX), .rd_num_OUT(rd_num_MEM),
    .link_out_IN(link_out_EX), .link_out_OUT(link_out_MEM),
    .rs_num_IN(rs_num_EX), .rs_num_OUT(rs_num_MEM),
    .rt_num_IN(rt_num_EX), .rt_num_OUT(rt_num_MEM),
    .alu_op_IN(alu_op_EX), .alu_op_OUT(alu_op_MEM),
    .wr_enable_IN(wr_enable_EX), .wr_enable_OUT(wr_enable_MEM),
    .itype_IN(itype_EX), .itype_OUT(itype_MEM),
    .except_IN(except_EX), .except_OUT(except_MEM),
    .control_type_IN(control_type_EX), .control_type_OUT(control_type_MEM),
    .lui_IN(lui_EX), .lui_OUT(lui_MEM),
    .slt_IN(slt_EX), .slt_OUT(slt_MEM),
    .byte_load_IN(byte_load_EX), .byte_load_OUT(byte_load_MEM),
    .word_we_IN(word_we_EX), .word_we_OUT(word_we_MEM),
    .byte_we_IN(byte_we_EX), .byte_we_OUT(byte_we_MEM),
    .mem_read_IN(mem_read_EX), .mem_read_OUT(mem_read_MEM),
    .shift_v_IN(shift_v_EX), .shift_v_OUT(shift_v_MEM),
    .shift_op_IN(shift_op_EX), .shift_op_OUT(shift_op_MEM),
    .shift_IN(shift_EX), .shift_OUT(shift_MEM),
    .load_ra_IN(load_ra_EX), .load_ra_OUT(load_ra_MEM),
    .link_IN(link_EX), .link_OUT(link_MEM),
    .alu_b_zero_IN(alu_b_zero_EX), .alu_b_zero_OUT(alu_b_zero_MEM),
    .branch_control_IN(branch_control_EX), .branch_control_OUT(branch_control_MEM),
    .jr_IN(jr_EX), .jr_OUT(jr_MEM)
);

// Import for pipeline register 'mem_wb'.
// Wire assumptions:
// Any wire that crosses the register will use format name_STAGE i.e. data_out_MEM
// (For best results, generate wires with generate_pipelines.py)
// Clock signal is clk
// Reset signal is reset
// Enable wire is mem_wb_reg_enable
mem_wb_reg MEM_WB(
    .clk(clk), .reset(reset), .enable(mem_wb_reg_enable), .flush(mem_wb_reg_flush),
    .alu_out_IN(alu_out_MEM), .alu_out_OUT(alu_out_WB),
    .data_out_IN(data_out_MEM), .data_out_OUT(data_out_WB),
    .rd_num_IN(rd_num_MEM), .rd_num_OUT(rd_num_WB),
    .link_out_IN(link_out_MEM), .link_out_OUT(link_out_WB),
    .alu_op_IN(alu_op_MEM), .alu_op_OUT(alu_op_WB),
    .wr_enable_IN(wr_enable_MEM), .wr_enable_OUT(wr_enable_WB),
    .itype_IN(itype_MEM), .itype_OUT(itype_WB),
    .except_IN(except_MEM), .except_OUT(except_WB),
    .control_type_IN(control_type_MEM), .control_type_OUT(control_type_WB),
    .lui_IN(lui_MEM), .lui_OUT(lui_WB),
    .slt_IN(slt_MEM), .slt_OUT(slt_WB),
    .byte_load_IN(byte_load_MEM), .byte_load_OUT(byte_load_WB),
    .word_we_IN(word_we_MEM), .word_we_OUT(word_we_WB),
    .byte_we_IN(byte_we_MEM), .byte_we_OUT(byte_we_WB),
    .mem_read_IN(mem_read_MEM), .mem_read_OUT(mem_read_WB),
    .shift_v_IN(shift_v_MEM), .shift_v_OUT(shift_v_WB),
    .shift_op_IN(shift_op_MEM), .shift_op_OUT(shift_op_WB),
    .shift_IN(shift_MEM), .shift_OUT(shift_WB),
    .load_ra_IN(load_ra_MEM), .load_ra_OUT(load_ra_WB),
    .link_IN(link_MEM), .link_OUT(link_WB),
    .alu_b_zero_IN(alu_b_zero_MEM), .alu_b_zero_OUT(alu_b_zero_WB),
    .branch_control_IN(branch_control_MEM), .branch_control_OUT(branch_control_WB),
    .jr_IN(jr_MEM), .jr_OUT(jr_WB)
);

