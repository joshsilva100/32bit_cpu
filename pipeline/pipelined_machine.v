// pipelined_machine: execute a series of MIPS instructions from an instruction cache
//
// except (output) - set to 1 when an unrecognized instruction is to be executed.
// clk     (input) - the clock signal
// reset   (input) - set to 1 to set all registers to zero, set to 0 for normal execution.

module pipelined_machine(except, clk, reset);
   output except;
   input clk, reset;

   // IF STAGE WIRES
   wire [31:0] PC;
   wire [31:0] nextPC;
   wire [31:0] branch_offset;
   wire [31:0] branch_alu_out;
   wire [31:0] branch_taken_mux_out;
   wire stall; // If 1, will stall the pipeline
   wire pc_write = ~stall; // Only write to PC if we are not stalling
   wire control_sel;

   // EX STAGE WIRES
   wire [31:0] shift_out;
   wire [31:0] shift_v_mux_out;
   wire [31:0] ALU_A;
   wire [31:0] ALU_B;
   wire [31:0] forwardB_out;
   wire [31:0] imm32;
   wire [31:0] itype_out_B;
   wire [4:0] itype_out;
   wire [31:0] shift_mux_out;
   wire [31:0] lui_out;
   wire [31:0] control_type_mux_out;
   wire [31:0] slt_out;
   wire [1:0] forwardA;
   wire [1:0] forwardB;
   wire branch_taken;
   wire zero;
   wire negative;

   // WB STAGE WIRES
   wire [7:0] data_size_out8;
   wire [31:0] data_size_out;
   wire [31:0] byte_load_out;
   wire [31:0] rd_data;

   // INTERSTAGE WIRES
   wire [31:0] inst_IF;
   wire [31:0] link_alu_out_IF;
   wire [31:0] plus4_alu_out_IF;
   wire [2:0] alu_op_IF;
   wire wr_enable_IF;
   wire itype_IF;
   wire except_IF;
   wire control_type_IF;
   wire lui_IF;
   wire slt_IF;
   wire byte_load_IF;
   wire word_we_IF;
   wire byte_we_IF;
   wire mem_read_IF;
   wire shift_v_IF;
   wire [1:0] shift_op_IF;
   wire shift_IF;
   wire load_ra_IF;
   wire link_IF;
   wire alu_b_zero_IF;
   wire [2:0] branch_control_IF;
   wire jr_IF;
   wire [31:0] inst_ID;
   wire [31:0] link_alu_out_ID;
   wire [31:0] plus4_alu_out_ID;
   wire [2:0] alu_op_ID;
   wire wr_enable_ID;
   wire itype_ID;
   wire except_ID;
   wire control_type_ID;
   wire lui_ID;
   wire slt_ID;
   wire byte_load_ID;
   wire word_we_ID;
   wire byte_we_ID;
   wire mem_read_ID;
   wire shift_v_ID;
   wire [1:0] shift_op_ID;
   wire shift_ID;
   wire load_ra_ID;
   wire link_ID;
   wire alu_b_zero_ID;
   wire [2:0] branch_control_ID;
   wire jr_ID;
   wire [31:0] rs_data_ID;
   wire [31:0] rt_data_ID;
   wire [4:0] rs_num_ID;
   wire [4:0] rt_num_ID;
   wire [31:0] inst_EX;
   wire [31:0] rs_data_EX;
   wire [31:0] rt_data_EX;
   wire [31:0] link_alu_out_EX;
   wire [4:0] rs_num_EX;
   wire [4:0] rt_num_EX;
   wire [31:0] plus4_alu_out_EX;
   wire [2:0] alu_op_EX;
   wire wr_enable_EX;
   wire itype_EX;
   wire except_EX;
   wire control_type_EX;
   wire lui_EX;
   wire slt_EX;
   wire byte_load_EX;
   wire word_we_EX;
   wire byte_we_EX;
   wire mem_read_EX;
   wire shift_v_EX;
   wire [1:0] shift_op_EX;
   wire shift_EX;
   wire load_ra_EX;
   wire link_EX;
   wire alu_b_zero_EX;
   wire [2:0] branch_control_EX;
   wire jr_EX;
   wire [31:0] alu_out_EX;
   wire [4:0] rd_num_EX;
   wire [31:0] link_out_EX;
   wire [31:0] alu_out_MEM;
   wire [31:0] rt_data_MEM;
   wire [4:0] rd_num_MEM;
   wire [31:0] link_out_MEM;
   wire [4:0] rs_num_MEM;
   wire [4:0] rt_num_MEM;
   wire [2:0] alu_op_MEM;
   wire wr_enable_MEM;
   wire itype_MEM;
   wire except_MEM;
   wire control_type_MEM;
   wire lui_MEM;
   wire slt_MEM;
   wire byte_load_MEM;
   wire word_we_MEM;
   wire byte_we_MEM;
   wire mem_read_MEM;
   wire shift_v_MEM;
   wire [1:0] shift_op_MEM;
   wire shift_MEM;
   wire load_ra_MEM;
   wire link_MEM;
   wire alu_b_zero_MEM;
   wire [2:0] branch_control_MEM;
   wire jr_MEM;
   wire [31:0] data_out_MEM;
   wire [31:0] alu_out_WB;
   wire [31:0] data_out_WB;
   wire [4:0] rd_num_WB;
   wire [31:0] link_out_WB;
   wire [2:0] alu_op_WB;
   wire wr_enable_WB;
   wire itype_WB;
   wire except_WB;
   wire control_type_WB;
   wire lui_WB;
   wire slt_WB;
   wire byte_load_WB;
   wire word_we_WB;
   wire byte_we_WB;
   wire mem_read_WB;
   wire shift_v_WB;
   wire [1:0] shift_op_WB;
   wire shift_WB;
   wire load_ra_WB;
   wire link_WB;
   wire alu_b_zero_WB;
   wire [2:0] branch_control_WB;
   wire jr_WB;

   wire if_id_reg_enable = ~stall; // only enable the IF_ID register if not stalled
   wire id_ex_reg_enable = 1'b1;
   wire ex_mem_reg_enable = 1'b1;
   wire mem_wb_reg_enable = 1'b1;

   wire if_id_reg_flush = hazard_flush; // only flush the IF/ID register if we took a branch or jump register
   wire id_ex_reg_flush = stall || hazard_flush; // only flush the ID_EX register if we are stalling or took a branch/jump register
   wire ex_mem_reg_flush = 1'b0;
   wire mem_wb_reg_flush = 1'b0; 
   wire hazard_flush; // Handle branch and jr hazard. Flush both IF/ID and ID/EX registers if high

   assign rs_num_ID = inst_ID[25:21];
   assign rt_num_ID = inst_ID[20:16];

   /* Pipeline Changes */ 
   // Need to be reading from Pipeline Registers. Will implement this in other components 
   // Need wr_enable and mem_read in WB stage 
   // Need MIPS instruction to feed into IF_ID register, break apart after 
   // Maybe change order of muxes to rdData? Have them all in EX stage, have mem_read be the last that goes in the WB stage?
   // Have muxes going into rdNum, mux_itype & mux_load_ra, go into EX stage 
   // Branch decision should be taken in EX stage, not ID by the decoder. Maybe have decoder output signal "branch_inst = 1" for branch instructions, then do condition && zero, to determine control type for IF stage 
   //    To build up on the branch decision. We need another module to compare both proposed signals. Then probably another mux that uses previous control_type decision (next instruction or jmp from ID) or the branch 
   //    taken decision (from the EX stage). Essentially more muxes! 
   // Need a mux to take all control signals from MIPS decoder, or set them all to zero depending on Hazard Unit sel signal

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

   // Forward Unit
   forward_unit forward( 
      .rt_num_EX(rt_num_EX), //Input
      .rs_num_EX(rs_num_EX),
      .wr_enable_MEM(wr_enable_MEM),  
      .wr_enable_WB(wr_enable_WB), 
      .rd_num_MEM(rd_num_MEM), 
      .rd_num_WB(rd_num_WB), 
      .forwardA(forwardA), //Output
      .forwardB(forwardB)
   ); 

   // Hazard Unit
   hazard_unit hazard( 
      .mem_read_EX(mem_read_EX),
      .rd_num_EX(rd_num_EX),
      .rs_num_ID(rs_num_ID),
      .rt_num_ID(rt_num_ID), 
      .branch_taken(branch_taken), 
      .jr(jr_EX),
      .stall(stall),
      .flush(hazard_flush)
   ); 

   // Branch Module: EX Stage. Determines if branch is taken based on zero from ALU and branch signal from MIPS decoder
   branch_mod branch_module(
      .zero(zero),
      .negative(negative),
      .branch_control(branch_control_EX),
      .branch_taken(branch_taken)
   );


   // Decoder: EX stage
   mips_decode decoder(
      .alu_op(alu_op_IF),
      .write_enable(wr_enable_IF),
      .itype(itype_IF),
      .except(except),
      .control_type(control_type_IF),
      .mem_read(mem_read_IF),
      .word_we(word_we_IF),
      .byte_we(byte_we_IF),
      .byte_load(byte_load_IF),
      .lui(lui_IF),
      .slt(slt_IF),
      .opcode(inst_IF[31:26]),
      .funct(inst_IF[5:0]),
      .shift(shift_IF),
      .shift_v(shift_v_IF),
      .shift_op(shift_op_IF),
      .load_ra(load_ra_IF),
      .link(link_IF),
      .branch_control(branch_control_IF),
      .jr(jr_IF),
      .alu_b_zero(alu_b_zero_IF)
   );

   // Program Counter: IF stage
   register #(32) PC_reg(
      .q(PC),
      .d(nextPC),
      .clk(clk),
      .enable(pc_write), //Was just enabled. Now will get signal from Hazard Unit
      .reset(reset)
   );

   // Register File: ID stage
   regfile rf (
      .rsNum(inst_ID[25:21]), //Taking from IF/ID pipeline
      .rtNum(inst_ID[20:16]),
      .rdNum(rd_num_WB), //Taking from M/WB pipeline. Determined in EX stage
      .rsData(rs_data_ID),
      .rtData(rt_data_ID),
      .rdData(rd_data), //Switching to output from mem_read mux in WB stage. Last mux to decide which data is written.
      .rdWriteEnable(wr_enable_WB), //Taking from M/WB pipeline. Determined in ID stage
      .clock(clk),
      .reset(reset)
   );

   // Instruction Memory: IF stage
   instruction_memory inst_mem(
      .addr(PC[31:2]),
      .data(inst_IF)
   );

   // Data Memory: M stage
   data_mem data_memory(
      .data_out(data_out_MEM),
      .addr(alu_out_MEM),
      .data_in(rt_data_MEM),
      .word_we(word_we_MEM),
      .byte_we(byte_we_MEM),
      .clk(clk),
      .reset(reset)
   );

   // Shifter Module: EX stage
   shifter shift_mod(
      .inA(rt_data_EX),
      .inB(shift_v_mux_out),
      .shiftOp(shift_op_EX),
      .out(shift_out)
   );

   // MUXES (reference full_machine.pdf for locations) 

   // EX stage - Determine register number in Execution stage. Will carry over values through pipeline registers all the way to WB stage
   mux2v #(.width(5)) mux_itypeA(
      .out(itype_out),
      .A(inst_EX[15:11]),
      .B(inst_EX[20:16]),
      .sel(itype_EX)
   );

   // EX stage - One of the inputs for primary ALU. Either rtData, future stage data, or imm32
   mux2v mux_itypeB(
      .out(itype_out_B),
      .A(forwardB_out), //Switching to output from forwardB_mux. Either RTData or data from future stages
      .B(imm32),
      .sel(itype_EX)
   );

   mux2v mux_alu_b(
      .out(ALU_B),
      .A(itype_out_B),
      .B(32'd0),
      .sel(alu_b_zero_EX)
   );

   // EX stage
   mux2v mux_slt(
      .out(slt_out),
      .A(alu_out_EX),
      .B({ 31'b0, negative }),
      .sel(slt_EX)
   );

   // WB stage
   mux4v #(.width(8)) mux_data_size(
      .out(data_size_out8),
      .A(data_out_WB[7:0]),
      .B(data_out_WB[15:8]),
      .C(data_out_WB[23:16]),
      .D(data_out_WB[31:24]),
      .sel(alu_out_WB[1:0])
   );
   assign data_size_out = {24'd0, data_size_out8};

   // Mem Stage
   mux2v mux_byte_load(
      .out(byte_load_out),
      .A(data_out_WB),
      .B(data_size_out),
      .sel(byte_load_WB)
   );

   // WB Stage. Last mux to decide data for register file
   mux2v mux_mem_read(
      .out(rd_data), //Now is the data written for Register file 
      .A(link_out_WB), //Switching to data output from primary ALU or from SLT, LUI, LINK, etc. muxes pipelined from EX stage
      .B(byte_load_out), //Still byte load out
      .sel(mem_read_WB) 
   );

   // EX stage. Ideally in EX due to Calhoun's design, but the last mux going into rdData needs to be in WB stage!
   mux2v mux_lui(
      .out(lui_out),
      .A(shift_mux_out),
      .B({ inst_EX[15:0], 16'b0 }), //Previously { inst[15:0], 16'b0 }
      .sel(lui_EX)
   );

   // IF stage. NEED TO CHANGE THIS MORE. Should not handle branch, will feed into another mux after that determines if branch offset is applied
   // mux4v mux_control_type(
   //    .out(nextPC),
   //    .A(plus4_alu_out),
   //    .B(branch_alu_out),
   //    .C({ PC[31:28], inst[25:0], 2'b0 }),
   //    .D(rs_data_EX), //From ID/EX pipeline now
   //    .sel(control_type)
   // ); 

   // PC control
   mux2v mux_control_type(
      .out(control_type_mux_out),
      .A(plus4_alu_out_IF),
      .B({ PC[31:28], inst_IF[25:0], 2'b0 }),
      .sel(control_type_IF)
   );

   mux2v mux_branch_taken(
      .out(branch_taken_mux_out),
      .A(control_type_mux_out),
      .B(branch_alu_out),
      .sel(branch_taken)
   );

   mux2v mux_jump_register(
      .out(nextPC),
      .A(branch_taken_mux_out),
      .B(ALU_A),
      .sel(jr_EX)
   );

   // EX stage - Determine register number in Execution stage. Will carry over values through pipeline registers all the way to WB stage
   mux2v mux_shift_v(
      .out(shift_v_mux_out),
      .A({6'd0, inst_EX[25:0]}), //Not best naming convention, but this is from the ID/EX pipeline
      .B(rs_data_EX),
      .sel(shift_v_EX)
   );

   // EX stage
   mux2v mux_shift(
      .out(shift_mux_out),
      .A(slt_out), 
      .B(shift_out),
      .sel(shift_EX)
   );

   // EX stage
   mux2v #(.width(5)) mux_load_ra(
      .out(rd_num_EX),
      .A(itype_out),
      .B(5`R_RA),
      .sel(load_ra_EX)
   );

   // EX stage 
   mux2v mux_link(
      .out(link_out_EX), //Previously rd_data, will need feed data output (alu or slt or lui or link etc.) to EX/MEM pipeline
      .A(lui_out),
      .B(link_alu_out_EX), //Now taking out of ID/EX pipeline. Previously link_alu_out
      .sel(link_EX)
   );

   // ForwardA Mux: EX stage
   mux3v forwardA_mux( 
      .out(ALU_A), 
      .A(rs_data_EX), 
      .B(alu_out_MEM), 
      .C(rd_data), //Output from mem_read MUX
      .sel(forwardA)
   );

   // ForwardB Mux: EX stage
   mux3v forwardB_mux( 
      .out(forwardB_out), 
      .A(rt_data_EX), 
      .B(alu_out_MEM), 
      .C(rd_data), //Output from mem_read MUX
      .sel(forwardB)
   ); 

   // ALUS 

   // Primary ALU: EX stage
   alu32 primary_alu(
      .out(alu_out_EX),
      .overflow(),
      .zero(zero),
      .negative(negative),
      .inA(ALU_A), //RS data or future stage data from forward_A mux
      .inB(ALU_B),
      .control(alu_op_EX)
   );

   // PC+4 ALU: IF stage
   alu32 plus4_alu(
      .out(plus4_alu_out_IF),
      .overflow(),
      .zero(),
      .negative(),
      .inA(PC),
      .inB(32'd4),
      .control(`ALU_ADD)
   );

   // Branch Offset ALU: EX stage
   alu32 branch_alu(
      .out(branch_alu_out),
      .overflow(),
      .zero(),
      .negative(),
      .inA(plus4_alu_out_EX),
      .inB(branch_offset),
      .control(`ALU_ADD)
   );

   // Link ALU (PC + 8): IF stage
   alu32 link_alu(
      .out(link_alu_out_IF),
      .overflow(),
      .zero(),
      .negative(),
      .inA(PC),
      .inB(32'd8),
      .control(`ALU_ADD)
   );

   // Sign Extender: EX stage
   sign_extender extender(
      .in(inst_EX[15:0]),
      .out(imm32)
   );

   // Shift Left 2: EX stage
   shift_left_2 shift_left(
      .in(imm32[29:0]),
      .out(branch_offset)
   );

endmodule // pipelined_machine
