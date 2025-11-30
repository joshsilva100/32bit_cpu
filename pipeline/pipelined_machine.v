`define CONTROL_NEXT    2'd0    // Add 4 to the program counter
`define CONTROL_BRANCH  2'd1    // Offset address by inst[15:0]*4 + 4
`define CONTROL_JUMP    2'd2    // Jump to {PC[31:28],inst[25:0],00}
`define CONTROL_JUMPR   2'd3    // Jump to address loaded from rs_data

// pipelined_machine: execute a series of MIPS instructions from an instruction cache
//
// except (output) - set to 1 when an unrecognized instruction is to be executed.
// clk     (input) - the clock signal
// reset   (input) - set to 1 to set all registers to zero, set to 0 for normal execution.

module pipelined_machine(except, clk, reset);
   output except;
   input clk, reset;

   wire [31:0] 	inst;  
   wire [31:0] 	PC;
   wire [31:0]    nextPC;

   // decoder signals
   wire [2:0] alu_op;
   wire wr_enable;
   wire itype;
   wire [1:0] control_type;
   wire lui;
   wire slt;
   wire byte_load;
   wire word_we;
   wire byte_we;
   wire mem_read;
   wire [1:0] shift_op;
   wire shift_v;
   wire shift;
   wire load_ra;
   wire link;
   wire except;

   // register file signals
   wire [31:0] rs_data;
   wire [31:0] rt_data;
   wire [31:0] rd_data;
   wire [4:0] rdest;
   
   // primary alu signals
   wire [31:0] alu_out;
   wire zero;
   wire overflow;
   wire negative;
   wire [31:0] ALU_B;

   // plus4 alu signals
   wire [31:0] plus4_alu_out;
   wire [31:0] branch_alu_out;
   wire [31:0] branch_offset;

   // data memory signals
   wire [31:0] data_out;

   // misc wires
   wire [31:0] imm32;
   wire [31:0] slt_out;
   wire [7:0] data_size_out8;
   wire [31:0] data_size_out;
   assign data_size_out = { 24'b0, data_size_out8 }; // change from 8bit output to 32bit by padding
   wire [31:0] byte_load_out;
   wire [31:0] mem_read_out;
   wire [31:0] shift_v_mux_out;
   wire [31:0] shift_mux_out;
   wire [31:0] shift_out;
   wire [4:0] itype_out;
   wire [31:0] lui_out;
   wire [31:0] link_alu_out; 

   // pipeline wires 
   wire [31:0] if_id_inst; 
   wire [31:0] if_id_link_alu_out; 
   wire [31:0] id_ex_link_alu_out; 
   wire [1:0] forwardA; //Sel signal
   wire [1:0] forwardB; //Sel signal
   wire [31:0] forwardA_out; //Output from the forwardA mux
   wire [31:0] forwardB_out; //Output from the forwardB mux
   wire [31:0] id_ex_rsData; //RS from ID_EX stage
   wire [31:0] id_ex_rtData; //RT from ID_EX stage
   wire [31:0] ex_mem_alu_out; //Result from EX_MEM stage 
   wire [4:0] id_ex_rt; 
   wire [4:0] id_ex_rd; 
   wire [4:0] id_ex_rs; 
   wire control_sel; //Hazard unit determing Control signals from module  
   wire if_id_write; //Hazard unit stalling IF/ID register
   wire pc_write; //Hazard unit stalling Program Counter
   wire mux_shift_v_A; //Adding this to handle transition to EX stage from ID stage 
   wire id_ex_wr_enable; 
   wire id_ex_word_we; 
   wire id_ex_byte_we;
   wire id_ex_byte_load;
   wire id_ex_itype;
   wire id_ex_load_ra;
   wire [2:0] id_ex_alu_op;
   wire id_ex_slt;
   wire id_ex_shift;
   wire id_ex_lui;
   wire id_ex_link; 
   wire [31:0] id_ex_imm32; 
   wire [15:0] id_ex_lui_data; 
   wire ex_mem_wr_enable; 
   wire ex_mem_mem_read; 
   wire ex_mem_word_we; 
   wire ex_mem_byte_we; 
   wire ex_mem_byte_load; 
   wire [4:0] ex_mem_rdNum; 
   wire [31:0] ex_mem_alu_out; 
   wire [31:0] ex_mem_rtData;

   wire mem_wb_wr_enable; 
   wire mem_wb_mem_read; 
   wire [4:0] mem_wb_rdNum; 
   wire [31:0] mem_wb_alu_out; 
   wire [31:0] mem_wb_data_out;


   /* Pipeline Changes */ 
   // Need to be reading from Pipeline Registers. Will implement this in other components 
   // Need wr_enable and mux line for mem_read, lui, slt etc. in the WB stage 
   // Need MIPS instruction to feed into IF_ID register, break apart after 
   // Maybe change order of muxes to rdData? Have them all in EX stage, have mem_read be the last that goes in the WB stage?
   // Have muxes going into rdNum, mux_itype & mux_load_ra, go into EX stage 
   // Branch decision should be taken in EX stage, not ID by the decoder. Maybe have decoder output signal "branch_inst = 1" for branch instructions, then do condition && zero, to determine control type for IF stage 
   //    To build up on the branch decision. We need another module to compare both proposed signals. Then probably another mux that uses previous control_type decision (next instruction or jmp from ID) or the branch 
   //    taken decision (from the EX stage). Essentially more muxes! 
   // Need mux to take all control signals from MIPS decoder, or set them all to zero.

   // IF/ID Register
   if_id_reg if_id( 
      .clk(clk), 
      .flush(reset),
      .if_id_write(if_id_write),
      .inst(inst), 
      .link_data(link_alu_out), 
      .out_inst(if_id_inst), 
      .out_link_data(if_id_link_alu_out)
   ); 

   // ID/EX Register
   id_ex_reg id_ex( 
      .clk(clk), //Input
      .flush(reset),
      .write(wr_enable), 
      .mem_read(mem_read), 
      .word_we(word_we), 
      .byte_we(byte_we), 
      .byte_load(byte_load),
      .i_type(itype),
      .load_ra(load_ra), 
      .alu_op(alu_op),
      .slt(slt),
      .shift(shift), 
      .lui(lui), 
      .link(link),
      .rsData(rs_data), 
      .rtData(rt_data),
      .imm32(imm32),
      .rt(if_id_inst[20:16]),
      .rd(if_id_inst[15:11]),
      .rs(if_id_inst[25:21]), 
      .mux_shift_v_A({ 27'b0, inst[10:6] }),
      .lui_data(if_id_inst[15:0]), 
      .link_data(if_id_link_alu_out), 
      .out_write(id_ex_wr_enable), //Output
      .out_mem_read(id_ex_mem_read), 
      .out_word_we(id_ex_word_we), 
      .out_byte_we(id_ex_byte_we), 
      .out_byte_load(id_ex_byte_load),
      .out_i_type(id_ex_itype),
      .out_load_ra(id_ex_load_ra), 
      .out_alu_op(id_ex_alu_op),
      .out_slt(id_ex_slt),
      .out_shift(id_ex_shift), 
      .out_lui(id_ex_lui), 
      .out_link(id_ex_link),
      .out_rsData(id_ex_rsData), 
      .out_rtData(id_ex_rtData),
      .out_imm32(id_ex_imm32),
      .out_rt(id_ex_rt),
      .out_rd(id_ex_rd),
      .out_rs(id_ex_rs), 
      .out_mux_shift_v_A(mux_shift_v_A),
      .out_lui_data(id_ex_lui_data), 
      .out_link_data(id_ex_link_alu_out)
   ); 

   // EX/MEM Register
   ex_mem_reg ex_mem( 
      .clk(clk), //Input
      .flush(reset),
      .write(id_ex_wr_enable), 
      .mem_read(id_ex_mem_read), 
      .word_we(id_ex_word_we), 
      .byte_we(id_ex_byte_we), 
      .byte_load(id_ex_byte_load), 
      .rdNum(rdest), //Last output from itype, load_ra (Output from 'RegDst')
      .aluOut(), //Do this later. Last output from alu, lui, slt, link, etc. muxes 
      .rtData(), //Do this later as well. Either RT data or whatever else comes out of the forwardB mux
      .out_write(ex_mem_wr_enable), //Output
      .out_mem_read(ex_mem_mem_read), 
      .out_word_we(ex_mem_word_we), 
      .out_byte_we(ex_mem_byte_we), 
      .out_byte_load(ex_mem_byte_load), 
      .out_rdNum(ex_mem_rdNum), 
      .out_aluOut(ex_mem_alu_out), 
      .out_rtData(ex_mem_rtData)
   );

   // MEM/WB Register
   mem_wb_reg mem_wb( 
      .clk(clk), //Input
      .flush(reset),
      .write(ex_mem_wr_enable), 
      .mem_read(ex_mem_mem_read),
      .mem_data(data_out), //From Data Memory
      .rdNum(ex_mem_rdNum), 
      .aluOut(ex_mem_alu_out),
      .out_write(mem_wb_wr_enable), //Output
      .out_mem_read(mem_wb_mem_read),
      .out_mem_data(mem_wb_data_out),
      .out_rdNum(mem_wb_rdNum), 
      .out_aluOut(mem_wb_alu_out)
   );

   // Forward Unit
   forward_unit forward( 
      .id_ex_rt(id_ex_rt), //Input
      .id_ex_rs(id_ex_rs), 
      .ex_mem_write(ex_mem_wr_enable), 
      .ex_mem_mem_read(ex_mem_mem_read), 
      .mem_wb_write(mem_wb_wr_enable), 
      .mem_wb_mem_read(mem_wb_mem_read), 
      .ex_mem_rd(ex_mem_rdNum), 
      .mem_wb_rd(mem_wb_rdNum), 
      .forwardA(forwardA), //Output
      .forwardB(forwardB)
   ); 

   // Hazard Unit
   hazard_unit hazard( 
      .rs(if_id_inst[25:21]), //Input
      .rt(if_id_inst[20:16]), 
      .id_ex_memRead(id_ex_mem_read), 
      .id_ex_rt(id_ex_rt), 
      .control_sel(control_sel), //Output
      .if_id_write(if_id_write), 
      .pc_write(pc_write)
   );
   
   // ForwardA Mux: EX stage
   mux3v forwardA_mux( 
      .out(forwardA_out), 
      .A(id_ex_rs), 
      .B(ex_mem_alu_out), 
      .C(mem_read_out), 
      .sel(forwardA)
   ); 

   // ForwardB Mux: EX stage
   mux3v forwardB_mux( 
      .out(forwardB_out), 
      .A(id_ex_rt), 
      .B(ex_mem_alu_out), 
      .C(mem_read_out), 
      .sel(forwardB)
   ); 

   // Decoder: ID stage
   mips_decode decoder(
      .alu_op(alu_op),
      .write_enable(wr_enable),
      .itype(itype),
      .except(except),
      .control_type(control_type),
      .mem_read(mem_read),
      .word_we(word_we),
      .byte_we(byte_we),
      .byte_load(byte_load),
      .lui(lui),
      .slt(slt),
      .opcode(if_id_inst[31:26]),
      .funct(if_id_inst[5:0]),
      .zero(zero),
      .shift(shift),
      .shift_v(shift_v),
      .shift_op(shift_op),
      .load_ra(load_ra),
      .link(link)
   );

   // Program Counter. IF stage
   register #(32) PC_reg(
      .q(PC),
      .d(nextPC),
      .clk(clk),
      .enable(1'b1),
      .reset(reset)
   );

   // Register File. ID stage
   regfile rf (
      .rsNum(if_id_inst[25:21]),
      .rtNum(if_id_inst[20:16]),
      .rdNum(rdest),
      .rsData(rs_data),
      .rtData(rt_data),
      .rdData(rd_data),
      .rdWriteEnable(wr_enable),
      .clock(clk),
      .reset(reset)
   );

   // Instruction Memory. IF stage
   instruction_memory inst_mem(
      .addr(PC[31:2]),
      .data(inst)
   );

   // Data Memory. MEM stage
   data_mem data_memory(
      .data_out(data_out),
      .addr(alu_out),
      .data_in(rt_data),
      .word_we(word_we),
      .byte_we(byte_we),
      .clk(clk),
      .reset(reset)
   );

   // Shifter Module. EX stage
   shifter shift_mod(
      .inA(rt_data),
      .inB(shift_v_mux_out),
      .shiftOp(shift_op),
      .out(shift_out)
   );

   // MUXES (reference full_machine.pdf for locations) 

   // EX stage - Determine register number in Execution stage. Will carry over values through pipeline registers all the way to WB stage
   mux2v #(.width(5)) mux_itypeA(
      .out(itype_out),
      .A(if_id_inst[15:11]),
      .B(if_id_inst[20:16]),
      .sel(itype)
   );

   // EX stage - Determine register number in Execution stage. Will carry over values through pipeline registers all the way to WB stage 
   mux2v mux_itypeB(
      .out(ALU_B),
      .A(rt_data),
      .B(imm32),
      .sel(itype)
   );

   // EX stage
   mux2v mux_slt(
      .out(slt_out),
      .A(alu_out),
      .B({ 31'b0, negative }),
      .sel(slt)
   );

   // MEM stage
   mux4v #(.width(8)) mux_data_size(
      .out(data_size_out8),
      .A(data_out[7:0]),
      .B(data_out[15:8]),
      .C(data_out[23:16]),
      .D(data_out[31:24]),
      .sel(alu_out[1:0])
   );

   // Mem Stage
   mux2v mux_byte_load(
      .out(byte_load_out),
      .A(data_out),
      .B(data_size_out),
      .sel(byte_load)
   );

   // Ideally WB stage. WB stage for now
   mux2v mux_mem_read(
      .out(mem_read_out),
      .A(slt_out),
      .B(byte_load_out),
      .sel(mem_read)
   );

   // EX stage. Ideally in EX due to Calhoun's design, but the last mux going into rdData needs to be in WB stage!
   mux2v mux_lui(
      .out(lui_out),
      .A(shift_mux_out),
      .B({ inst[15:0], 16'b0 }),
      .sel(lui)
   );

   // IF stage
   mux4v mux_control_type(
      .out(nextPC),
      .A(plus4_alu_out),
      .B(branch_alu_out),
      .C({ PC[31:28], inst[25:0], 2'b0 }),
      .D(rs_data),
      .sel(control_type)
   );

   // EX stage - Determine register number in Execution stage. Will carry over values through pipeline registers all the way to WB stage
   mux2v mux_shift_v(
      .out(shift_v_mux_out),
      .A(mux_shift_v_A),
      .B(rs_data),
      .sel(shift_v)
   );

   // EX stage
   mux2v mux_shift(
      .out(shift_mux_out),
      .A(mem_read_out),
      .B(shift_out),
      .sel(shift)
   );

   // EX stage
   mux2v #(.width(5)) mux_load_ra(
      .out(rdest),
      .A(itype_out),
      .B(`R_RA),
      .sel(load_ra)
   );

   // EX stage 
   mux2v mux_link(
      .out(rd_data),
      .A(lui_out),
      .B(link_alu_out),
      .sel(link)
   );

   // ALUS 

   // Primary ALU 
   // EX stage
   alu32 primary_alu(
      .out(alu_out),
      .overflow(overflow),
      .zero(zero),
      .negative(negative),
      .inA(rs_data),
      .inB(ALU_B),
      .control(alu_op)
   );

   // PC+4 ALU 
   // IF stage
   alu32 plus4_alu(
      .out(plus4_alu_out),
      .overflow(),
      .zero(),
      .negative(),
      .inA(PC),
      .inB(32'd4),
      .control(`ALU_ADD)
   );

   // Branch Offset ALU 
   // IF stage
   alu32 branch_alu(
      .out(branch_alu_out),
      .overflow(),
      .zero(),
      .negative(),
      .inA(plus4_alu_out),
      .inB(branch_offset),
      .control(`ALU_ADD)
   );

   // Link ALU (PC + 8) 
   // IF stage
   alu32 link_alu(
      .out(link_alu_out),
      .overflow(),
      .zero(),
      .negative(),
      .inA(PC),
      .inB(32'd8),
      .control(`ALU_ADD)
   );

   // Sign Extender 
   // ID stage
   sign_extender extender(
      .in(if_id_inst[15:0]),
      .out(imm32)
   );

   // Shift Left 2 
   // EX stage. mux_control_type needs rsData anyways
   shift_left_2 shift_left(
      .in(imm32[29:0]),
      .out(branch_offset)
   );

endmodule // pipelined_machine
