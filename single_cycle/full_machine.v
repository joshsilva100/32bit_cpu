// mips_decode: a decoder for MIPS arithmetic instructions
//
// alu_op      (output) - control signal to be sent to the ALU
// writeenable (output) - should a new value be captured by the register file
// itype       (output) - should the ALU receive 1 reg. value and 1 immediate (1) or 2 reg values (0)
// except      (output) - set to 1 when we don't recognize an opdcode & funct combination
// control_type[1:0] (output) - 00 = fallthrough, 01 = branch_target, 10 = jump_target, 11 = jump_register 
// mem_read    (output) - the register value written is coming from the memory
// word_we     (output) - we're writing a word's worth of data
// byte_we     (output) - we're only writing a byte's worth of data
// byte_load   (output) - we're doing a byte load
// lui         (output) - the instruction is a lui
// slt         (output) - the instruction is an slt
// opcode       (input) - the opcode field from the instruction
// funct        (input) - the function field from the instruction
// zero         (input) - from the ALU
//

`define CONTROL_NEXT    2'd0    // Add 4 to the program counter
`define CONTROL_BRANCH  2'd1    // Offset address by inst[15:0]*4 + 4
`define CONTROL_JUMP    2'd2    // Jump to {PC[31:28],inst[25:0],00}
`define CONTROL_JUMPR   2'd3    // Jump to address loaded from rs_data

module mips_decode(alu_op, write_enable, itype, except, control_type,
		   mem_read, word_we, byte_we, byte_load, lui, slt, 
		   opcode, funct, zero, shift, shift_v, shift_op, load_ra, link);
	output reg [2:0] alu_op;
   output reg [1:0] shift_op;
	output reg write_enable, itype, except;
	output reg [1:0] control_type;
	output reg mem_read, word_we, byte_we, byte_load, lui, slt;
   output reg shift, shift_v;
   output reg load_ra, link;
	input [5:0] opcode, funct;
	input zero;

	// Concat MIPs instruction (Not in use yet. Will determine if decoding method changes)
	// wire [12:0] instruction;
	// assign instruction = {zero,opcode,zero};

	always @(*) begin 
		// Set up default values 
		alu_op = `ALU_ADD;            // By default, ALU is set to add for easier debugging.
		write_enable = 1'b0;          // When 1, writes rdest with rd_data.
		itype = 1'bx;                 // If 0, uses rd as write address and rtData as ALU B. If 1, uses
                                    // ... rt as write address and immediate as ALU B.
		except = 1'b0;                // Should always be zero unless decode failure
		control_type = `CONTROL_NEXT; // By default, go to next instruction
		mem_read = 1'b0;              // When 1, loads memory from current alu_out
		word_we = 1'b0;               // When 1, writes a word of rtData into alu_out
		byte_we = 1'b0;               // When 1, writes a byte of rtData into alu_out
		byte_load = 1'b0;             // When 1, uses byte select mux instead of loading entire word
		lui = 1'b0;                   // When 1, sets rdData to inst[15:0]
		slt  = 1'b0;                  // When 1, sets writeback line to the negative flag of primary alu
      shift = 1'b0;                 // When 1, sets writeback line to output of the shift module
      shift_v = 1'b0;               // When 1, loads inB of shifter module with rs instead of shamt
      shift_op = 2'd0;              // By default, shifter will shift left for easy debugging.
      load_ra = 1'b0;               // When 1, forces rd_num to 31 (return address register)
      link = 1'b0;                  // When 1, sets writeback line to PC + 8.

		// Decode MIPS instruction - TODO (Decide between nested case and concat) (ALSO DO ALL THE INSTRUCTIONS!) 
		// K-Map would require 2^12 or 2^13 (4096 or 8192 cells :D)
		case (opcode) // instruction = {zero,opcode,funct} or {opcode,funct} if we get rid of nested case (probably not bad, mux to a mux and they're cheap)
			/* R Type Instruction: opcode is always zero */
			6'h00: begin
				itype = 1'b0;  // Set itype to 0 as these are r type
				case (funct)

               // SLL Instruction: R[rd] = R[rt] << imm (shamt)
               // shift left by a constant amount.
               `OP0_SLL: begin
                  write_enable = 1'b1;
                  shift = 1'b1;
                  shift_op = `SHIFT_LEFT;
               end

               // SRL Instruction: R[rd] = R[rt] >>> imm (shamt)
               // shift right unsigned (logical) by a constant amount.
               `OP0_SRL: begin
                  write_enable = 1'b1;
                  shift = 1'b1;
                  shift_op = `SHIFT_RIGHT_LOGICAL;
               end

               // SRA Instruction: R[rd] = R[rt] >> imm (shamt)
               // shift right arithmetic by a constant amount.
               `OP0_SRL: begin
                  write_enable = 1'b1;
                  shift = 1'b1;
                  shift_op = `SHIFT_RIGHT_ARITHMETIC;
               end

               // SLLV Instruction: R[rd] = R[rt] << R[rs]
               // shift left by the amount in a register.
               `OP0_SLLV: begin
                  write_enable = 1'b1;
                  shift_v = 1'b1;
                  shift = 1'b1;
                  shift_op = `SHIFT_LEFT;
               end

               // SRLV Instruction: R[rd] = R[rt] >>> R[rs]
               // shift right unsigned (logical) by the amount in a register.
               `OP0_SRLV: begin
                  write_enable = 1'b1;
                  shift = 1'b1;
                  shift_v = 1'b1;
                  shift_op = `SHIFT_RIGHT_LOGICAL;
               end

               // SRAV Instruction: R[rd] = R[rt] >> R[rs]
               // shift right arithmetic by the amount in a register.
               `OP0_SRAV: begin
                  write_enable = 1'b1;
                  shift = 1'b1;
                  shift_v = 1'b1;
                  shift_op = `SHIFT_RIGHT_ARITHMETIC;
               end

					// JR Instruction: PC = R[rs]
               // Jump Register
					`OP0_JR: begin
						control_type = `CONTROL_JUMPR;
					end

               // JALR Instruction: 
               // Jump and Link Register
               `OP0_JALR: begin
                  control_type = `CONTROL_JUMPR;
                  link = 1'b1;
                  write_enable = 1'b1;
               end
					
					// ADD Instruction: R[rd] = R[rs] + R[rt]
					`OP0_ADD: begin 
						alu_op = `ALU_ADD; 
						write_enable = 1'b1; //Write to register
					end

					// ADDU Instruction: R[rd] = R[rs] + R[rt]
					`OP0_ADDU: begin 
						alu_op = `ALU_ADD; 
						write_enable = 1'b1; //Write to register
					end 
					
					// SUB Instruction: R[rd] = R[rs] - R[rt] 
					`OP0_SUB: begin 
						alu_op = `ALU_SUB; 
						write_enable = 1'b1; //Write to register 
					end 

					// SUBU Instruction: R[rd] = R[rs] - R[rt] 
					`OP0_SUBU: begin 
						alu_op = `ALU_SUB; 
						write_enable = 1'b1; //Write to register 
					end 

					// AND Instruction: R[rd] = R[rs] & R[rt]
					`OP0_AND: begin 
						alu_op = `ALU_AND; 
						write_enable = 1'b1; //Write to register
					end 

					// OR Instruction: R[rd] = R[rs] | R[rt]
					`OP0_OR: begin 
						alu_op = `ALU_OR; 
						write_enable = 1'b1; //Write to register
					end  

					// XOR Instruction: R[rd] = R[rs] ^ R[rt]
					`OP0_XOR: begin 
						alu_op = `ALU_XOR; 
						write_enable = 1'b1; //Write to register
					end  

					// NOR Instruction: R[rd] = ~(R[rs] | R[rt])
					`OP0_NOR: begin 
						alu_op = `ALU_NOR; 
						write_enable = 1'b1; //Write to register
					end

               // SLT Instruction: R[rd] = (R[rs] < R[rt]) ? 1 : 0
               // set less than
               `OP0_SLT: begin
                  alu_op = `ALU_SUB;
                  slt = 1'b1;
                  write_enable = 1'b1;
               end

               // SLTU Instruction: R[rd] = (R[rs] < R[rt]) ? 1 : 0
               // set less than unsigned
               `OP0_SLTU: begin
                  alu_op = `ALU_SUB;
                  slt = 1'b1;
                  write_enable = 1'b1;
               end

				endcase 
			end
			
			/* J Type Instruction: opcode = 0x02 (j) */ 
			// jump: PC = JumpAddr
			`OP_J: begin
				control_type = `CONTROL_JUMP; // JumpAddr = Inst[25:0] 
			end
			
			// jump and link: R[31] = PC + 8; PC = JumpAddr
			`OP_JAL: begin 
				control_type = `CONTROL_JUMP; // JumpAddr = Inst[25:0] 
            write_enable = 1'b1;
            link = 1'b1;
            load_ra = 1'b1;
				// Work on R[31] = PC + 8. How are we implementing this in hardware?
			end

         // branch on equal: if(R[rs]==R[rt]) PC=PC+4+immediate*4
         `OP_BEQ: begin
            control_type = zero ? `CONTROL_BRANCH : `CONTROL_NEXT;
            itype = 0;
            alu_op = `ALU_SUB;
         end

         // branch not equal: if(R[rs]!=R[rt]) PC=PC+4+immediate*4
         `OP_BNE: begin
            control_type = zero ? `CONTROL_NEXT : `CONTROL_BRANCH;
            itype = 0;
            alu_op = `ALU_SUB;
         end

         // branch less than zero: if(R[rs]<0) PC=PC+4+immediate*4
         `OP_BLEZ: begin
            except = 1; // TODO: add hardware to support this instruction
         end

         // branch greater than zero: if (R[rs]>0) PC=PC+4+immediate*4
         `OP_BGTZ: begin
            except = 1; // TODO: add hardware to support this instruction
         end
			
			/* I Type Instruction: opcode is anything else */
			// ADDI: R[rt] = R[rs] + SignExtImm 
         // add immediate
			`OP_ADDI: begin 
				alu_op = `ALU_ADD; 
				write_enable = 1'b1; //Write to register
            itype = 1'b1;
			end 

			// ADDIU: R[rt] = R[rs] + SignExtImm
         // add immediate unsigned
			`OP_ADDIU: begin 
				alu_op = `ALU_ADD; 
				write_enable = 1'b1; //Write to register
				itype = 1'b1;
			end 

         // SLTI: R[rt] = (R[rs] < imm) ? 1 : 0
         // set less than immediate
         `OP_SLTI: begin
            alu_op = `ALU_SUB;
            write_enable = 1'b1;
            slt = 1'b1;
            itype = 1'b1;
         end

         // SLTIU: R[rt] = (R[rs] < imm) ? 1 : 0
         // set less than immediate unsigned
         `OP_SLTIU: begin
            alu_op = `ALU_SUB;
            write_enable = 1'b1;
            slt = 1'b1;
            itype = 1'b1;
         end

			// ANDI Instruction: R[rt] = R[rs] & ZeroExtImm
         // and immediate
			`OP_ANDI: begin 
				alu_op = `ALU_AND; 
				write_enable = 1'b1; //Write to register
            itype = 1'b1;
			end 

			// ORI Instruction: R[rt] = R[rs] | ZeroExtImm
         // or immediate
			`OP_ORI: begin 
				alu_op = `ALU_OR; 
				write_enable = 1'b1; //Write to register
				itype = 1'b1;
			end  

			// XORI Instruction: R[rt] = R[rs] ^ ZeroExtImm
         // xor immediate
			`OP_XORI: begin 
				alu_op = `ALU_XOR; 
				write_enable = 1'b1; //Write to register
				itype = 1'b1;
			end

         // LUI Instruction: R[rt] = {imm, 16'b0}
         // load unsigned immediate
         `OP_LUI: begin
            alu_op = `ALU_ADD;
            write_enable = 1'b1;
            lui = 1'b1;
            itype = 1'b1;
         end

         // LB Instruction: R[rt] = M[R[rs] + imm] (one byte)
         // load byte
         // TODO: implement hardware to load bytes signed
         `OP_LB: begin
            alu_op = `ALU_ADD;
            write_enable = 1'b1;
            mem_read = 1'b1;
            byte_load = 1'b1;
            itype = 1'b1;
         end

         // LBU Instruction: R[rt] = M[R[rs] + imm] (one byte)
         // load byte unsigned
         `OP_LBU: begin
            alu_op = `ALU_ADD;
            write_enable = 1'b1;
            mem_read = 1'b1;
            byte_load = 1'b1;
            itype = 1'b1;
         end

         // LW Instruction: R[rt] = M[R[rs] + imm]
         // load word
         `OP_LW: begin
            alu_op = `ALU_ADD;
            write_enable = 1'b1;
            mem_read = 1'b1;
            itype = 1'b1;
         end

         // SB Instruction: M[R[rs]+imm](7:0) = R[rt](7:0)
         // store byte
         `OP_SB: begin
            alu_op = `ALU_ADD;
            byte_we = 1'b1;
         end

         // SW Instruction: M[R[rs]+imm] = R[rt]
         // store word
         `OP_SW: begin
            alu_op = `ALU_ADD;
            word_we = 1'b1;
         end
			
			// Unrecognized instruction: Return exception
			default: begin
				except = 1'b1; 
			end
		endcase 
	end
 

endmodule // mips_decode


// full_machine: execute a series of MIPS instructions from an instruction cache
//
// except (output) - set to 1 when an unrecognized instruction is to be executed.
// clk     (input) - the clock signal
// reset   (input) - set to 1 to set all registers to zero, set to 0 for normal execution.

module full_machine(except, clk, reset);
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

   // Decoder
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
      .opcode(inst[31:26]),
      .funct(inst[5:0]),
      .zero(zero),
      .shift(shift),
      .shift_v(shift_v),
      .shift_op(shift_op),
      .load_ra(load_ra),
      .link(link)
   );

   // Program Counter
   register #(32) PC_reg(
      .q(PC),
      .d(nextPC),
      .clk(clk),
      .enable(1'b1),
      .reset(reset)
   );

   // Register File
   regfile rf (
      .rsNum(inst[25:21]),
      .rtNum(inst[20:16]),
      .rdNum(rdest),
      .rsData(rs_data),
      .rtData(rt_data),
      .rdData(rd_data),
      .rdWriteEnable(wr_enable),
      .clock(clk),
      .reset(reset)
   );

   // Instruction Memory
   instruction_memory inst_mem(
      .addr(PC[31:2]),
      .data(inst)
   );

   // Data Memory
   data_mem data_memory(
      .data_out(data_out),
      .addr(alu_out),
      .data_in(rt_data),
      .word_we(word_we),
      .byte_we(byte_we),
      .clk(clk),
      .reset(reset)
   );

   // Shifter Module
   shifter shift_mod(
      .inA(rt_data),
      .inB(shift_v_mux_out),
      .shiftOp(shift_op),
      .out(shift_out)
   );

   // MUXES (reference full_machine.pdf for locations)
   mux2v #(.width(5)) mux_itypeA(
      .out(itype_out),
      .A(inst[15:11]),
      .B(inst[20:16]),
      .sel(itype)
   );

   mux2v mux_itypeB(
      .out(ALU_B),
      .A(rt_data),
      .B(imm32),
      .sel(itype)
   );

   mux2v mux_slt(
      .out(slt_out),
      .A(alu_out),
      .B({ 31'b0, negative }),
      .sel(slt)
   );

   mux4v #(.width(8)) mux_data_size(
      .out(data_size_out8),
      .A(data_out[7:0]),
      .B(data_out[15:8]),
      .C(data_out[23:16]),
      .D(data_out[31:24]),
      .sel(alu_out[1:0])
   );

   mux2v mux_byte_load(
      .out(byte_load_out),
      .A(data_out),
      .B(data_size_out),
      .sel(byte_load)
   );

   mux2v mux_mem_read(
      .out(mem_read_out),
      .A(slt_out),
      .B(byte_load_out),
      .sel(mem_read)
   );

   mux2v mux_lui(
      .out(lui_out),
      .A(shift_mux_out),
      .B({ inst[15:0], 16'b0 }),
      .sel(lui)
   );

   mux4v mux_control_type(
      .out(nextPC),
      .A(plus4_alu_out),
      .B(branch_alu_out),
      .C({ PC[31:28], inst[25:0], 2'b0 }),
      .D(rs_data),
      .sel(control_type)
   );

   mux2v mux_shift_v(
      .out(shift_v_mux_out),
      .A({ 27'b0, inst[10:6] }),
      .B(rs_data),
      .sel(shift_v)
   );

   mux2v mux_shift(
      .out(shift_mux_out),
      .A(mem_read_out),
      .B(shift_out),
      .sel(shift)
   );

   mux2v #(.width(5)) mux_load_ra(
      .out(rdest),
      .A(itype_out),
      .B(`R_RA),
      .sel(load_ra)
   );

   mux2v mux_link(
      .out(rd_data),
      .A(lui_out),
      .B(link_alu_out),
      .sel(link)
   );

   // ALUS
   // Primary ALU
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
   sign_extender extender(
      .in(inst[15:0]),
      .out(imm32)
   );

   // Shift Left 2
   shift_left_2 shift_left(
      .in(imm32[29:0]),
      .out(branch_offset)
   );

endmodule // full_machine
