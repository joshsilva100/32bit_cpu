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