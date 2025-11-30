// register: A register which may be reset to an arbirary value
//
// q      (output) - Current value of register
// d      (input)  - Next value of register
// clk    (input)  - Clock (positive edge-sensitive)
// enable (input)  - Load new value? (yes = 1, no = 0)
// reset  (input)  - Synchronous reset    (reset = 1)
//
module register(q, d, clk, enable, reset);

   parameter
	    width = 32,
	    reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0] 	q;
   input [(width-1):0] 	d;
   input 		clk, enable, reset;

   always@(posedge clk)
     if (reset == 1'b1)
       q <= reset_value;
     else if (enable == 1'b1)
       q <= d;

endmodule // register

////////////////////////////////////////////////////////////////////////
//
// Module: regfile
//
// Description:
//   A behavioral MIPS register file.  R0 is hardwired to zero.
//   Given that you won't write behavioral code, don't worry if you don't
//   understand how this works;  We have to use behavioral code (as 
//   opposed to the structural code you are writing), because of the 
//   latching by the the register file.
//
module regfile (rsNum, rtNum, rdNum, 
              	rsData, rtData, rdData, 
                rdWriteEnable, clock, reset);

   input [4:0]          rsNum, rtNum, rdNum;
   input [31:0]  	rdData;
   input                rdWriteEnable, clock, reset;
   output [31:0] 	rsData, rtData;
   
   reg [31:0] 	r[0:31];
   integer 		i;

   always@(reset)
     if(reset == 1'b1)
       begin
	  r[0] <= 32'b0;
	  for(i = 1; i <= 31; i = i + 1)
	    r[i] <= 32'h10010000;
       end

   assign rsData = r[rsNum];
   assign rtData = r[rtNum];

   always@(posedge clock)
     begin
	if((reset == 1'b0) && (rdWriteEnable == 1'b1) && (rdNum != 5'b0))
	  r[rdNum] <= rdData;
     end

endmodule // regfile_3port

`define ALU_ADD    3'h0
`define ALU_SUB    3'h1
`define ALU_AND    3'h2
`define ALU_OR     3'h3
`define ALU_SLT    3'h4

////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// inA (input)  - Operand modified by the operation
//// inB (input)  - Operand used (in arithmetic ops) to modify inA
//// control (input) - Selects which operation is to be performed
////
module alu32(out, overflow, zero, negative, inA, inB, control);
   output [31:0] out;
   output       overflow, zero, negative;
   input [31:0] inA, inB;
   input  [2:0] control;

   assign out = (({32{(control == `ALU_AND)}} & (inA & inB)) |
		 ({32{(control == `ALU_OR)}} & (inA | inB)) |
		 ({32{(control == `ALU_XOR)}} & (inA ^ inB)) |
		 ({32{(control == `ALU_NOR)}} & ~(inA | inB)) |
		 ({32{(control == `ALU_ADD)}} & (inA + inB)) |
		 ({32{(control == `ALU_SUB)}} & (inA - inB)));
   assign zero = (out[31:18] == 14'b0) & (out[17:1] == 17'd0) & ~out[0];
   xor  x1(negative, out[31], 1'b0);
   assign overflow = 1'b0;  // we're just not computing this here because we don't need it.
   
endmodule

module adder30(out, in1, in2);
   output [29:0] out;
   input [29:0] in1, in2;

   assign out = in1 + in2;
endmodule


