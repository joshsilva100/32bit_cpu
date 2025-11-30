// shifter
// Module that can perform arithmetic and logical shifts.
//
// inA  (input)
// inB  (input)
// shiftOp  (input)
// out  (output)
//
// David Bootle

`define SHIFT_LEFT              2'd0
`define SHIFT_RIGHT_LOGICAL     2'd1
`define SHIFT_RIGHT_ARITHMETIC  2'd2

module shifter(inA, inB, shiftOp, out);
    
    input [31:0] inA;
    input [31:0] inB;
    input [1:0] shiftOp;
    output reg [31:0] out;

    always @(*) begin

        case (shiftOp)

            `SHIFT_LEFT: begin
                out = inA << inB;
            end

            `SHIFT_RIGHT_LOGICAL: begin
                out = inA >> inB;
            end

            `SHIFT_RIGHT_ARITHMETIC: begin
                out = inA >>> inB;
            end

        endcase
    end

endmodule