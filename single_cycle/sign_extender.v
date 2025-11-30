// sign_extender
// Module to extend a signed 16-bit number into a signed
// 32-bit number
//
// in   (input)     - 16bit number input
// out  (output)    - 32bit number output
//
// David Bootle

module sign_extender(in, out);

    input [15:0] in;
    output [31:0] out;

    assign out = { {16{in[15]}}, in };

endmodule