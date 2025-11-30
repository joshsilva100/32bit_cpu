// shift_left_2
// Module that shifts a 30 bit input left by two,
// filling with zeros
//
// in   (input)
// out  (output)
//
// David Bootle

module shift_left_2(in, out);

    input [29:0] in;
    output [31:0] out;

    assign out = {in, 2'b00};

endmodule