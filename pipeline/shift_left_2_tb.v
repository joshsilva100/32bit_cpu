// Test module for the left shifter.
// David Bootle

`include "shift_left_2.v"

module shift_left_2_tb;

    reg [29:0] in;
    wire [31:0] out;

    shift_left_2 shifter(.in(in), .out(out));

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, shift_left_2_tb);

        #3 in = 30'h4FFFFFFF;
        $finish;
    end


endmodule