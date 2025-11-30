// Test module for the sign extender.
// David Bootle

`include "sign_extender.v"

module sign_extender_tb;

    reg [15:0] num;
    wire [31:0] out;

    sign_extender extender(.in(num), .out(out));

    initial begin
        $dumpfile("test.vcd");
        $dumpvars(0, sign_extender_tb);

        #3 num = 16'h0123;
        #3 num = 16'hF000;
        $finish;
    end


endmodule