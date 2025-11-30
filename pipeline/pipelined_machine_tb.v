`include "mips_defines.v"
`include "rom.v"
`include "modules.v"
`include "mux_lib.v"
`include "pipelined_machine.v"

module test;
   /* Make a regular pulsing clock. */
   reg       clk = 0;
   always #5 clk = !clk;
   integer     i;

   reg 		reset = 1, done = 0;
   wire         except;
   
   pipelined_machine pm(except, clk, reset);
   
   initial begin
      $dumpfile("pm.vcd");
      $dumpvars(0, test);
      # 13 reset = 0;
      # 300 done = 1;
   end
   
  initial
     $monitor("At time %t, reset = %d pc = %h, inst = %h",
              $time, reset, pm.PC, pm.inst);

   // periodically check for the end of simulation.  When it happens
   // dump the register file contents.
   always @(negedge clk)
     begin
	#0;
	if (done === 1'b1)
	begin
           $display ( "Dumping register state: " );
	   $display ( "  Register :  hex-value (  dec-value )" );
           for (i = 0 ; i < 32 ; i = i + 1)
 	     $display ( "%d: 0x%x ( %d )", i, pm.rf.r[i], pm.rf.r[i]);
           $display ( "Done.  Simulation ending." );
	   $finish;
	end
     end

   
endmodule // test
