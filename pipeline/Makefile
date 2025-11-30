pipelined_machine: pm.vcd

pipelined_machine_exe: mips_defines.v modules.v rom.v pipelined_machine.v mips_decode.v shifter.v sign_extender.v shift_left_2.v pipeline_registers.v forward_unit.v hazard_unit.v pipelined_machine_tb.v  
	iverilog -o pipelined_machine_exe -Wall pipelined_machine_tb.v

pm.vcd: pipelined_machine_exe memory.text.dat memory.data.dat
	./pipelined_machine_exe


no_deps_or_beq: 
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s no_deps_or_beq.text.dat memory.text.dat
	ln -s no_deps_or_beq.data.dat memory.data.dat


beqs_but_no_deps: 
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s beqs_but_no_deps.text.dat memory.text.dat
	ln -s beqs_but_no_deps.data.dat memory.data.dat

deps_but_no_beqs:
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s deps_but_no_beqs.text.dat memory.text.dat
	ln -s deps_but_no_beqs.data.dat memory.data.dat

pipeline_test:
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s pipeline_test.text.dat memory.text.dat
	ln -s pipeline_test.data.dat memory.data.dat
	



