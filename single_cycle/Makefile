full_machine: fm.vcd

full_machine_exe: mips_defines.v rf.v alu32.v rom.v full_machine.v full_machine_tb.v shifter.v sign_extender.v shift_left_2.v
	iverilog -o full_machine_exe -Wall full_machine_tb.v

fm.vcd: full_machine_exe memory.text.dat memory.data.dat
	./full_machine_exe


all_test: 
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s all.text.dat memory.text.dat
	ln -s all.data.dat memory.data.dat


lwbr_test: 
	rm -f memory.text.dat
	rm -f memory.data.dat
	ln -s lwbr2.text.dat memory.text.dat
	ln -s lwbr2.data.dat memory.data.dat

memory.text.dat:
	cp all.text.dat memory.text.dat

memory.data.dat:
	cp all.data.dat memory.data.dat