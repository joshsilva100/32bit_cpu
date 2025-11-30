// forward_unit: Implement forwarding to pipeline
// id_ex_rt (input) - RT from ID/EX Register
// id_ex_rs (input) - RS from ID/EX Register
// ex_mem_write (input) - Write Enable from the EX/MEM Register
// ex_mem_mem_read (input) - Mem_Read Select Signal from the EX/MEM Register
// mem_wb_write (input) - Write Enable from the MEM/WB Register 
// mem_wb_mem_read (input) - Mem_Read Select Signal from the MEM/WB Register
// ex_mem_rd (input) - RD from EX_MEM Register
// mem_wb_rd (input) - RD from MEM_WB Register
// forwardA (output) - Sel for ALU input 1 src (Read data 1)
// forwardB (output) - Sel for ALU input 2 src (Read data 2)

module forward_unit(id_ex_rt, id_ex_rs, ex_mem_write, ex_mem_mem_read, mem_wb_write, mem_wb_mem_read, ex_mem_rd, mem_wb_rd, 
                    forwardA, forwardB); 

    input [4:0] id_ex_rt, id_ex_rs, ex_mem_rd, mem_wb_rd;
    input ex_mem_write, ex_mem_mem_read, mem_wb_write, mem_wb_mem_read;
    output forwardA, forwardB;

endmodule