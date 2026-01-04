# Generates the verilog file for the pipeline registers.
# David Bootle 2025

from io import TextIOWrapper
from jinja2 import Template

# Registers, defined as a list of tuples (name: str, bit_length: int)

DECODER_SIGNALS = [
    ('alu_op', 3),
    ('wr_enable', 1),
    ('itype', 1),
    ('except', 1),
    ('control_type', 1),
    ('lui', 1),
    ('slt', 1),
    ('byte_load', 1),
    ('word_we', 1),
    ('byte_we', 1),
    ('mem_read', 1),
    ('shift_v', 1),
    ('shift_op', 2),
    ('shift', 1),
    ('load_ra', 1),
    ('link', 1),
    ('alu_b_zero', 1),
    ('branch_control', 3),
    ('jr', 1)
]

IF_ID_SIGNALS = [
    ('inst', 32),
    ('link_alu_out', 32),
    ('plus4_alu_out', 32),
    *DECODER_SIGNALS    # this syntax unpacks the variable and adds all items in it to the list
]

ID_EX_SIGNALS = [
    ('inst', 32),
    ('rs_data', 32),
    ('rt_data', 32),
    ('link_alu_out', 32),
    ('rs_num', 5),
    ('rt_num', 5),
    ('plus4_alu_out', 32),
    *DECODER_SIGNALS
]

EX_MEM_SIGNALS = [
    ('alu_out', 32),
    ('rt_data', 32),
    ('rd_num', 5),
    ('link_out', 32),
    ('rs_num', 5),
    ('rt_num', 5),
    *DECODER_SIGNALS
]

MEM_WB_SIGNALS = [
    ('alu_out', 32),
    ('data_out', 32),
    ('rd_num', 5),
    ('link_out', 32),
    *DECODER_SIGNALS
]

class PipelineRegister:

    signals: list[tuple[str, int]]
    in_stage: str
    out_stage: str
    name: str

    def __init__(self, signals, in_stage, out_stage):
        self.signals = signals
        self.in_stage = in_stage
        self.out_stage = out_stage
        self.name = f'{str.lower(in_stage)}_{str.lower(out_stage)}'
    
    def generate_module(self, file: TextIOWrapper):
        '''
        Generate the code for a single pipeline register
        file - The file to write to, must already be opened
        signals - The list of signals defined as tuples of (name: str, bit_length: int)
        in_stage - The input stage of the register i.e. 'EX'
        out_stage - The output stage of the register i.e. 'MEM'
        '''

        # Open template file, render it, and write it to the output file
        with open('pipeline_templates/pipeline_module.v.jinja') as template_file:
            template = Template(template_file.read())
            file.write(template.render(signals=self.signals, name=self.name) + '\n\n')
        
    
    def generate_imports(self, file: TextIOWrapper):
        '''
        Generate the code for a single pipeline register import.
        This is designed to be copied and used as import.
        '''

        # Open template file, render it, and write it to the output file
        with open('pipeline_templates/pipeline_imports.v.jinja') as template_file:
            template = Template(template_file.read())
            file.write(template.render(signals=self.signals, name=self.name, in_stage=self.in_stage, out_stage=self.out_stage) + '\n\n')
    
    def get_wire_list(self) -> list[tuple[str, int]]:
        # loop through each signal and generate in and out wires for the module
        all_wires: list[tuple[str, int]]

        # define lambda functions
        in_name = lambda signal: (f'{signal[0]}_{self.in_stage}', signal[1])
        out_name = lambda signal: (f'{signal[0]}_{self.out_stage}', signal[1])

        # create input and output wire lists
        input_wires = list(map(in_name, self.signals))
        output_wires = list(map(out_name, self.signals))

        # combine lists
        return [*input_wires, *output_wires]

        
    def generate_input_wires(self, file: TextIOWrapper):
        '''
        Generates the verilog definitions for the input wires into this pipeline module.
        '''
        # Open template file, render it, and write it to the output file
        with open('pipeline_templates/pipeline_input_wires.v.jinja') as template_file:
            template = Template(template_file.read())
            file.write(template.render(signals=self.signals, in_stage=self.in_stage) + '\n\n')

if __name__ == '__main__':

    IF_ID = PipelineRegister(IF_ID_SIGNALS, 'IF', 'ID')
    ID_EX = PipelineRegister(ID_EX_SIGNALS, 'ID', 'EX')
    EX_MEM = PipelineRegister(EX_MEM_SIGNALS, 'EX', 'MEM')
    MEM_WB = PipelineRegister(MEM_WB_SIGNALS, 'MEM', 'WB')

    # generate pipeline registers
    with open('pipeline_registers.auto.v', 'w+') as file:
        IF_ID.generate_module(file)
        ID_EX.generate_module(file)
        EX_MEM.generate_module(file)
        MEM_WB.generate_module(file)
    
    # generate pipeline register imports
    with open('pipeline_registers.imports.v', 'w+') as file:
        IF_ID.generate_imports(file)
        ID_EX.generate_imports(file)
        EX_MEM.generate_imports(file)
        MEM_WB.generate_imports(file)
    
    # generate wire definitions
    with open('pipeline_registers.wires.v', 'w+') as file:
        # combine all inputs and outputs from each register
        all_wires: list[tuple[str, int]] = []
        for register in [IF_ID, ID_EX, EX_MEM, MEM_WB]:
            all_wires += register.get_wire_list()
            
        
        # deduplicate the list
        tmp = []
        for wire in all_wires:
            if wire in tmp: # if wire already exists in the new list, skip it
                continue
            tmp.append(wire)
        all_wires = tmp

        # all_wires += [
        #     ('if_id_reg_enable', 1),
        #     ('id_ex_reg_enable', 1),
        #     ('ex_mem_reg_enable', 1),
        #     ('mem_wb_reg_enable', 1),
        #     ('if_id_reg_flush', 1),
        #     ('id_ex_reg_flush', 1),
        #     ('ex_mem_reg_flush', 1),
        #     ('mem_wb_reg_flush', 1)
        # ]

        # generate file with all definitions
        template = Template('{%for wire in wires%}wire {%if wire[1] > 1%}[{{wire[1]-1}}:0] {%endif%}{{wire[0]}};\n{%endfor%}')

        # write to file
        with open('pipeline_registers.wires.v', 'w+') as file:
            file.write(template.render(wires = all_wires))
                    