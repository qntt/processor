`timescale 1 ns / 100 ps

module proc_tb();

	 integer CYCLE_LIMIT = 100;
	 
    // inputs to the proc are reg type
    reg            clock, reset;

    skeleton dut (clock, reset);

    wire[31:0] pc = dut.my_processor.pc;
	 wire[31:0] o_xm = dut.my_processor.o_xm;
	 wire[31:0] b_xm = dut.my_processor.b_xm;
	 wire[31:0] a_dx = dut.my_processor.a_dx;
	 wire isI_x = dut.my_processor.isI_x;
	 wire[31:0] signextend = dut.my_processor.signextend;
	 
	 wire[31:0] d_mw = dut.my_processor.d_mw;
	 wire[31:0] b_out_regfile = dut.my_processor.b_out_regfile;
	 wire sel2 = dut.my_processor.sel2;
	 wire MX1 = dut.my_processor.MX1;
	 wire WX1 = dut.my_processor.WX1;
	 wire MX2 = dut.my_processor.MX2;
	 wire WX2 = dut.my_processor.WX2;
	 
	 wire [11:0] address_imem = dut.my_processor.address_imem;

    // Dmem
    wire [11:0] address_dmem = dut.my_processor.address_dmem;
    wire [31:0] data = dut.my_processor.data;
    wire wren = dut.my_processor.wren;

    // Regfile
    wire ctrl_writeEnable = dut.my_processor.ctrl_writeEnable;
    wire [4:0] ctrl_writeReg = dut.my_processor.ctrl_writeReg;
	 wire [4:0] ctrl_readRegA = dut.my_processor.ctrl_readRegA;
	 wire [4:0] ctrl_readRegB = dut.my_processor.ctrl_readRegB;
    wire [31:0] data_writeReg = dut.my_processor.data_writeReg;
	 
	 wire [31:0] alu_out = dut.my_processor.alu_out;
	 
	 
	 // branch testing
	 wire [1:0] pc_branch_select = dut.my_processor.pc_branch_select;
	 wire isBranch = dut.my_processor.isBranch;
	 wire [31:0] branch_value = dut.my_processor.branch_value;
	 wire [31:0] alu_input_1 = dut.my_processor.alu_input_1;
	 wire [31:0] alu_input_2 = dut.my_processor.alu_input_2;
	 
	 
    initial
    begin
        $display($time, "<< Starting the Simulation >>");
        clock = 1'b0;    // at time 0
		  
		  // branching monitor
		  //$monitor("pc: %d, alu_1: %d, alu_2: %d, isBranch: %d, branch_value: %d, pc_branch_sel: %d, ctrl_writeEnable: %d, ctrl_writeReg: %d, data_writeReg: %d",
		  //pc, alu_input_1, alu_input_2, isBranch, branch_value, pc_branch_select, ctrl_writeEnable, ctrl_writeReg, data_writeReg);
		  
		  // processor output monitor
		  $monitor("pc: %d, address_imem: %d, ctrl_readRegA: %d, ctrl_readRegB: %d, data_writeReg: %d, ctrl_writeReg: %d, ctrl_writeEnable: %d, address_dmem: %d, data: %d, wren: %d", pc, address_imem, ctrl_readRegA, ctrl_readRegB, data_writeReg, ctrl_writeReg, ctrl_writeEnable, address_dmem, data, wren);
		  
		  //$monitor("pc: %d, address_imem: %d, ctrl_readRegA: %d, ctrl_readRegB: %d, data_writeReg: %d, ctrl_writeReg: %d, ctrl_writeEnable: %d, address_dmem: %d, data: %d, wren: %d, isBranch: %d, branch_value: %d, alu_out: %d\n\n\n", pc, address_imem, ctrl_readRegA, ctrl_readRegB, data_writeReg, ctrl_writeReg, ctrl_writeEnable, address_dmem, data, wren, isBranch, branch_value, alu_out);
        
		  
        //$monitor("clock: %d, pc: %d, a_dx: %d, o_xm: %b, isI_x: %d, signextend: %b, alu_input_2: %b", clock, pc, a_dx, o_xm, isI_x, signextend, alu_input_2);
		  
		  //$monitor("pc: %d, b_out_regfile: %d, sel2_mx: %d, o_xm: %d, b_xm: %d, d_mw: %d, MX1: %d, WX1: %d, MX2: %d, WX2: %d", pc, b_out_regfile, sel2_mx, o_xm, b_xm, d_mw, MX1, WX1, MX2, WX2);
		  
		  #(20*(CYCLE_LIMIT+1.5))

        $stop;
    end

    // Clock generator
    always
         #10     clock = ~clock;
endmodule