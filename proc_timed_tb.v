`timescale 1 ns / 100 ps

module proc_timed_tb();

	 integer CYCLE_LIMIT = 100;
	 
    // inputs to the proc are reg type
    reg            clock, reset;

    skeleton dut (clock, reset);

	 wire [11:0] address_imem = dut.address_imem;

    // Dmem
    wire [11:0] address_dmem = dut.address_dmem;
    wire [31:0] data = dut.data;
    wire wren = dut.wren;

    // Regfile
    wire ctrl_writeEnable = dut.ctrl_writeEnable;
    wire [4:0] ctrl_writeReg = dut.ctrl_writeReg;
	 wire [4:0] ctrl_readRegA = dut.ctrl_readRegA;
	 wire [4:0] ctrl_readRegB = dut.ctrl_readRegB;
    wire [31:0] data_writeReg = dut.data_writeReg;

	 
    initial
    begin
        $display($time, "<< Starting the Simulation >>");
        clock = 1'b0;    // at time 0
		  
		  // multdiv with branching
		  //$monitor("clock: %d, pc: %d, address_imem: %d, ctrl_readRegA: %d, ctrl_readRegB: %d, data_writeReg: %d, ctrl_writeReg: %d, ctrl_writeEnable: %d, isBranch, %d, branch_value: %d", 
		  //clock, pc, address_imem, ctrl_readRegA, ctrl_readRegB, data_writeReg, ctrl_writeReg, ctrl_writeEnable, isBranch, branch_value);
		  
		  // multdiv
		  //$monitor("clock: %d, pc: %d, data_a: %d, data_b: %d, result_mult: %d, ctrl_MULT: %d, data_resultRDY: %d, multdiv_result: %d, o_in_x_sel: %d, isStillMultDiv: %d",
		  //clock, pc, data_a, data_b, result_mult, ctrl_MULT, data_resultRDY, multdiv_result, o_in_x_sel, isStillMultDiv);

		  // branching monitor
		  //$monitor("pc: %d, alu_1: %d, alu_2: %d, isBranch: %d, branch_value: %d, pc_branch_sel: %d, ctrl_writeEnable: %d, ctrl_writeReg: %d, data_writeReg: %d",
		  //pc, alu_input_1, alu_input_2, isBranch, branch_value, pc_branch_select, ctrl_writeEnable, ctrl_writeReg, data_writeReg);
		  
		  // processor output monitor
		  $monitor("address_imem: %d, ctrl_readRegA: %d, ctrl_readRegB: %d, data_writeReg: %d, ctrl_writeReg: %d, ctrl_writeEnable: %d, address_dmem: %d, data: %d, wren: %d", 
		  address_imem, ctrl_readRegA, ctrl_readRegB, data_writeReg, ctrl_writeReg, ctrl_writeEnable, address_dmem, data, wren);
		  
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