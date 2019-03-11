/**
 * READ THIS DESCRIPTION!
 *
 * The processor takes in several inputs from a skeleton file.
 *
 * Inputs
 * clock: this is the clock for your processor at 50 MHz
 * reset: we should be able to assert a reset to start your pc from 0 (sync or
 * async is fine)
 *
 * Imem: input data from imem
 * Dmem: input data from dmem
 * Regfile: input data from regfile
 *
 * Outputs
 * Imem: output control signals to interface with imem
 * Dmem: output control signals and data to interface with dmem
 * Regfile: output control signals and data to interface with regfile
 *
 * Notes
 *
 * Ultimately, your processor will be tested by subsituting a master skeleton, imem, dmem, so the
 * testbench can see which controls signal you active when. Therefore, there needs to be a way to
 * "inject" imem, dmem, and regfile interfaces from some external controller module. The skeleton
 * file acts as a small wrapper around your processor for this purpose.
 *
 * You will need to figure out how to instantiate two memory elements, called
 * "syncram," in Quartus: one for imem and one for dmem. Each should take in a
 * 12-bit address and allow for storing a 32-bit value at each address. Each
 * should have a single clock.
 *
 * Each memory element should have a corresponding .mif file that initializes
 * the memory element to certain value on start up. These should be named
 * imem.mif and dmem.mif respectively.
 *
 * Importantly, these .mif files should be placed at the top level, i.e. there
 * should be an imem.mif and a dmem.mif at the same level as process.v. You
 * should figure out how to point your generated imem.v and dmem.v files at
 * these MIF files.
 *
 * imem
 * Inputs:  12-bit address, 1-bit clock enable, and a clock
 * Outputs: 32-bit instruction
 *
 * dmem
 * Inputs:  12-bit address, 1-bit clock, 32-bit data, 1-bit write enable
 * Outputs: 32-bit data at the given address
 *
 */
module processor(
    // Control signals
    clock,                          // I: The master clock
    reset,                          // I: A reset signal

    // Imem
    address_imem,                   // O: The address of the data to get from imem
    q_imem,                         // I: The data from imem

    // Dmem
    address_dmem,                   // O: The address of the data to get or put from/to dmem
    data,                           // O: The data to write to dmem
    wren,                           // O: Write enable for dmem
    q_dmem,                         // I: The data from dmem

    // Regfile
    ctrl_writeEnable,               // O: Write enable for regfile
    ctrl_writeReg,                  // O: Register to write to in regfile
    ctrl_readRegA,                  // O: Register to read from port A of regfile
    ctrl_readRegB,                  // O: Register to read from port B of regfile
    data_writeReg,                  // O: Data to write to for regfile
    data_readRegA,                  // I: Data from port A of regfile
    data_readRegB                   // I: Data from port B of regfile
);
    // Control signals
    input clock, reset;

    // Imem
    output [11:0] address_imem;
    input [31:0] q_imem;

    // Dmem
    output [11:0] address_dmem;
    output [31:0] data;
    output wren;
    input [31:0] q_dmem;

    // Regfile
    output ctrl_writeEnable;
    output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
    output [31:0] data_writeReg;
    input [31:0] data_readRegA, data_readRegB;
	 
	 
	//========================================= Fetch Stage
	 
	wire [31:0] pc, ir_fd, pc_fd;
	 
	dff_fd dff_fd1 (.ir_in(q_imem), .pc_in(pc), .clk(clock), .clrn1(1'b1), .prn(1'b1), 
		.ena(1'b1), .ir(ir_fd), .pc(pc_fd));
		
	//========================================= Decode Stage	
	
	wire [31:0] ir_dx, pc_dx, a_dx, b_dx, a_out_regfile, b_out_regfile;
	
	dff_dx dff_dx1 (.ir_in(ir_fd), .pc_in(pc_fd), .a_in(a_out_regfile), .b_in(b_out_regfile), 
		.clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .ir(ir_dx), .pc(pc_dx), .a(a_dx), .b(b_dx));
		
	wire [4:0] opcode_d;
	assign opcode_d = ir_fd[31:27];
	
	wire [4:0] rd_d, rs_d, rt_d;
	assign rd_d = ir_fd[26:22];
	assign rs_d = ir_fd[21:17];
	assign rt_d = ir_fd[16:12];
	
	assign ctrl_readRegA = rs_d;
	assign ctrl_readRegB = rt_d;
	
	wire isR_d;
	assign isR_d = ~opcode_d[4] & ~opcode_d[3] & ~opcode_d[2] & ~opcode_d[1] & ~opcode_d[0];
	
	assign a_dx = data_readRegA;
	// assign register value or immediate value
	assign b_dx[15:0] = isR_d ? data_readRegB : ir_fd[16:0]; 
	
	//========================================= Execute Stage
	
	wire [31:0] ir_xm, o_xm, b_xm, o_out_alu;
	
	dff_xm dff_xm1 (.ir_in(ir_dx), .o_in(o_out_alu), .b_in(b_dx), .clk(clock), .clrn(1'b1), 
		.prn(1'b1), .ena(1'b1), .ir(ir_xm), .o(o_xm), .b(b_xm));
		
	wire [4:0] opcode_x;
	assign opcode_x = ir_dx[31:27];
	
	wire [4:0] rd_x, rs_x, rt_x;
	assign rd_x = ir_dx[26:22];
	assign rs_x = ir_dx[21:17];
	assign rt_x = ir_dx[16:12];
	
	wire [4:0] aluop = ir_dx[6:2];
	
	wire isR_x;
	assign isR_x = ~opcode_x[4] & ~opcode_x[3] & ~opcode_x[2] & ~opcode_x[1] & ~opcode_x[0];
	
	
	//========================================= Memory Stage
	
	wire [31:0] ir_mw, o_mw, d_mw;
	
	dff_mw dff_mw1 (.ir_in(ir_xm), .o_in(o_xm), .d_in(q_dmem), .clk(clock), 
		.clrn(1'b1), .prn(1'b1), .ena(1'b1), .ir(ir_mw), .o(o_mw), .d(d_mw));
	
	
	//========================================= Write-back Stage
	 

endmodule 
