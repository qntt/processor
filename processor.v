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
	 
	 wire [4:0] rd_m, rs_m, rt_m;
	 wire [4:0] rd_w, rs_w, rt_w;
	 
	 
	//========================================= Fetch Stage
	
	wire negclock;
	assign negclock = ~clock;
	 
	wire [31:0] pc;
	wire [31:0] ir_fd, pc_fd, next_pc;
	 
	//dff_fd dff_fd1 (.ir_in(q_imem), .pc_in(next_pc), .clk(clock), .clrn(1'b1), .prn(1'b1), 
	//	.ena(1'b1), .ir(ir_fd), .pc(pc_fd));
	
	latch_fd latch_fd1 (.ir_in(q_imem), .pc_in(next_pc), .clock(clock), .reset(reset), 
		.ir_out(ir_fd), .pc_out(pc_fd));
		
	wire [31:0] pc_data_in;
	assign pc_data_in = next_pc;
	//dflipflop pc_dff (.d(pc_data_in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(pc));
	latch_pc latch_pc1 (.pc_in(pc_data_in), .clock(clock), .reset(reset), .pc_out(pc));
	assign address_imem = pc;
	
	alu alu_next_pc (.data_operandA(pc), .data_operandB(32'd1), .ctrl_ALUopcode(5'b00000),
		.ctrl_shiftamt(5'b00000), .data_result(next_pc), .isNotEqual(), 
		.isLessThan(), .overflow());
		
	//========================================= Decode Stage	
	
	wire [31:0] ir_dx, pc_dx, a_dx, b_dx, a_out_regfile, b_out_regfile;
	
	//dff_dx dff_dx1 (.ir_in(ir_fd), .pc_in(pc_fd), .a_in(a_out_regfile), .b_in(b_out_regfile), 
	//	.clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .ir(ir_dx), .pc(pc_dx), .a(a_dx), .b(b_dx));
		
	latch_dx latch_dx1 (.ir_in(ir_fd), .pc_in(pc_fd), .a_in(a_out_regfile), .b_in(b_out_regfile), 
		.clock(clock), .reset(reset), .ir_out(ir_dx), .pc_out(pc_dx), .a_out(a_dx), .b_out(b_dx));
		
	wire [4:0] opd;
	assign opd = ir_fd[31:27];
	
	wire [4:0] rd_d, rs_d, rt_d;
	assign rd_d = ir_fd[26:22];
	assign rs_d = ir_fd[21:17];
	assign rt_d = ir_fd[16:12];
	
	wire isSW_d;
	assign isSW_d =  ~opd[4] & ~opd[3] & opd[2] & opd[1] & opd[0];
	
	assign ctrl_readRegA = rs_d;
	// if sw, then b_dx is value of rd
	assign ctrl_readRegB = isSW_d ? rd_d : rt_d;
	
	wire isR_d;
	assign isR_d = ~opd[4] & ~opd[3] & ~opd[2] & ~opd[1] & ~opd[0];
	
	// bypassing the updated write register value if write address matches rd
	// similar to writing to register before reading in the same clock cycle
	assign a_out_regfile = data_readRegA;
	wire rdNotEqualWriteAddress;
	assign rdNotEqualWriteAddress = (rd_d[4]^ctrl_writeReg[4] | rd_d[3]^ctrl_writeReg[3] |
		rd_d[2]^ctrl_writeReg[2] | rd_d[1]^ctrl_writeReg[1] | rd_d[0]^ctrl_writeReg[0]);
	assign b_out_regfile = (~rdNotEqualWriteAddress & isSW_d)
		? data_writeReg : data_readRegB;	
	
	//========================================= Execute Stage
	
	wire [31:0] ir_xm, o_xm, b_xm, alu_out;
	
	//dff_xm dff_xm1 (.ir_in(ir_dx), .o_in(alu_out), .b_in(b_dx), .clk(clock), .clrn(1'b1), 
	//	.prn(1'b1), .ena(1'b1), .ir(ir_xm), .o(o_xm), .b(b_xm));
		
	wire [4:0] opx;
	assign opx = ir_dx[31:27];
	
	wire [4:0] rd_x, rs_x, rt_x;
	assign rd_x = ir_dx[26:22];
	assign rs_x = ir_dx[21:17];
	assign rt_x = ir_dx[16:12];
	
	wire [4:0] aluop = ir_dx[6:2];
	wire [4:0] shamt = ir_dx[11:7];
	wire [16:0] immediate = ir_dx[16:0];
	
	wire isNotEqual_x, isLessThan_x, overflow_x;
	
	wire [31:0] signextend;
	assign signextend[16:0] = immediate[16:0];
	assign signextend[31:17] = immediate[16] ? 15'b111111111111111 : 15'b0;
	
	wire isI_x;
	// 00101 | 00111 | 01000
	// addi | sw | lw
	assign isI_x = (~opx[4]&~opx[3]&opx[2]&~opx[1]&opx[0]) | (~opx[4]&~opx[3]&opx[2]&opx[1]&opx[0])
		| (~opx[4]&opx[3]&~opx[2]&~opx[1]&~opx[0]);
		
	wire isSW_x;
	assign isSW_x =  ~opx[4] & ~opx[3] & opx[2] & opx[1] & opx[0];
	
	wire [31:0] alu_input_1;
	wire [31:0] pre_alu_input_2;
	wire [31:0] alu_input_2;
	assign alu_input_2 = isI_x ? signextend : pre_alu_input_2;
	
	wire [4:0] final_aluop = isI_x ? 5'b00000 : aluop;
	
	// ====== MX Bypassing
	
	wire MX1;
	equality5 mx1_eq (.out(MX1), .a(rd_m), .b(rs_x));
	
	wire MX2a, MX2b, MX2;
	equality5 mx2a_eq (.out(MX2a), .a(rd_m), .b(rt_x));
	equality5 mx2b_eq (.out(MX2b), .a(rd_m), .b(rd_x));
	assign MX2 = MX2a | (MX2b & isSW_x);
	
	// ====== WX Bypassing
	
	wire WX1;
	equality5 wx1_eq (.out(WX1), .a(rd_w), .b(rs_x));
	
	wire WX2a, WX2b, WX2;
	equality5 wx2a_eq (.out(WX2a), .a(rd_w), .b(rt_x));
	equality5 wx2b_eq (.out(WX2b), .a(rd_w), .b(rd_x));
	assign WX2 = WX2a | (WX2b & isSW_x);
	
	// ====== Integrating MX and WX bypassing
	
	// selector for mux that determines alu_input_1 and alu_input_2
	wire [1:0] sel1, sel1_wx, sel1_mx;
	assign sel1 = 2'b10;
	assign sel1_wx = WX1 ? 2'b01 : sel1;
	assign sel1_mx = MX1 ? 2'b00 : sel1_wx;
	
	wire [1:0] sel2, sel2_wx, sel2_mx;
	assign sel2 = 2'b10;
	assign sel2_wx = WX2 ? 2'b01 : sel2;
	assign sel2_mx = MX2 ? 2'b00 : sel2_wx;
	
	mux_4_1 mux_alu_input_1 (
		.out(alu_input_1), 
		.in0(o_xm), .in1(data_writeReg), .in2(a_dx), .in3(a_dx),
		.select(sel1_mx));
		
	mux_4_1 mux_alu_input_2 (
		.out(pre_alu_input_2), 
		.in0(o_xm), .in1(data_writeReg), .in2(b_dx), .in3(b_dx),
		.select(sel2_mx));
	
	
	alu alu1 (.data_operandA(alu_input_1), .data_operandB(alu_input_2), .ctrl_ALUopcode(final_aluop),
		.ctrl_shiftamt(shamt), .data_result(alu_out), .isNotEqual(isNotEqual_x), 
		.isLessThan(isLessThan_x), .overflow(overflow_x));
	
	latch_xm latch_xm1 (.ir_in(ir_dx), .o_in(alu_out), .b_in(pre_alu_input_2), .clock(clock), 
		.reset(reset), .ir_out(ir_xm), .o_out(o_xm), .b_out(b_xm));
	
	
	//========================================= Memory Stage
	
	wire [31:0] ir_mw, o_mw, d_mw;
	
	//dff_mw dff_mw1 (.ir_in(ir_xm), .o_in(o_xm), .d_in(q_dmem), .clk(clock), 
	//	.clrn(1'b1), .prn(1'b1), .ena(1'b1), .ir(ir_mw), .o(o_mw), .d(d_mw));
	
	latch_mw latch_mw1 (.ir_in(ir_xm), .o_in(o_xm), .d_in(q_dmem), .clock(clock), .reset(reset), 
		.ir_out(ir_mw), .o_out(o_mw), .d_out(d_mw));
		
	wire [4:0] opm;
	assign opm = ir_xm[31:27];
	
	// rd_m is defined at the top
	assign rd_m = ir_xm[26:22];
	assign rs_m = ir_xm[21:17];
	assign rt_m = ir_xm[16:12];
	
	wire isR_m, isALU;
	assign isR_m = ~opm[4] & ~opm[3] & ~opm[2] & ~opm[1] & ~opm[0];
	
	assign address_dmem = o_xm[11:0];
   assign data = b_xm;
	
	wire isSW_m;
	assign isSW_m =  ~opm[4] & ~opm[3] & opm[2] & opm[1] & opm[0];
   assign wren = isSW_m;
	
	
	//========================================= Write-back Stage
	
	wire [4:0] opw;
	assign opw = ir_mw[31:27];
	
	// rd_w is defined at the top
	assign rd_w = ir_mw[26:22];
	assign rs_w = ir_mw[21:17];
	assign rt_w = ir_mw[16:12];
	
	// Regfile
	wire isALUOp;
	// 00000 | 00101
   assign isALUOp = (~opw[4]&~opw[3]&~opw[2]&~opw[1]&~opw[0]) 
		| (~opw[4]&~opw[3]&opw[2]&~opw[1]&opw[0]) ;
	assign isLoadOp = (~opw[4]&opw[3]&~opw[2]&~opw[1]&~opw[0]);
	
	assign ctrl_writeEnable = isALUOp | isLoadOp;
	
   assign ctrl_writeReg = rd_w;
   assign data_writeReg = isLoadOp ? d_mw : o_mw;
	
	
	 

endmodule 

module equality5 (out, a, b);

	input [4:0] a, b;
	output out;

	assign out = ~(a[4]^b[4] | a[3]^b[3] | a[2]^b[2] | a[1]^b[1] | a[0]^b[0]);

endmodule 