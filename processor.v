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
	 
	 wire isSW_m;
	 wire isLW_m;
	 wire isALUOp_m;
	 wire isAddi_m;
	 wire isBne_m;
	 wire isJr_m;
	 wire isBlt_m;
	 wire isR_m;
	 wire isI_m;
	 
	 wire isSW_w;
	 wire isLW_w;
	 wire isALUOp_w;
	 wire isAddi_w;
	 wire isBne_w;
	 wire isJr_w;
	 wire isBlt_w;
	 wire isR_w;
	 wire isI_w;
	 
	 wire [31:0] branch_value;
	 wire isBranch;
	 
	 wire [31:0] noop;
	 assign noop = 32'b0;
	 
	//========================================= Fetch Stage
	
	wire negclock;
	assign negclock = ~clock;
	 
	wire [31:0] pc;
	wire [31:0] ir_fd, pc_fd, next_pc;
	
	wire [31:0] ir_in_fd;
	assign ir_in_fd = isBranch ? noop : q_imem;
	
	latch_fd latch_fd1 (.ir_in(ir_in_fd), .pc_in(next_pc), .clock(clock), .reset(reset), 
		.ir_out(ir_fd), .pc_out(pc_fd));
		
	wire [31:0] pc_data_in;
	assign pc_data_in = isBranch ? branch_value : next_pc;
	//dflipflop pc_dff (.d(pc_data_in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(pc));
	latch_pc latch_pc1 (.pc_in(pc_data_in), .clock(clock), .reset(reset), .pc_out(pc));
	assign address_imem = pc;
	
	alu alu_next_pc (.data_operandA(pc), .data_operandB(32'd1), .ctrl_ALUopcode(5'b00000),
		.ctrl_shiftamt(5'b00000), .data_result(next_pc), .isNotEqual(), 
		.isLessThan(), .overflow(), .carry_in(1'b0));
		
	//========================================= Decode Stage	
	
	wire [31:0] ir_dx, pc_dx, a_dx, b_dx, a_out_regfile, b_out_regfile;
	
	wire [31:0] ir_in_dx;
	assign ir_in_dx = isBranch ? noop : ir_fd;
		
	latch_dx latch_dx1 (.ir_in(ir_in_dx), .pc_in(pc_fd), .a_in(a_out_regfile), .b_in(b_out_regfile), 
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
		
	wire [4:0] opx;
	assign opx = ir_dx[31:27];
	
	wire [4:0] rd_x, rs_x, rt_x;
	assign rd_x = ir_dx[26:22];
	assign rs_x = ir_dx[21:17];
	assign rt_x = ir_dx[16:12];
	
	wire [4:0] aluop;
	assign aluop = ir_dx[6:2];
	wire [4:0] shamt;
	assign shamt = ir_dx[11:7];
	wire [16:0] immediate;
	assign immediate = ir_dx[16:0];
	wire [26:0] T_x;
	assign T_x = ir_dx[26:0];
	
	wire isNotEqual_x, isLessThan_x, overflow_x;
	
	wire [31:0] signextend;
	assign signextend[16:0] = immediate[16:0];
	assign signextend[31:17] = immediate[16] ? 15'b111111111111111 : 15'b0;
			
	wire isSW_x;
	assign isSW_x = ~opx[4]&~opx[3]&opx[2]&opx[1]&opx[0];
	wire isLW_x;
	assign isLW_x = ~opx[4]&opx[3]&~opx[2]&~opx[1]&~opx[0];
	wire isALUOp_x;
	assign isALUOp_x = ~opx[4]&~opx[3]&~opx[2]&~opx[1]&~opx[0];
	wire isAddi_x;
	assign isAddi_x = ~opx[4]&~opx[3]&opx[2]&~opx[1]&opx[0];
	
	wire isJ_x;
	assign isJ_x = ~opx[4]&~opx[3]&~opx[2]&~opx[1]&opx[0];
	wire isBne_x;
	assign isBne_x = ~opx[4]&~opx[3]&~opx[2]&opx[1]&~opx[0];
	wire isJal_x;
	assign isJal_x = ~opx[4]&~opx[3]&~opx[2]&opx[1]&opx[0];
	wire isJr_x;
	assign isJr_x = ~opx[4]&~opx[3]&opx[2]&~opx[1]&~opx[0];
	wire isBlt_x;
	assign isBlt_x = ~opx[4]&~opx[3]&opx[2]&opx[1]&~opx[0];
	
	wire isR_x;
	assign isR_x = isALUOp_x;
	wire isI_x;
	assign isI_x = isAddi_x || isSW_x || isLW_x;
	
	wire [31:0] alu_input_1;
	wire [31:0] pre_alu_input_2;
	wire [31:0] alu_input_2;
	assign alu_input_2 = isI_x ? signextend : pre_alu_input_2;
	
	wire [4:0] final_aluop = isI_x ? 5'b00000 : aluop;
	
	// ====== Branching in Execute stage
	
	wire bne_alu, blt_alu;
	wire [31:0] pc_add_n;
	
	alu pc_branch_alu (.data_operandA(pc_dx), .data_operandB(signextend), 
		.ctrl_ALUopcode(5'b00000), .ctrl_shiftamt(5'b00000), .data_result(pc_add_n), 
		.isNotEqual(), .isLessThan(), .overflow(), .carry_in(1'b1));
	
	wire isBranch3, isBranch2, isBranch1;
	assign isBranch3 = isJ_x || isJal_x || isJr_x ? 1'b1 : 1'b0;
	assign isBranch2 = isBne_x && bne_alu ? 1'b1 : isBranch3;
	assign isBranch1 = isBlt_x && (!blt_alu && bne_alu) ? 1'b1 : isBranch2;
	assign isBranch = isBranch1;
	
	wire [1:0] pc_branch_select;
	// assuming if pc_branch_select == 2'b00, then this is bne or blt, so don't need to assign.
	assign pc_branch_select[0] = isJ_x || isJal_x;
	assign pc_branch_select[1] = isJr_x;
	
	wire [31:0] T_x_extend;
	assign T_x_extend[26:0] = T_x;
	assign T_x_extend[31:27] = 5'b00000;
	
	// 0: PC + N + 1
	// 1: 32 bit extend of T
	// 2: value of $rd
	// 3: nothing
	mux_4_1 mux_branch (
		.out(branch_value), 
		.in0(pc_add_n), .in1(T_x_extend), .in2(pre_alu_input_2), .in3(32'b0),
		.select(pc_branch_select));
		
	
	// ====== MX Bypassing
	
	/*
	
	Check if 
	(1) register rs/rt/rd in x matches with rd in m
	(2) if operation in x needs to use the updated value in rs/rt/rd
	(3) if operation in m actually updates register rs/rt/rd
	
	*/
	
	wire reg_match_rs_mx, MX1;
	equality5 mx1_eq (.out(reg_match_rs_mx), .a(rd_m), .b(rs_x));
	assign MX1 = (reg_match_rs_mx && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isBne_x || isBlt_x))
		&& (isALUOp_m || isAddi_m);
	
	wire reg_match_rt_mx, reg_match_rd_mx, MX2;
	equality5 mx2a_eq (.out(reg_match_rt_mx), .a(rd_m), .b(rt_x));
	equality5 mx2b_eq (.out(reg_match_rd_mx), .a(rd_m), .b(rd_x));
	assign MX2 = ((reg_match_rt_mx && (isALUOp_x)) || 
		(reg_match_rd_mx && (isSW_x || isBne_x || isJr_x || isBlt_x))) &&
		(isALUOp_m || isAddi_m);
	
	// ====== WX Bypassing
	
	wire reg_match_rs_wx, WX1;
	equality5 wx1_eq (.out(reg_match_rs_wx), .a(rd_w), .b(rs_x));
	assign WX1 = (reg_match_rs_wx && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isBne_x || isBlt_x))
		&& (isALUOp_w || isAddi_w);
	
	wire reg_match_rt_wx, reg_match_rd_wx, WX2;
	equality5 wx2a_eq (.out(reg_match_rt_wx), .a(rd_w), .b(rt_x));
	equality5 wx2b_eq (.out(reg_match_rd_wx), .a(rd_w), .b(rd_x));
	assign WX2 = ((reg_match_rt_wx && (isALUOp_x)) || 
		(reg_match_rd_wx && (isSW_x || isBne_x || isJr_x || isBlt_x))) &&
		(isALUOp_w || isAddi_w);
	
	// ====== Integrating MX and WX bypassing
	
	// selector for mux that determines alu_input_1 and alu_input_2
	wire [1:0] sel1;
	assign sel1[0] = WX1 || MX1;
	assign sel1[1] = MX1;
	
	wire [1:0] sel2;
	assign sel2[0] = WX2 || MX2;
	assign sel2[1] = MX2;
	
	// 00: a_dx/b_dx, 01: WX, 10: MX, 11: MX
	mux_4_1 mux_alu_input_1 (
		.out(alu_input_1), 
		.in0(a_dx), .in1(data_writeReg), .in2(o_xm), .in3(o_xm),
		.select(sel1));
		
	mux_4_1 mux_alu_input_2 (
		.out(pre_alu_input_2), 
		.in0(b_dx), .in1(data_writeReg), .in2(o_xm), .in3(o_xm),
		.select(sel2));
	
	
	alu alu1 (.data_operandA(alu_input_1), .data_operandB(alu_input_2), .ctrl_ALUopcode(final_aluop),
		.ctrl_shiftamt(shamt), .data_result(alu_out), .isNotEqual(bne_alu), 
		.isLessThan(blt_alu), .overflow(overflow_x), .carry_in(1'b0));
	
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
	
	assign isSW_m = ~opm[4]&~opm[3]&opm[2]&opm[1]&opm[0];
	assign isLW_m = ~opm[4]&opm[3]&~opm[2]&~opm[1]&~opm[0];
	assign isALUOp_m = ~opm[4]&~opm[3]&~opm[2]&~opm[1]&~opm[0];
	assign isAddi_m = ~opm[4]&~opm[3]&opm[2]&~opm[1]&opm[0];
	assign isBne_m = ~opm[4]&~opm[3]&~opm[2]&opm[1]&~opm[0];
	assign isJr_m = ~opm[4]&~opm[3]&opm[2]&~opm[1]&~opm[0];
	assign isBlt_m = ~opm[4]&~opm[3]&opm[2]&opm[1]&~opm[0];
	
	assign isR_m = isALUOp_m;
	assign isI_m = isAddi_m || isSW_m || isLW_m;
	
	// ====== WM Bypassing
	
	wire reg_match_wm, WM;
	equality5 wm_eq (.out(reg_match_wm), .a(rd_w), .b(rd_m));
	assign WM = reg_match_wm && (isSW_m && (isALUOp_w || isLW_w || isAddi_w));
	
	assign address_dmem = o_xm[11:0];
   assign data = WM ? data_writeReg : b_xm;
   assign wren = isSW_m;
	
	
	//========================================= Write-back Stage
	
	wire [4:0] opw;
	assign opw = ir_mw[31:27];
	
	// rd_w is defined at the top
	assign rd_w = ir_mw[26:22];
	assign rs_w = ir_mw[21:17];
	assign rt_w = ir_mw[16:12];
	
	assign isSW_w = ~opw[4]&~opw[3]&opw[2]&opw[1]&opw[0];
	assign isLW_w = ~opw[4]&opw[3]&~opw[2]&~opw[1]&~opw[0];
	assign isALUOp_w = ~opw[4]&~opw[3]&~opw[2]&~opw[1]&~opw[0];
	assign isAddi_w = ~opw[4]&~opw[3]&opw[2]&~opw[1]&opw[0];
	assign isBne_w = ~opw[4]&~opw[3]&~opw[2]&opw[1]&~opw[0];
	assign isJr_w = ~opw[4]&~opw[3]&opw[2]&~opw[1]&~opw[0];
	assign isBlt_w = ~opw[4]&~opw[3]&opw[2]&opw[1]&~opw[0];
	
	assign isR_w = isALUOp_w;
	assign isI_w = isAddi_w || isSW_w || isLW_w;
	
	assign ctrl_writeEnable = isALUOp_w || isLW_w || isAddi_w;
	
   assign ctrl_writeReg = rd_w;
   assign data_writeReg = isLW_w ? d_mw : o_mw;
	
	
	 

endmodule 

module equality5 (out, a, b);

	input [4:0] a, b;
	output out;

	assign out = ~(a[4]^b[4] || a[3]^b[3] || a[2]^b[2] || a[1]^b[1] || a[0]^b[0]);

endmodule 