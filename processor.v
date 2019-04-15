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
    data_readRegB,                  // I: Data from port B of regfile
	 
	 snake
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
	 
	 // changed from 226
	 output [359:0] snake;
	 
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
	 
	 wire isStall_pc, isStall_fd, isStall_dx, isStall_xm;
	 
	 wire [31:0] noop;
	 assign noop = 32'b0;
	 
	 wire isSetx_m;
	 wire isSetx_w;
	 wire [31:0] T_m_extend;
	 wire [31:0] T_w_extend;
	 
	 wire isRStatus_xm, isRStatus_mw;
	 wire [31:0] rStatus_xm, rStatus_mw;
	 
	 wire isLoadToALU;
	 
	//========================================= Fetch Stage
	
	wire negclock;
	assign negclock = ~clock;
	 
	wire [31:0] pc;
	wire [31:0] ir_fd, pc_fd, next_pc;
	
	wire [31:0] ir_in_fd;
	assign ir_in_fd = isBranch ? noop : q_imem;
	
	latch_fd latch_fd1 (.ir_in(ir_in_fd), .pc_in(next_pc), .clock(clock), .reset(reset), 
		.enable(~isStall_fd), .ir_out(ir_fd), .pc_out(pc_fd));
		
	wire [31:0] pc_data_in;
	assign pc_data_in = isBranch ? branch_value : next_pc;
	//dflipflop pc_dff (.d(pc_data_in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(pc));
	latch_pc latch_pc1 (.pc_in(pc_data_in), .clock(clock), .reset(reset), .enable(~isStall_pc), 
		.pc_out(pc));
	assign address_imem = pc;
	
	alu alu_next_pc (.data_operandA(pc), .data_operandB(32'd1), .ctrl_ALUopcode(5'b00000),
		.ctrl_shiftamt(5'b00000), .data_result(next_pc), .isNotEqual(), 
		.isLessThan(), .overflow(), .carry_in(1'b0));
		
	//========================================= Decode Stage	
	
	wire [31:0] ir_dx, pc_dx, a_dx, b_dx, a_out_regfile, b_out_regfile, a_in_dx, b_in_dx;
	
	// all1 is a noop
	wire [31:0] all1;
	assign all1 = 32'b11111111111111111111111111111111;
	
	wire [31:0] ir_in_dx;
	assign ir_in_dx = isBranch || isLoadToALU ? all1 : ir_fd;
		
	latch_dx latch_dx1 (.ir_in(ir_in_dx), .pc_in(pc_fd), .a_in(a_in_dx), .b_in(b_in_dx), 
		.clock(clock), .reset(reset), .enable(~isStall_dx), .ir_out(ir_dx), .pc_out(pc_dx), 
		.a_out(a_dx), .b_out(b_dx));
		
	wire [4:0] opd;
	assign opd = ir_fd[31:27];
	
	wire [4:0] rd_d, rs_d, rt_d;
	assign rd_d = ir_fd[26:22];
	assign rs_d = ir_fd[21:17];
	assign rt_d = ir_fd[16:12];
	
	wire isLoadSnake_d;
	assign isLoadSnake_d = ~opd[4]&opd[3]&opd[2]&opd[1]&opd[0];
	
	wire isSW_d;
	assign isSW_d = ~opd[4]&~opd[3]&opd[2]&opd[1]&opd[0];
	wire isLW_d;
	assign isLW_d = ~opd[4]&opd[3]&~opd[2]&~opd[1]&~opd[0];
	wire isALUOp_d;
	assign isALUOp_d = ~opd[4]&~opd[3]&~opd[2]&~opd[1]&~opd[0];
	wire isAddi_d;
	assign isAddi_d = ~opd[4]&~opd[3]&opd[2]&~opd[1]&opd[0];
	
	wire isJ_d;
	assign isJ_d = ~opd[4]&~opd[3]&~opd[2]&~opd[1]&opd[0];
	wire isBne_d;
	assign isBne_d = ~opd[4]&~opd[3]&~opd[2]&opd[1]&~opd[0];
	wire isJal_d;
	assign isJal_d = ~opd[4]&~opd[3]&~opd[2]&opd[1]&opd[0];
	wire isJr_d;
	assign isJr_d = ~opd[4]&~opd[3]&opd[2]&~opd[1]&~opd[0];
	wire isBlt_d;
	assign isBlt_d = ~opd[4]&~opd[3]&opd[2]&opd[1]&~opd[0];
	
	wire isBex_d;
	assign isBex_d = opd[4]&~opd[3]&opd[2]&opd[1]&~opd[0];
	wire isSetx_d;
	assign isSetx_d = opd[4]&~opd[3]&opd[2]&~opd[1]&opd[0];
	
	wire isR_d;
	assign isR_d = isALUOp_d;
	wire isI_d;
	assign isI_d = isAddi_d || isSW_d || isLW_d;
	
	assign ctrl_readRegA = isBex_d ? 5'd30 : rs_d;
	wire need_rd_reg;
	assign need_rd_reg = isSW_d || isBne_d || isBlt_d || isJr_d;
	wire need_rt_reg;
	assign need_rt_reg = isALUOp_d;
	wire need_rs_reg;
	assign need_rs_reg = isALUOp_d || isAddi_d || isSW_d || isLW_d || isBlt_d || isBne_d || isLoadSnake_d;
	
	
	assign ctrl_readRegB = need_rd_reg ? rd_d : rt_d;
	
	// bypassing the updated write register value if write address matches rd
	// similar to writing to register before reading in the same clock cycle
//	assign a_out_regfile = data_readRegA;
//	wire rdNotEqualWriteAddress;
//	assign rdNotEqualWriteAddress = (rd_d[4]^ctrl_writeReg[4] || rd_d[3]^ctrl_writeReg[3] ||
//		rd_d[2]^ctrl_writeReg[2] || rd_d[1]^ctrl_writeReg[1] || rd_d[0]^ctrl_writeReg[0]);
//	assign b_out_regfile = (~rdNotEqualWriteAddress && need_rd_reg)
//		? data_writeReg : data_readRegB;	

	wire match_write_rs, match_write_rt, match_write_rd;
	equality5 write_a_eq (.out(match_write_rs), .a(ctrl_writeReg), .b(rs_d));
	equality5 write_b_eq1 (.out(match_write_rt), .a(ctrl_writeReg), .b(rt_d));
	equality5 write_b_eq2 (.out(match_write_rd), .a(ctrl_writeReg), .b(rd_d));
	
	assign a_out_regfile = (match_write_rs && need_rs_reg && ctrl_writeEnable) 
		? data_writeReg : data_readRegA;
	assign b_out_regfile = (match_write_rt && need_rt_reg && ctrl_writeEnable) ||
		(match_write_rd && need_rd_reg && ctrl_writeEnable) ? data_writeReg : data_readRegB;
		
	assign a_in_dx = isBranch || isLoadToALU ? noop : a_out_regfile;
	assign b_in_dx = isBranch || isBex_d || isLoadToALU ? noop : b_out_regfile;
	
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
	
	wire isLoadSnake_x;
	assign isLoadSnake_x = ~opx[4]&opx[3]&opx[2]&opx[1]&opx[0];
			
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
	
	wire isBex_x;
	assign isBex_x = opx[4]&~opx[3]&opx[2]&opx[1]&~opx[0];
	wire isSetx_x;
	assign isSetx_x = opx[4]&~opx[3]&opx[2]&~opx[1]&opx[0];
	
	wire isR_x;
	assign isR_x = isALUOp_x;
	wire isI_x;
	assign isI_x = isAddi_x || isSW_x || isLW_x || isLoadSnake_x;
	
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
		.isNotEqual(), .isLessThan(), .overflow(), .carry_in(1'b0));
	
	wire isBranch4, isBranch3, isBranch2, isBranch1;
	assign isBranch4 = isBex_x && bne_alu;
	assign isBranch3 = isJ_x || isJal_x || isJr_x;
	assign isBranch2 = isBne_x && bne_alu;
	assign isBranch1 = isBlt_x && (!blt_alu && bne_alu);
	
	assign isBranch = isBranch1 || isBranch2 || isBranch3 || isBranch4;
	
	wire [1:0] pc_branch_select;
	// assuming if pc_branch_select == 2'b00, then this is bne or blt, so don't need to assign.
	// 00: bne || blt, 01: J || Jal || Bex, 10: Jr, 11: N/A
	assign pc_branch_select[0] = isJ_x || isJal_x || isBex_x;
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
		
		
	// ====== MX/WX Bypassing for RStatus (setx)
		
	wire is_rs_30, is_rt_30, is_rd_30;
	equality5 reg30_eq1 (.out(is_rs_30), .a(5'd30), .b(rs_x));
	equality5 reg30_eq2 (.out(is_rt_30), .a(5'd30), .b(rt_x));
	equality5 reg30_eq3 (.out(is_rd_30), .a(5'd30), .b(rd_x));
	
	wire MX_30_rs, MX_30_rt, MX_30_rd;
	wire WX_30_rs, WX_30_rt, WX_30_rd;
	assign MX_30_rs = (isSetx_m || isRStatus_xm) && 
		((is_rs_30 && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isBne_x || isBlt_x)) || isBex_x);
	assign MX_30_rt = (isSetx_m || isRStatus_xm) &&
		(is_rt_30 && (isALUOp_x || isAddi_x));
	assign MX_30_rd = (isSetx_m || isRStatus_xm) &&
		(is_rd_30 && (isSW_x || isBne_x || isBlt_x));
	assign WX_30_rs = (isSetx_w || isRStatus_mw) && 
		((is_rs_30 && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isBne_x || isBlt_x)) || isBex_x);
	assign WX_30_rt = (isSetx_w || isRStatus_mw) &&
		(is_rt_30 && (isALUOp_x || isAddi_x));
	assign WX_30_rd = (isSetx_w || isRStatus_mw) &&
		(is_rd_30 && (isSW_x || isBne_x || isBlt_x));
		
	
	// ====== MX Bypassing
	
	/*
	
	Check if 
	(1) register rs/rt/rd in x matches with rd in m
	(2) if operation in x needs to use the updated value in rs/rt/rd
	(3) if operation in m actually updates register rs/rt/rd
	
	*/
	
	
	wire reg_match_rs_mx, MX1;
	equality5 mx1_eq (.out(reg_match_rs_mx), .a(rd_m), .b(rs_x));
	assign MX1 = (reg_match_rs_mx && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isLoadSnake_x
		|| isBne_x || isBlt_x))
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
	assign WX1 = (reg_match_rs_wx && (isALUOp_x || isAddi_x || isSW_x || isLW_x || isLoadSnake_x
		|| isBne_x || isBlt_x || isBex_x))
		&& (isALUOp_w || isAddi_w || isLW_w);
	
	wire reg_match_rt_wx, reg_match_rd_wx, WX2;
	equality5 wx2a_eq (.out(reg_match_rt_wx), .a(rd_w), .b(rt_x));
	equality5 wx2b_eq (.out(reg_match_rd_wx), .a(rd_w), .b(rd_x));
	assign WX2 = ((reg_match_rt_wx && (isALUOp_x)) || 
		(reg_match_rd_wx && (isSW_x || isBne_x || isJr_x || isBlt_x))) &&
		(isALUOp_w || isAddi_w || isLW_w);
	
	// ====== Integrating MX and WX bypassing
	
	// selector for mux that determines alu_input_1 and alu_input_2
	wire [1:0] sel1;
	assign sel1[0] = WX1 || MX1;
	assign sel1[1] = MX1;
	
	wire [1:0] sel2;
	assign sel2[0] = WX2 || MX2;
	assign sel2[1] = MX2;
	
	wire [31:0] alu_1, alu_2;
	
	// 00: a_dx/b_dx, 01: WX, 10: MX, 11: MX
	mux_4_1 mux_alu_1 (
		.out(alu_1), 
		.in0(a_dx), .in1(data_writeReg), .in2(o_xm), .in3(o_xm),
		.select(sel1));
		
	mux_4_1 mux_alu_2 (
		.out(alu_2), 
		.in0(b_dx), .in1(data_writeReg), .in2(o_xm), .in3(o_xm),
		.select(sel2));
		
	wire [1:0] sel_alu_input1;
	assign sel_alu_input1[0] = MX_30_rs;
	assign sel_alu_input1[1] = WX_30_rs && (~MX_30_rs && ~MX1);
	
	wire [1:0] sel_alu_input2;
	assign sel_alu_input2[0] = MX_30_rt || MX_30_rd;
	assign sel_alu_input2[1] = (WX_30_rt || WX_30_rd) && (~(MX_30_rt || MX_30_rd) && ~MX2);

	mux_4_1 mux_alu_input_1 (
		.out(alu_input_1), 
		.in0(alu_1), .in1(isRStatus_xm ? rStatus_xm : T_m_extend), 
		.in2(isRStatus_mw ? rStatus_mw : T_w_extend), .in3(noop),
		.select(sel_alu_input1));
	
	mux_4_1 mux_alu_input_2 (
		.out(pre_alu_input_2), 
		.in0(alu_2), .in1(isRStatus_xm ? rStatus_xm : T_m_extend), 
		.in2(isRStatus_mw ? rStatus_mw : T_w_extend), .in3(noop),
		.select(sel_alu_input2));
	
	
	alu alu1 (.data_operandA(alu_input_1), .data_operandB(alu_input_2), .ctrl_ALUopcode(final_aluop),
		.ctrl_shiftamt(shamt), .data_result(alu_out), .isNotEqual(bne_alu), 
		.isLessThan(blt_alu), .overflow(overflow_x), .carry_in(1'b0));
		
	
	// ====== Multdiv
	
	wire isAdd_x;
	assign isAdd_x = (~aluop[4]&~aluop[3]&~aluop[2]&~aluop[1]&~aluop[0]) && isALUOp_x;
	wire isSub_x;
	assign isSub_x = (~aluop[4]&~aluop[3]&~aluop[2]&~aluop[1]&aluop[0]) && isALUOp_x;
	wire isMul_x;
	assign isMul_x = (~aluop[4]&~aluop[3]&aluop[2]&aluop[1]&~aluop[0]) && isALUOp_x;
	assign isDiv_x = (~aluop[4]&~aluop[3]&aluop[2]&aluop[1]&aluop[0]) && isALUOp_x;
	
	wire [31:0] multdiv_result;
	wire data_exception, data_resultRDY;
	
	// Check if multdiv is still ongoing
	wire startMultDiv, ready_reg;
	dflipflop dff_startMultDiv (.d(isMul_x || isDiv_x), 
		.clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(ready_reg));
	assign startMultDiv = (isMul_x || isDiv_x) && ~ready_reg;
	
	wire isStillMultDiv, pre_isStillMultDiv;
	assign isStillMultDiv = startMultDiv || pre_isStillMultDiv;
	dflipflop dff_preMultDiv (.d(startMultDiv || pre_isStillMultDiv), 
		.clk(clock), .clrn(~data_resultRDY), .prn(1'b1), .ena(1'b1), .q(pre_isStillMultDiv));
	
	assign isStall_pc = (isStillMultDiv && ~data_resultRDY) || isLoadToALU;
	assign isStall_fd = (isStillMultDiv && ~data_resultRDY) || isLoadToALU;
	assign isStall_dx = (isStillMultDiv && ~data_resultRDY); //|| isLoadToALU;
	assign isStall_xm = 1'b0; //isLoadToALU;
	
	multdiv md1 (.data_operandA(alu_input_1), .data_operandB(alu_input_2), 
		.ctrl_MULT(isMul_x && startMultDiv), 
		.ctrl_DIV(isDiv_x && startMultDiv && ~isLoadToALU), .clock(clock), 
		.data_result(multdiv_result), .data_exception(data_exception), .data_resultRDY(data_resultRDY));
		
	
	// ====== R Status
	
	wire isRStatus_x;
	wire [31:0] rStatus_x;
	assign isRStatus_x = (isALUOp_x && (
		(isAdd_x && overflow_x) ||
		(isSub_x && overflow_x) ||
		(isMul_x && data_exception) ||
		(isDiv_x && data_exception))
		) || (isAddi_x && overflow_x);
	assign rStatus_x[0] = isAdd_x || isSub_x || isDiv_x;
	assign rStatus_x[1] = isAddi_x || isSub_x;
	assign rStatus_x[2] = isMul_x || isDiv_x;
	assign rStatus_x[31:3] = 29'b0;
	
	
	wire [31:0] o_in_x, b_in_x;
	
	// 00: normal alu output, 01: noop(branches or still in multdiv computation)
	// 10: finished multdiv, dataresultRDY is 1, 11: unused
	wire [1:0] o_in_x_sel;
	assign o_in_x_sel[0] = isBranch || isStillMultDiv;
	assign o_in_x_sel[1] = data_resultRDY;
	
	mux_4_1 o_in_x_mux (
		.out(o_in_x), 
		.in0(alu_out), .in1(noop), .in2(multdiv_result), .in3(multdiv_result),
		.select(o_in_x_sel));
		
	mux_4_1 b_in_x_mux (
		.out(b_in_x), 
		.in0(pre_alu_input_2), .in1(noop), .in2(pre_alu_input_2), .in3(pre_alu_input_2),
		.select(o_in_x_sel));
		
		
	wire [31:0] ir_in_xm;
	assign ir_in_xm = isBranch || (isStillMultDiv && ~data_resultRDY) ? noop : ir_dx;
	
	wire [31:0] ir_in_xm_jal;
	assign ir_in_xm_jal[31:27] = 5'b00000;
	assign ir_in_xm_jal[26:22] = 5'd31;
	assign ir_in_xm_jal[21:0] = 22'b0;
	
	wire [31:0] ir_in_xm_final;
	assign ir_in_xm_final = (isJal_x ? ir_in_xm_jal : ir_in_xm);
	
	wire [31:0] o_in_x_final;
	assign o_in_x_final = isJal_x ? pc_dx : o_in_x;
	
	latch_xm latch_xm1 (.ir_in(ir_in_xm_final), .o_in(o_in_x_final), .b_in(b_in_x), 
		.isRStatus_in(isRStatus_x), .rStatus_in(rStatus_x), .clock(clock), .reset(reset), 
		.enable(1'b1), .ir_out(ir_xm), .o_out(o_xm), .b_out(b_xm), .isRStatus_out(isRStatus_xm), 
		.rStatus_out(rStatus_xm));
	
	//========================================= Memory Stage
	
	wire [31:0] ir_mw, o_mw, d_mw;
	
	latch_mw latch_mw1 (.ir_in(ir_xm), .o_in(o_xm), .d_in(q_dmem), .isRStatus_in(isRStatus_xm), 
		.rStatus_in(rStatus_xm), .clock(clock), .reset(reset), .ir_out(ir_mw), .o_out(o_mw), 
		.d_out(d_mw), .isRStatus_out(isRStatus_mw), .rStatus_out(rStatus_mw));
		
	wire [4:0] opm;
	assign opm = ir_xm[31:27];
	
	// rd_m is defined at the top
	assign rd_m = ir_xm[26:22];
	assign rs_m = ir_xm[21:17];
	assign rt_m = ir_xm[16:12];
	
	wire isLoadSnake_m;
	assign isLoadSnake_m = ~opm[4]&opm[3]&opm[2]&opm[1]&opm[0];
	
	assign isSW_m = ~opm[4]&~opm[3]&opm[2]&opm[1]&opm[0];
	assign isLW_m = ~opm[4]&opm[3]&~opm[2]&~opm[1]&~opm[0];
	assign isALUOp_m = ~opm[4]&~opm[3]&~opm[2]&~opm[1]&~opm[0];
	assign isAddi_m = ~opm[4]&~opm[3]&opm[2]&~opm[1]&opm[0];
	assign isBne_m = ~opm[4]&~opm[3]&~opm[2]&opm[1]&~opm[0];
	assign isJr_m = ~opm[4]&~opm[3]&opm[2]&~opm[1]&~opm[0];
	assign isBlt_m = ~opm[4]&~opm[3]&opm[2]&opm[1]&~opm[0];
	
	wire isBex_m;
	assign isBex_m = opm[4]&~opm[3]&opm[2]&opm[1]&~opm[0];
	assign isSetx_m = opm[4]&~opm[3]&opm[2]&~opm[1]&opm[0];
	
	assign isR_m = isALUOp_m;
	assign isI_m = isAddi_m || isSW_m || isLW_m || isLoadSnake_m;
	
	wire [26:0] T_m;
	assign T_m = ir_xm[26:0];
	assign T_m_extend[26:0] = T_m;
	assign T_m_extend[31:27] = 5'b00000;
	
	// ====== WM Bypassing
	
	wire reg_match_wm, WM;
	equality5 wm_eq (.out(reg_match_wm), .a(rd_w), .b(rd_m));
	assign WM = reg_match_wm && (isSW_m && (isALUOp_w || isLW_w || isAddi_w));
	

	
	assign address_dmem = o_xm[11:0];
   	assign data = WM ? data_writeReg : b_xm;
   	assign wren = isSW_m;
	
	
	// Load Snake register
	
	wire isLoadSnake_w;
	
	snake_register sr1 (
		.value_in(d_mw), 
		.index(o_mw-2100), 
		.clock(clock), 
		.reset(reset),
		.enable(isLoadSnake_w),
		.value_out(snake)
	);
	
	//========================================= Write-back Stage
	
	wire [4:0] opw;
	assign opw = ir_mw[31:27];
	
	// rd_w is defined at the top
	assign rd_w = ir_mw[26:22];
	assign rs_w = ir_mw[21:17];
	assign rt_w = ir_mw[16:12];
	
	assign isLoadSnake_w = ~opw[4]&opw[3]&opw[2]&opw[1]&opw[0];
	
	assign isSW_w = ~opw[4]&~opw[3]&opw[2]&opw[1]&opw[0];
	assign isLW_w = ~opw[4]&opw[3]&~opw[2]&~opw[1]&~opw[0];
	assign isALUOp_w = ~opw[4]&~opw[3]&~opw[2]&~opw[1]&~opw[0];
	assign isAddi_w = ~opw[4]&~opw[3]&opw[2]&~opw[1]&opw[0];
	assign isBne_w = ~opw[4]&~opw[3]&~opw[2]&opw[1]&~opw[0];
	assign isJr_w = ~opw[4]&~opw[3]&opw[2]&~opw[1]&~opw[0];
	assign isBlt_w = ~opw[4]&~opw[3]&opw[2]&opw[1]&~opw[0];
	
	wire isBex_w;
	assign isBex_w = opw[4]&~opw[3]&opw[2]&opw[1]&~opw[0];
	assign isSetx_w = opw[4]&~opw[3]&opw[2]&~opw[1]&opw[0];
	
	assign isR_w = isALUOp_w;
	assign isI_w = isAddi_w || isSW_w || isLW_w;
	
	wire [26:0] T_w;
	assign T_w = ir_mw[26:0];
	assign T_w_extend[26:0] = T_w;
	assign T_w_extend[31:27] = 5'b00000;
	
	
	
	
	assign ctrl_writeEnable = isALUOp_w || isLW_w || isAddi_w || isRStatus_mw || isSetx_w;
	
	// data write sel
	// 00: o_mw, 01: rStatus_mw, 10: T extend, 11: d_mw
	wire [1:0] data_write_sel;
	assign data_write_sel[0] = isRStatus_mw || isLW_w;
	assign data_write_sel[1] = isSetx_w || isLW_w;
	
   assign ctrl_writeReg = (isRStatus_mw || isSetx_w) ? 5'd30 : rd_w;
	mux_4_1 mux_data_write (
		.out(data_writeReg), 
		.in0(o_mw), .in1(rStatus_mw), .in2(T_w_extend), .in3(d_mw),
		.select(data_write_sel));
		
	wire rtd_rdx, rsd_rdx;
	equality5 loadStall1 (.out(rtd_rdx), .a(rt_d), .b(rd_x));
	equality5 loadStall2 (.out(rsd_rdx), .a(rs_d), .b(rd_x));
	//assign isLoadToALU = isLW_m && ((rtx_rdm) || (rsx_rdm && ~isSW_x)) && isALUOp_x;
	assign isLoadToALU = isLW_x && ( 
		(rtd_rdx && (isALUOp_d))  
		|| 
		(rsd_rdx && (isSW_d || isALUOp_d || isAddi_d || isLW_d || isBne_d || isBlt_d))
	);
	 

endmodule 

module equality5 (out, a, b);

	input [4:0] a, b;
	output out;

	assign out = ~(a[4]^b[4] || a[3]^b[3] || a[2]^b[2] || a[1]^b[1] || a[0]^b[0]);

endmodule 