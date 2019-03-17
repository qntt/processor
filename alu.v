module alu(data_operandA, data_operandB, ctrl_ALUopcode,
				ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow, carry_in);
				
	input [31:0] data_operandA, data_operandB;
	input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
	input carry_in;
	
	output [31:0] data_result;
	output isNotEqual, isLessThan, overflow;
	
	wire [31:0] sum_result, difference_result, sll_result, sra_result, and_result, or_result;
	wire eq_result, gt_result;
	wire sum_overflow, difference_overflow;
	
	wire [31:0] data_operandB_ALU;
	wire carry_in_ALU;
	
	wire [31:0] flippedB;
	//two_complement tc1 (.out(flippedB), .in(data_operandB));
	//mux_2_1 mux21_flip_B (.out(data_operandB_ALU), .in1(data_operandB), .in2(flippedB), 
	//							.enable(ctrl_ALUopcode[0]));
								
	//mux_21_all_1bit mux21_carry_in (.out(carry_in_ALU), .in1(1'b0), .in2(1'b1), 
	//							.enable(ctrl_ALUopcode[0]));
	
	//CLA_32 cla1 (.out(sum_result), .in1(data_operandA), .in2(data_operandB_ALU),
	//				.cout(sum_carry_out), .cin(carry_in_ALU), .overflow(sum_overflow));
	
	CLA_32 sum1 (.out(sum_result), .in1(data_operandA), .in2(data_operandB),
					.cout(sum_carry_out), .cin(carry_in), .overflow(sum_overflow));
	
	subtract sub1 (.out(difference_result), .in1(data_operandA), .in2(data_operandB), 
						.overflow(difference_overflow));
						
	sll_barrel_32 sll1 (.out(sll_result), .in(data_operandA), .shamt(ctrl_shiftamt), .enable(1'b1));
	
	sra_barrel_32 sra1 (.out(sra_result), .in(data_operandA), .shamt(ctrl_shiftamt), .enable(1'b1));
	
	and_32 and32 (.out(and_result), .in1(data_operandA), .in2(data_operandB));
	
	or_32 or32 (.out(or_result), .in1(data_operandA), .in2(data_operandB));
	
	// isNotEqual: true if any bit in the difference is 1
	wire not_equal_8, not_equal_16, not_equal_24, not_equal_32;
	or o1 (not_equal_8, difference_result[7], difference_result[6], difference_result[5], difference_result[4],
								difference_result[3], difference_result[2], difference_result[1], difference_result[0]);
	or o2 (not_equal_16, difference_result[15], difference_result[14], difference_result[13], difference_result[12],
								difference_result[11], difference_result[10], difference_result[9], difference_result[8]);
	or o3 (not_equal_24, difference_result[23], difference_result[22], difference_result[21], difference_result[20],
								difference_result[19], difference_result[18], difference_result[17], difference_result[16]);
	or o4 (not_equal_32, difference_result[31], difference_result[30], difference_result[29], difference_result[28],
								difference_result[27], difference_result[26], difference_result[25], difference_result[24]);
	or o5 (isNotEqual, not_equal_8, not_equal_16, not_equal_24, not_equal_32);
	
	// isLessThan: true if difference is negative
	// if overflow, select the negated version of difference
	mux_21_all_1bit mux_less_than (.out(isLessThan), .in1(difference_result[31]), .in2(~difference_result[31]), .enable(overflow));
	
	// check if overflow
	
	wire subtract_diff_signs, subtract_result_diff_sign, subtract_overflow;
	xor x1 (subtract_diff_signs, data_operandA[31], data_operandB[31]);
	xor x2 (subtract_result_diff_sign, data_operandA[31], difference_result[31]);
	and a2 (subtract_overflow, subtract_diff_signs, subtract_result_diff_sign);
	
	
	mux_21_all_1bit mux21_for_overflow (.out(overflow), .in1(sum_overflow), .in2(subtract_overflow), 
										.enable(ctrl_ALUopcode[0]));
	
	
	mux_8_1 mux8 (.out(data_result), .in0(sum_result), .in1(difference_result), .in2(and_result), 
						.in3(or_result), .in4(sll_result), .in5(sra_result), .in6(32'b0), .in7(32'b0),
						.select(ctrl_ALUopcode[2:0]));
	
	
endmodule

module CLA_32 (out, in1, in2, cout, cin, overflow);

	input [31:0] in1, in2;
	input cin;
	
	output [31:0] out;
	output cout, overflow;
	
	wire c1, c2, c3, c4;
	wire c5a, c5b, c6a, c6b, c7a, c7b;
	wire cout_a, cout_b;
	
	wire [31:0] out_a, out_b;
	
	/*
	CLA_4 cla1 (.out(out[3:0]), .in1(in1[3:0]), .in2(in2[3:0]), .cout(c1), .cin(cin));
	CLA_4 cla2 (.out(out[7:4]), .in1(in1[7:4]), .in2(in2[7:4]), .cout(c2), .cin(c1));
	CLA_4 cla3 (.out(out[11:8]), .in1(in1[11:8]), .in2(in2[11:8]), .cout(c3), .cin(c2));
	CLA_4 cla4 (.out(out[15:12]), .in1(in1[15:12]), .in2(in2[15:12]), .cout(c4), .cin(c3));
	
	CLA_4 cla5a (.out(out_a[19:16]), .in1(in1[19:16]), .in2(in2[19:16]), .cout(c5a), .cin(1'b0));
	CLA_4 cla6a (.out(out_a[23:20]), .in1(in1[23:20]), .in2(in2[23:20]), .cout(c6a), .cin(c5a));
	CLA_4 cla7a (.out(out_a[27:24]), .in1(in1[27:24]), .in2(in2[27:24]), .cout(c7a), .cin(c6a));
	CLA_4 cla8a (.out(out_a[31:28]), .in1(in1[31:28]), .in2(in2[31:28]), .cout(cout_a), .cin(c7a));
	
	CLA_4 cla5b (.out(out_b[19:16]), .in1(in1[19:16]), .in2(in2[19:16]), .cout(c5b), .cin(1'b1));
	CLA_4 cla6b (.out(out_b[23:20]), .in1(in1[23:20]), .in2(in2[23:20]), .cout(c6b), .cin(c5b));
	CLA_4 cla7b (.out(out_b[27:24]), .in1(in1[27:24]), .in2(in2[27:24]), .cout(c7b), .cin(c6b));
	CLA_4 cla8b (.out(out_b[31:28]), .in1(in1[31:28]), .in2(in2[31:28]), .cout(cout_b), .cin(c7b));
	*/
	
	CLA_8 cla1 (.out(out[7:0]), .in1(in1[7:0]), .in2(in2[7:0]), .cout(c1), .cin(cin));
	CLA_8 cla2 (.out(out[15:8]), .in1(in1[15:8]), .in2(in2[15:8]), .cout(c2), .cin(c1));
	CLA_8 cla3 (.out(out[23:16]), .in1(in1[23:16]), .in2(in2[23:16]), .cout(c3), .cin(c2));
	CLA_8 cla4 (.out(out[31:24]), .in1(in1[31:24]), .in2(in2[31:24]), .cout(cout), .cin(c3));
	
	//mux_2_1_31_16 carry_select_adder (.out(out[31:16]), .in1(out_a[31:16]), .in2(out_b[31:16]), .enable(c4));
	//mux_21_all_1bit carry_select_carry_out (.out(cout), .in1(cout_a), .in2(cout_b), .enable(c4));
	
	
	// check if overflow
	
	wire in1DiffSign, in2DiffSign;
	xor x2 (in1DiffSign, in1[31], out[31]);
	xor x3 (in2DiffSign, in2[31], out[31]);
	
	and a1 (overflow, in1DiffSign, in2DiffSign);

endmodule 

module mux_2_1_31_16 (out, in1, in2, enable);

	input [15:0] in1, in2;
	input enable;
	
	output [15:0] out;
	
	assign out = enable ? in2 : in1;

endmodule 

module CLA_8 (out, in1, in2, cout, cin);

	input [7:0] in1, in2;
	input cin;
	wire c0;
	assign c0 = cin;
	
	output [7:0] out;
	output cout;
	
	wire c1, c2, c3, c4, c5, c6, c7;
	
	wire p0, p1, p2, p3, p4, p5, p6, p7;
	wire g0, g1, g2, g3, g4, g5, g6, g7;
	
	and g0_and (g0, in1[0], in2[0]);
	and g1_and (g1, in1[1], in2[1]);
	and g2_and (g2, in1[2], in2[2]);
	and g3_and (g3, in1[3], in2[3]);
	and g4_and (g4, in1[4], in2[4]);
	and g5_and (g5, in1[5], in2[5]);
	and g6_and (g6, in1[6], in2[6]);
	and g7_and (g7, in1[7], in2[7]);
	
	xor p0_or (p0, in1[0], in2[0]);
	xor p1_or (p1, in1[1], in2[1]);
	xor p2_or (p2, in1[2], in2[2]);
	xor p3_or (p3, in1[3], in2[3]);
	xor p4_or (p4, in1[4], in2[4]);
	xor p5_or (p5, in1[5], in2[5]);
	xor p6_or (p6, in1[6], in2[6]);
	xor p7_or (p7, in1[7], in2[7]);
	
	
	// c1 = g0 + p0c0
	wire p0c0;
	and a1 (p0c0, p0, c0);
	or o1 (c1, g0, p0c0);
	
	// c2 = g1 + g0p1 + c0p0p1;
	wire g0p1, c0p0p1;
	and a2 (g0p1, g0, p1);
	and a3 (c0p0p1, c0, p0, p1);
	or o2 (c2, g1, g0p1, c0p0p1);
	
	// c3 = g2 + g1p2 + g0p1p2 + c0p0p1p2
	wire g1p2, g0p1p2, c0p0p1p2;
	and a4 (g1p2, g1, p2);
	and a5 (g0p1p2, g0, p1, p2);
	and a6 (c0p0p1p2, c0, p0, p1, p2);
	or o3 (c3, g2, g1p2, g0p1p2, c0p0p1p2);
	
	// c4 = g3 + g2p3 + g1p2p3 + g0p1p2p3 + c0p0p1p2p3;
	wire g2p3, g1p2p3, g0p1p2p3, c0p0p1p2p3;
	and a7 (g2p3, g2, p3);
	and a8 (g1p2p3, g1, p2, p3);
	and a9 (g0p1p2p3, g0, p1, p2, p3);
	and a10 (c0p0p1p2p3, c0, p0, p1, p2, p3);
	or o4 (c4, g3, g2p3, g1p2p3, g0p1p2p3, c0p0p1p2p3);
	
	// c5
	wire g3p4, g2p3p4, g1p2p3p4, g0p1p2p3p4, c0p0p1p2p3p4;
	and a11 (g3p4, g3, p4);
	and a12 (g2p3p4, g2, p3, p4);
	and a13 (g1p2p3p4, g1, p2, p3, p4);
	and a14 (g0p1p2p3p4, g0, p1, p2, p3, p4);
	and a15 (c0p0p1p2p3p4, c0, p0, p1, p2, p3, p4);
	or o5 (c5, g4, g3p4, g2p3p4, g1p2p3p4, g0p1p2p3p4, c0p0p1p2p3p4);
	
	// c6
	wire g4p5, g3p4p5, g2p3p4p5, g1p2p3p4p5, g0p1p2p3p4p5, c0p0p1p2p3p4p5;
	and a16 (g4p5, g4, p5);
	and a17 (g3p4p5, g3, p4, p5);
	and a18 (g2p3p4p5, g2, p3, p4, p5);
	and a19 (g1p2p3p4p5, g1, p2, p3, p4, p5);
	and a20 (g0p1p2p3p4p5, g0, p1, p2, p3, p4, p5);
	and a21 (c0p0p1p2p3p4p5, c0, p0, p1, p2, p3, p4, p5);
	or o6 (c6, g5, g4p5, g3p4p5, g2p3p4p5, g1p2p3p4p5, g0p1p2p3p4p5, c0p0p1p2p3p4p5);
	
	// c7
	wire g5p6, g4p5p6, g3p4p5p6, g2p3p4p5p6, g1p2p3p4p5p6, g0p1p2p3p4p5p6, c0p0p1p2p3p4p5p6;
	and a22 (g5p6, g5, p6);
	and a23 (g4p5p6, g4, p5, p6);
	and a24 (g3p4p5p6, g3, p4, p5, p6);
	and a25 (g2p3p4p5p6, g2, p3, p4, p5, p6);
	and a26 (g1p2p3p4p5p6, g1, p2, p3, p4, p5, p6);
	and a27 (g0p1p2p3p4p5p6, g0, p1, p2, p3, p4, p5, p6);
	and a28 (c0p0p1p2p3p4p5p6, c0, p0, p1, p2, p3, p4, p5, p6);
	or o7 (c7, g6, g5p6, g4p5p6, g3p4p5p6, g2p3p4p5p6, g1p2p3p4p5p6, g0p1p2p3p4p5p6, c0p0p1p2p3p4p5p6);
	
	
	// c8
	wire g6p7, g5p6p7, g4p5p6p7, g3p4p5p6p7, g2p3p4p5p6p7, g1p2p3p4p5p6p7, g0p1p2p3p4p5p6p7, c0p0p1p2p3p4p5p6p7;
	and a29 (g6p7, g6, p7);
	and a30 (g5p6p7, g5, p6, p7);
	and a31 (g4p5p6p7, g4, p5, p6, p7);
	and a32 (g3p4p5p6p7, g3, p4, p5, p6, p7);
	and a33 (g2p3p4p5p6p7, g2, p3, p4, p5, p6, p7);
	and a34 (g1p2p3p4p5p6p7, g1, p2, p3, p4, p5, p6, p7);
	and a35 (g0p1p2p3p4p5p6p7, g0, p1, p2, p3, p4, p5, p6, p7);
	and a36 (c0p0p1p2p3p4p5p6p7, c0, p0, p1, p2, p3, p4, p5, p6, p7);
	or o8 (cout, g7, g6p7, g5p6p7, g4p5p6p7, g3p4p5p6p7, g2p3p4p5p6p7, g1p2p3p4p5p6p7, g0p1p2p3p4p5p6p7, c0p0p1p2p3p4p5p6p7);
	
	
	// calculate the output
	xor x1 (out[0], p0, c0);
	xor x2 (out[1], p1, c1);
	xor x3 (out[2], p2, c2);
	xor x4 (out[3], p3, c3);
	xor x5 (out[4], p4, c4);
	xor x6 (out[5], p5, c5);
	xor x7 (out[6], p6, c6);
	xor x8 (out[7], p7, c7);

endmodule 

module and_32 (out, in1, in2);

	input [31:0] in1, in2;
	
	output [31:0] out;
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		and a1 (out[i], in1[i], in2[i]);
	end
	endgenerate

endmodule 

module or_32 (out, in1, in2);

	input [31:0] in1, in2;
	
	output [31:0] out;
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		or o1 (out[i], in1[i], in2[i]);
	end
	endgenerate

endmodule 

module sll_barrel_32 (out, in, shamt, enable);

	input [31:0] in;
	input [4:0] shamt;
	input enable;
	
	output [31:0] out;
	
	wire [31:0] out_16, out_mux_16, out_8, out_mux_8, out_4, out_mux_4, out_2, out_mux_2, out_1;
	
	sll_16 s1 (.out(out_16), .in(in));
	mux_21 m1 (.out(out_mux_16), .a(in), .b(out_16), .ctrl(shamt[4]));
	
	sll_8 s2 (.out(out_8), .in(out_mux_16));
	mux_21 m2 (.out(out_mux_8), .a(out_mux_16), .b(out_8), .ctrl(shamt[3]));
	
	sll_4 s3 (.out(out_4), .in(out_mux_8));
	mux_21 m3 (.out(out_mux_4), .a(out_mux_8), .b(out_4), .ctrl(shamt[2]));
	
	sll_2 s4 (.out(out_2), .in(out_mux_4));
	mux_21 m4 (.out(out_mux_2), .a(out_mux_4), .b(out_2), .ctrl(shamt[1]));
	
	sll_1 s5 (.out(out_1), .in(out_mux_2));
	mux_21 m5 (.out(out), .a(out_mux_2), .b(out_1), .ctrl(shamt[0]));
	
endmodule 

module sll_16 (out, in);

	input [31:0] in;
	output [31:0] out;
	
	assign out[31:16] = in[15:0];
	assign out[15:0] = 16'b0;


endmodule 

module sll_8 (out, in);

	input [31:0] in;
	output [31:0] out;
	
	assign out[31:8] = in[23:0];
	assign out[7:0] = 8'b0;


endmodule 

module sll_4 (out, in);

	input [31:0] in;
	output [31:0] out;
	
	assign out[31:4] = in[27:0];
	assign out[3:0] = 4'b0;


endmodule 

module sll_2 (out, in);

	input [31:0] in;
	output [31:0] out;
	
	assign out[31:2] = in[29:0];
	assign out[1:0] = 2'b0;


endmodule 

module sll_1 (out, in);

	input [31:0] in;
	output [31:0] out;
	
	assign out[31:1] = in[30:0];
	assign out[0] = 1'b0;


endmodule 

module sra_barrel_32 (out, in, shamt, enable);

	input [31:0] in;
	input [4:0] shamt;
	input enable;
	
	output [31:0] out;
	
	wire [31:0] out_16, out_mux_16, out_8, out_mux_8, out_4, out_mux_4, out_2, out_mux_2, out_1;
	
	sra_16 s1 (.out(out_16), .in(in));
	mux_21 m1 (.out(out_mux_16), .a(in), .b(out_16), .ctrl(shamt[4]));
	
	sra_8 s2 (.out(out_8), .in(out_mux_16));
	mux_21 m2 (.out(out_mux_8), .a(out_mux_16), .b(out_8), .ctrl(shamt[3]));
	
	sra_4 s3 (.out(out_4), .in(out_mux_8));
	mux_21 m3 (.out(out_mux_4), .a(out_mux_8), .b(out_4), .ctrl(shamt[2]));
	
	sra_2 s4 (.out(out_2), .in(out_mux_4));
	mux_21 m4 (.out(out_mux_2), .a(out_mux_4), .b(out_2), .ctrl(shamt[1]));
	
	sra_1 s5 (.out(out_1), .in(out_mux_2));
	mux_21 m5 (.out(out), .a(out_mux_2), .b(out_1), .ctrl(shamt[0]));
	
endmodule 

module sra_16 (out, in);

	input[31:0] in;
	output[31:0] out;
	
	assign out[15:0] = in[31:16];
	genvar i;
	generate
		for(i=16; i<32; i=i+1) begin:ok
			or or_temp(out[i], in[31], 1'b0);
	end
	endgenerate


endmodule 

module sra_8 (out, in);

	input[31:0] in;
	output[31:0] out;
	
	assign out[23:0] = in[31:8];
	genvar i;
	generate
		for(i=24; i<32; i=i+1) begin:ok
			or or_temp(out[i], in[31], 1'b0);
	end
	endgenerate


endmodule 

module sra_4 (out, in);

	input[31:0] in;
	output[31:0] out;
	
	assign out[27:0] = in[31:4];
	genvar i;
	generate
		for(i=28; i<32; i=i+1) begin:ok
			or or_temp(out[i], in[31], 1'b0);
	end
	endgenerate


endmodule 

module sra_2 (out, in);

	input[31:0] in;
	output[31:0] out;
	
	assign out[29:0] = in[31:2];
	genvar i;
	generate
		for(i=30; i<32; i=i+1) begin:ok
			or or_temp(out[i], in[31], 1'b0);
	end
	endgenerate


endmodule 

module sra_1 (out, in);

	input[31:0] in;
	output[31:0] out;
	
	assign out[30:0] = in[31:1];
	or or_temp(out[31], in[31], 1'b0);


endmodule 

module subtract (out, in1, in2, overflow);

	input [31:0] in1, in2;
	
	output [31:0] out;
	output overflow;
	
	wire o1, o2;
	
	wire [31:0] flip;
	wire overflow_sum;
	two_complement tc (.out(flip), .in(in2));
	CLA_32 cla (.out(out), .in1(in1), .in2(flip), .cout(o2), .cin(1'b1), .overflow(overflow_sum));
	
	
	// check if overflow
	
	wire in1DiffSign, in2DiffSign;
	xor x2 (in1DiffSign, in1[31], out[31]);
	xor x3 (in2DiffSign, flip[31], out[31]);
	
	and a1 (overflow, in1DiffSign, in2DiffSign);

endmodule 

module two_complement (out, in);

	input [31:0] in;
	
	output [31:0] out;
	
	//wire [31:0] flip;
	
	genvar i;
	generate
		for (i=0; i<32; i=i+1) begin: loop1
			not n1 (out[i], in[i]);
		end
	endgenerate
	
	/*
	wire overflow; // not needed?
	CLA_32 cla1 (.out(out), .in1(flip), 
					.in2(1'b1), .cout(overflow), .cin(1'b0));
	*/

endmodule 

module mux_21 (out, a, b, ctrl);
	
	input [31:0] a, b;
	input ctrl;
	
	output [31:0] out;

	assign out = ctrl ? b : a;

endmodule 

module mux_21_all_1bit (out, in1, in2, enable);

	input in1, in2;
	input enable;
	
	output out;
	
	assign out = enable ? in2 : in1;

endmodule 

module mux_2_1 (out, in1, in2, enable);

	input [31:0] in1, in2;
	input enable;
	
	output [31:0] out;
	
	assign out = enable ? in2 : in1;

endmodule 

module mux_32_1 (out, in0, in1, in2, in3, in4, in5, in6, in7,
							in8, in9, in10, in11, in12, in13, in14, in15,
							in16, in17, in18, in19, in20, in21, in22, in23,
							in24, in25, in26, in27, in28, in29, in30, in31, select);

	input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
	input [31:0] in8, in9, in10, in11, in12, in13, in14, in15;
	input [31:0] in16, in17, in18, in19, in20, in21, in22, in23;
	input [31:0] in24, in25, in26, in27, in28, in29, in30, in31;
	input [4:0] select;
	
	output [31:0] out;
	
	wire [31:0] w1, w2, w3, w4;
	mux_8_1 m1 (w1, in0, in1, in2, in3, in4, in5, in6, in7, select[2:0]);
	mux_8_1 m2 (w2, in8, in9, in10, in11, in12, in13, in14, in15, select[2:0]);
	mux_8_1 m3 (w3, in16, in17, in18, in19, in20, in21, in22, in23, select[2:0]);
	mux_8_1 m4 (w4, in24, in25, in26, in27, in28, in29, in30, in31, select[2:0]);
	
	mux_4_1 m5 (out, w1, w2, w3, w4, select[4:3]);
	
endmodule 

module mux_4_1 (out, in0, in1, in2, in3, select);

	input [31:0] in0, in1, in2, in3;
	input [1:0] select;
	
	output [31:0] out;
	
	wire [31:0] w1, w2;
	mux_2_1 m1 (w1, in0, in1, select[0]);
	mux_2_1 m2 (w2, in2, in3, select[0]);
	
	mux_2_1 m3 (out, w1, w2, select[1]);

endmodule 

module mux_8_1 (out, in0, in1, in2, in3, in4, in5, in6, in7, select);

	input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
	input [2:0] select;
	
	output [31:0] out;
	
	wire [31:0] w1, w2;
	mux_4_1 m1 (w1, in0, in1, in2, in3, select[1:0]);
	mux_4_1 m2 (w2, in4, in5, in6, in7, select[1:0]);
	
	mux_2_1 m3 (out, w1, w2, select[2]);

endmodule 
