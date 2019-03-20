module multdiv(data_operandA, data_operandB,
				ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY);
				
	input [31:0] data_operandA, data_operandB; 
	input ctrl_MULT, ctrl_DIV, clock;
	
	output [31:0] data_result;
	output data_exception, data_resultRDY; 
	
	wire [31:0] data_result_mult, data_result_div;
	wire data_exception_mult, data_exception_div, data_resultRDY_mult, data_resultRDY_div;
	
	wire [31:0] data_a, data_b; 
	
	register reg_a (
    .data_out(data_a),
	 .clock(~clock),
    .ctrl_writeEnable(ctrl_MULT || ctrl_DIV),
    .ctrl_reset(1'b0),
	 .data_in(data_operandA)
	);
	
	register reg_b (
    .data_out(data_b),
	 .clock(~clock),
    .ctrl_writeEnable(ctrl_MULT || ctrl_DIV),
    .ctrl_reset(1'b0),
	 .data_in(data_operandB)
	);
	
	mult m1 (.out(data_result_mult), .multiplicand(data_a), .multiplier(data_b), 
				.clock(clock), .ctrl_MULT(ctrl_MULT), .data_exception(data_exception_mult), 
				.data_resultRDY(data_resultRDY_mult));
	
	div d1 (.out(data_result_div), .dividend(data_a), .divisor(data_b), .clock(clock), 
				.ctrl_DIV(ctrl_DIV), .data_exception(data_exception_div), 
				.data_resultRDY(data_resultRDY_div));
				
	wire control_asserted;
	wire control_select;
	or o1(control_asserted, ctrl_MULT, ctrl_DIV);
	 
	//The output of this signals whether it's a multiplication
	dflipflop op_control (.d(~ctrl_MULT), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(control_asserted), .q(control_select));
	
	
	//or or_data_resultRDY (data_resultRDY, data_resultRDY_mult, data_resultRDY_div);
	
	mux_2_1 result_mux (.out(data_result), .in1(data_result_mult), .in2(data_result_div), .enable(control_select));
	mux_21_all_1bit data_exception_mux (.out(data_exception), .in1(data_exception_mult), .in2(data_exception_div), .enable(control_select));
	mux_21_all_1bit resultRDY_mux (.out(data_resultRDY), .in1(data_resultRDY_mult), .in2(data_resultRDY_div), .enable(control_select));

endmodule 

module mux_2_1_enable (out, in1, in2, select, enable);

	input [31:0] in1, in2;
	input select, enable;
	
	output [31:0] out;
	
	wire out1;
	assign out1 = select ? in2 : in1;
	
	assign out = enable ? out1 : 32'b0;

endmodule 

module mult (out, multiplicand, multiplier, clock, ctrl_MULT, data_exception, data_resultRDY);

	input[31:0] multiplicand, multiplier;
	input clock, ctrl_MULT;
	
	output[31:0] out;
	output data_exception, data_resultRDY;
	
	wire [63:0] product;
	
	wire doNothing, shift_one;
	wire [4:0] opcode;
	
	// ======================= Counter
	
	wire [15:0] counter_data;
	shift_register_8 counter (.out(data_resultRDY), .in(ctrl_MULT),
									.clock(clock), .clear(ctrl_MULT), .data(counter_data));
	
	// ======================== Shifted multiplicand
	
	wire [31:0] multiplicand_final;
	wire [31:0] shifted_multiplicand;
	assign shifted_multiplicand[31:1] = multiplicand[30:0];
	assign shifted_multiplicand[0] = 1'b0;
	
	mux_2_1 mux_shift_multiplicand (.out(multiplicand_final), .in1(multiplicand), 
				.in2(shifted_multiplicand), .enable(shift_one));
	
	wire [31:0] alu_input_2;
	mux_2_1 mux_do_nothing (.out(alu_input_2), .in1(multiplicand_final), .in2(32'b0), 
				.enable(doNothing));

	
	// ======================== Main ALU Stuff
	
	wire [31:0] alu_input_1;
	
	assign alu_input_1[31] = product[63];
	assign alu_input_1[30] = product[63];
	assign alu_input_1[29:0] = product[63:34];
	
	wire [31:0] alu_output;
	wire alu_overflow;
	wire placeholder;
	alu main_alu (.data_operandA(alu_input_1), .data_operandB(alu_input_2), .ctrl_ALUopcode(opcode),
					.ctrl_shiftamt(5'b00000), .data_result(alu_output), .isNotEqual(), 
					.isLessThan(), .overflow(alu_overflow), .carry_in(1'b0));
						
	product_module product1 (.in_from_alu(alu_output), .clock(clock), .data_out(product[63:32]), 
									.clear(ctrl_MULT));
	multiplier_module  multiplier1 (.data_out(product[31:0]), .clock(clock), .clear(1'b0), 
									.placeholder(placeholder), .product_in(product[33:32]), 
									.multiplier_in(multiplier), .first(ctrl_MULT));
	
	// ======================== Overflow (Data Exception)
	
	wire overflow_in, overflow_booth;
	or overflow_or (overflow_in, alu_overflow, overflow_booth);
	dflipflop oveflow_ff (.d(overflow_in), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(overflow_booth));
	
	wire overflow_32;
	
	wire overflow_positive, overflow_negative;
	or or_overflow_positive (overflow_positive, product[63:34]);
	and and_overflow_negative (overflow_negative, product[63:34]);
	
	mux_21_all_1bit mux_pos_neg_overflow (.out(overflow_32), .in1(overflow_positive), .in2(overflow_negative), .enable(product[33]));
	wire isOverflow_32;
	xor xor_isOverflow (isOverflow_32, overflow_32, product[33]);
	
	wire is_data_exception;
	or isDataExcpetion (is_data_exception, isOverflow_32, overflow_booth);
	and and_exception (data_exception, is_data_exception, data_resultRDY);
	
	
	// ======================== ALU OpCode
	
	wire allZero, allOne;
	wire isSubtract, isAdd;
	and a1 (allZero, ~product[1], ~product[0], ~placeholder);
	and a2 (allOne, product[1], product[0], placeholder);
	or o1 (doNothing, allZero, allOne);
	
	and a3 (isSubtract, ~doNothing, product[1]);
	and a4 (isAdd, ~doNothing, ~product[1]);
	
	wire shift_one_sub, shift_one_add;
	and a5 (shift_one_sub, isSubtract, ~product[0], ~placeholder);
	and a6 (shift_one_add, isAdd, product[0], placeholder);
	or o2 (shift_one, shift_one_sub, shift_one_add);
	
	mux_2_1_5 opcode_mux (.out(opcode), .in1(5'b00000), .in2(5'b00001), .enable(isSubtract));
	
	// ========================= Final Product
	
	assign out[31:0] = product[33:2];

endmodule 

module mux_2_1_5 (out, in1, in2, enable);

	input [4:0] in1, in2;
	input enable;
	
	output [4:0] out;
	
	assign out = enable ? in2 : in1;

endmodule 

module div (out, dividend, divisor, clock, ctrl_DIV, data_exception, data_resultRDY);

	input [31:0] dividend, divisor;
	input clock, ctrl_DIV;
	
	output [31:0] out;
	output data_exception, data_resultRDY;

	wire [63:0] remainder_quotient;
	wire clear;
	assign clear = ctrl_DIV;
	
	// ======================= Counter
	
	wire [32:0] counter_data;
	shift_register_32 counter (.out(data_resultRDY), .in(ctrl_DIV),
									.clock(clock), .clear(ctrl_DIV), .data(counter_data));
	
	// ======================= Get positive value of divisor and dividend
	
	wire [31:0] divisor_final, dividend_final;
	
	wire [4:0] divisor_opcode;
	assign divisor_opcode[4:1] = 4'b0000;
	assign divisor_opcode[0] = divisor[31];
	alu alu_get_divisor (.data_operandA(32'b0), .data_operandB(divisor), .ctrl_ALUopcode(divisor_opcode),
				.ctrl_shiftamt(5'b00000), .data_result(divisor_final), .isNotEqual(), 
				.isLessThan(), .overflow(), .carry_in(1'b0));
	
	wire [4:0] dividend_opcode;
	assign dividend_opcode[4:1] = 4'b0000;
	assign dividend_opcode[0] = dividend[31];
	alu alu_get_dividend (.data_operandA(32'b0), .data_operandB(dividend), .ctrl_ALUopcode(dividend_opcode),
				.ctrl_shiftamt(5'b00000), .data_result(dividend_final), .isNotEqual(), 
				.isLessThan(), .overflow(), .carry_in(1'b0));
	
	
	// ======================= Data Exception (divide by zero)
	
	wire isDivideBy0;
	//or isDivisor0 (isDivideBy0, divisor0[31:0]);
	assign isDivideBy0 = ~|divisor[31:0];
	and and_exception (data_exception, isDivideBy0, data_resultRDY);

	// ======================= Main ALU
	
	wire [31:0] alu_input_1;
	assign alu_input_1[31:0] = remainder_quotient[63:32];
	
	wire [31:0] alu_output;
	wire alu_overflow;
	wire isLessThan;
	
	alu main_alu (.data_operandA(alu_input_1), .data_operandB(divisor_final), .ctrl_ALUopcode(5'b00001),
				.ctrl_shiftamt(5'b00000), .data_result(alu_output), .isNotEqual(), 
				.isLessThan(isLessThan), .overflow(alu_overflow), .carry_in(1'b0));
				
	remainder_module remainder_module (.in_from_alu(alu_output), .isLessThan(isLessThan), .clock(clock), 
										.data_out(remainder_quotient[63:32]), .clear(ctrl_DIV), 
										.from_quotient(remainder_quotient[31]));
										
	quotient_module quotient_module (.isLessThan(isLessThan), .first(ctrl_DIV), .clock(clock), 
										.data_out(remainder_quotient[31:0]), .clear(1'b0), .dividend(dividend_final));
										
	
	// ======================= Set output

	wire[4:0] quotient_opcode;
	assign quotient_opcode[4:1] = 4'b0000;
	xor quotient_sign_xor (quotient_opcode[0], dividend[31], divisor[31]);
	
	alu quotient_sign_alu (.data_operandA(32'b0), .data_operandB(remainder_quotient[31:0]), 
				.ctrl_ALUopcode(quotient_opcode), .ctrl_shiftamt(5'b00000), .data_result(out), 
				.isNotEqual(), .isLessThan(), .overflow(), .carry_in(1'b0));
				
	//assign out = remainder_quotient[31:0];
	

endmodule 

module multiplier_module (data_out, clock, clear, product_in, multiplier_in, first, placeholder);

	input [1:0] product_in;
	input [31:0] multiplier_in;
	input first, clock, clear;
	output [31:0] data_out;
	output placeholder;
	
	wire [31:0] from_product;
	wire to_placeholder;
	
	assign from_product[31] = product_in[1];
	assign from_product[30] = product_in[0];
	
	assign from_product[29:0] = data_out[31:2];
	assign to_placeholder = data_out[1];
	
	wire [31:0] data_in;
	wire data_in_placeholder;
	mux_2_1 m1 (.out(data_in), .in1(from_product), .in2(multiplier_in), .enable(first));
	mux_21_all_1bit m2 (.out(data_in_placeholder), .in1(to_placeholder), .in2(1'b0), .enable(first));
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		dflipflop d1 (.d(data_in[i]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data_out[i]));
	end
	endgenerate
	
	dflipflop d_placeholder (.d(data_in_placeholder), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(placeholder));

endmodule 

module product_module (in_from_alu, clock, data_out, clear);

	input [31:0] in_from_alu;
	input clock, clear;
	
	output [31:0] data_out;
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: flipflops_loop
		dflipflop d1 (.d(in_from_alu[i]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data_out[i]));
	end
	endgenerate

endmodule 

module quotient_module (isLessThan, first, clock, data_out, clear, dividend);

	input [31:0] dividend;
	input isLessThan, first, clock, clear;
	
	output [31:0] data_out;
	
	wire[31:0] data_in;
	
	wire[31:0] in1_for_mux;
	assign in1_for_mux[0] = ~isLessThan;
	assign in1_for_mux[31:1] = data_out[30:0];
	
	mux_2_1 m1 (.out(data_in), .in1(in1_for_mux), .in2(dividend), .enable(first));
	
	genvar i;
	generate
	for (i=0; i<32; i=i+1) begin: loop1
		dflipflop d1 (.d(data_in[i]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data_out[i]));
	end
	endgenerate
	

endmodule 

module remainder_module (in_from_alu, isLessThan, clock, data_out, clear, from_quotient);

	input [31:0] in_from_alu;
	input isLessThan, clock, clear, from_quotient;
	
	output [31:0] data_out;

	wire[31:0] data_in;
	
	mux_2_1_31_bits m1 (.out(data_in[31:1]), .in1(in_from_alu[30:0]), 
							.in2(data_out[30:0]), .enable(isLessThan));
							
	dflipflop d0 (.d(from_quotient), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data_out[0]));
							
	genvar i;
	generate
	for (i=1; i<32; i=i+1) begin: loop1
		dflipflop d1 (.d(data_in[i]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data_out[i]));
	end
	endgenerate

endmodule 

module shift_left_1 (out, in);

	input [15:0] in;
	output [15:0] out;
	
	assign out[15] = in[14];
	assign out[14] = in[13];
	assign out[13] = in[12];
	assign out[12] = in[11];
	assign out[11] = in[10];
	assign out[10] = in[9];
	assign out[9] = in[8];
	assign out[8] = in[7];
	assign out[7] = in[6];
	assign out[6] = in[5];
	assign out[5] = in[4];
	assign out[4] = in[3];
	assign out[3] = in[2];
	assign out[2] = in[1];
	assign out[1] = in[0];
	assign out[0] = 1'b0;
	
endmodule 

module shift_register_16 (out, in, clock, clear, data);

	input in, clock, clear;
	
	output out;
	
	output [15:0] data;
	
	dflipflop din (.d(in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(data[0]));
	
	genvar i;
	generate
	for (i=1; i<=15; i=i+1) begin: loop1
		dflipflop d1 (.d(data[i-1]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data[i]));
	end
	endgenerate
	
	dflipflop dout (.d(data[15]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(out));

endmodule 

module shift_register_32 (out, in, clock, clear, data);

	input in, clock, clear;
	
	output out;
	
	output [32:0] data;
	
	dflipflop din (.d(in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(data[0]));
	
	genvar i;
	generate
	for (i=1; i<=32; i=i+1) begin: loop1
		dflipflop d1 (.d(data[i-1]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data[i]));
	end
	endgenerate
	
	dflipflop dout (.d(data[32]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(out));

endmodule 

module shift_register_8 (out, in, clock, clear, data);

	input in, clock, clear;
	
	output out;
	
	output [15:0] data;
	
	dflipflop din (.d(in), .clk(clock), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(data[0]));
	
	genvar i;
	generate
	for (i=1; i<=15; i=i+1) begin: loop1
		dflipflop d1 (.d(data[i-1]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(data[i]));
	end
	endgenerate
	
	dflipflop dout (.d(data[15]), .clk(clock), .clrn(~clear), .prn(1'b1), .ena(1'b1), .q(out));

endmodule 

module mux_2_1_15_bits (out, in1, in2, enable);

	input [14:0] in1, in2;
	input enable;
	
	output [14:0] out;
	
	assign out = enable ? in2 : in1;

endmodule 

module mux_2_1_31_bits (out, in1, in2, enable);

	input [30:0] in1, in2;
	input enable;
	
	output [30:0] out;
	
	assign out = enable ? in2 : in1;

endmodule 