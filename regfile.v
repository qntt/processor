module regfile (
    clock,
    ctrl_writeEnable,
    ctrl_reset, ctrl_writeReg,
    ctrl_readRegA, ctrl_readRegB, data_writeReg,
    data_readRegA, data_readRegB
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;

   output [31:0] data_readRegA, data_readRegB;

   /* YOUR CODE HERE */
	
	wire [1023:0] read_output;
	
	wire [31:0] write_bits;
	decoder dcW(ctrl_writeReg, write_bits);
	
	wire [31:0] readA_bits;
	decoder dcA(ctrl_readRegA, readA_bits);
	
	wire [31:0] readB_bits;
	decoder dcB(ctrl_readRegB, readB_bits);
	
	wire [31:0] write_enable_bit;
	and a0(write_enable_bit[0], ctrl_writeEnable, write_bits[0]);
	and a1(write_enable_bit[1], ctrl_writeEnable, write_bits[1]);
	and a2(write_enable_bit[2], ctrl_writeEnable, write_bits[2]);
	and a3(write_enable_bit[3], ctrl_writeEnable, write_bits[3]);
	and a4(write_enable_bit[4], ctrl_writeEnable, write_bits[4]);
	and a5(write_enable_bit[5], ctrl_writeEnable, write_bits[5]);
	and a6(write_enable_bit[6], ctrl_writeEnable, write_bits[6]);
	and a7(write_enable_bit[7], ctrl_writeEnable, write_bits[7]);
	and a8(write_enable_bit[8], ctrl_writeEnable, write_bits[8]);
	and a9(write_enable_bit[9], ctrl_writeEnable, write_bits[9]);
	and a10(write_enable_bit[10], ctrl_writeEnable, write_bits[10]);
	and a11(write_enable_bit[11], ctrl_writeEnable, write_bits[11]);
	and a12(write_enable_bit[12], ctrl_writeEnable, write_bits[12]);
	and a13(write_enable_bit[13], ctrl_writeEnable, write_bits[13]);
	and a14(write_enable_bit[14], ctrl_writeEnable, write_bits[14]);
	and a15(write_enable_bit[15], ctrl_writeEnable, write_bits[15]);
	and a16(write_enable_bit[16], ctrl_writeEnable, write_bits[16]);
	and a17(write_enable_bit[17], ctrl_writeEnable, write_bits[17]);
	and a18(write_enable_bit[18], ctrl_writeEnable, write_bits[18]);
	and a19(write_enable_bit[19], ctrl_writeEnable, write_bits[19]);
	and a20(write_enable_bit[20], ctrl_writeEnable, write_bits[20]);
	and a21(write_enable_bit[21], ctrl_writeEnable, write_bits[21]);
	and a22(write_enable_bit[22], ctrl_writeEnable, write_bits[22]);
	and a23(write_enable_bit[23], ctrl_writeEnable, write_bits[23]);
	and a24(write_enable_bit[24], ctrl_writeEnable, write_bits[24]);
	and a25(write_enable_bit[25], ctrl_writeEnable, write_bits[25]);
	and a26(write_enable_bit[26], ctrl_writeEnable, write_bits[26]);
	and a27(write_enable_bit[27], ctrl_writeEnable, write_bits[27]);
	and a28(write_enable_bit[28], ctrl_writeEnable, write_bits[28]);
	and a29(write_enable_bit[29], ctrl_writeEnable, write_bits[29]);
	and a30(write_enable_bit[30], ctrl_writeEnable, write_bits[30]);
	and a31(write_enable_bit[31], ctrl_writeEnable, write_bits[31]);
	
	register r0(read_output[31:0], clock, 1'b0, ctrl_reset, data_writeReg);
	
	//wire [1023:0] intermediate;
	
	generate
		genvar i;
		for (i=1; i<32; i = i+1) begin: gen1
			register r1 (
				 .data_out(read_output[32*(i+1)-1: 32*i]),
				 .clock(clock),
				 .ctrl_writeEnable(write_enable_bit[i]),
				 .ctrl_reset(ctrl_reset), 
				 .data_in(data_writeReg)
			);
			/*register_neg r2 (
				 .data_out(read_output[32*(i+1)-1: 32*i]),
				 .clock(clock),
				 .ctrl_writeEnable(1'b1),
				 .ctrl_reset(ctrl_reset), 
				 .data_in(intermediate[32*(i+1)-1: 32*i])
			);*/
		end
	endgenerate
	
	generate
		for (i=0; i<32; i = i+1) begin: gen2
			tri_reg tr(read_output[32*(i+1)-1 : 32*i], data_readRegA, readA_bits[i]);
		end
	endgenerate
	
	generate
		for (i=0; i<32; i = i+1) begin: gen3
			tri_reg tr(read_output[32*(i+1)-1 : 32*i], data_readRegB, readB_bits[i]);
		end
	endgenerate
	
endmodule

module decoder (
    in, out
);

   input [4:0] in;
	output [31:0] out;
	
	wire n0, n1, n2, n3, n4;

	not not0(n0, in[0]);
	not not1(n1, in[1]);
	not not2(n2, in[2]);
	not not3(n3, in[3]);
	not not4(n4, in[4]);
	
	and and1(out[0], n0, n1, n2, n3, n4);
	and and2(out[1], in[0], n1, n2, n3, n4);
	and and3(out[2], n0, in[1], n2, n3, n4);
	and and4(out[3], in[0], in[1], n2, n3, n4);
	
	and and5(out[4], n0, n1, in[2], n3, n4);
	and and6(out[5], in[0], n1, in[2], n3, n4);
	and and7(out[6], n0, in[1], in[2], n3, n4);
	and and8(out[7], in[0], in[1], in[2], n3, n4);
	
	and and9(out[8], n0, n1, n2, in[3], n4);
	and and10(out[9], in[0], n1, n2, in[3], n4);
	and and11(out[10], n0, in[1], n2, in[3], n4);
	and and12(out[11], in[0], in[1], n2, in[3], n4);
	
	and and13(out[12], n0, n1, in[2], in[3], n4);
	and and14(out[13], in[0], n1, in[2], in[3], n4);
	and and15(out[14], n0, in[1], in[2], in[3], n4);
	and and16(out[15], in[0], in[1], in[2], in[3], n4);
	
	and and17(out[16], n0, n1, n2, n3, in[4]);
	and and18(out[17], in[0], n1, n2, n3, in[4]);
	and and19(out[18], n0, in[1], n2, n3, in[4]);
	and and20(out[19], in[0], in[1], n2, n3, in[4]);
	
	and and21(out[20], n0, n1, in[2], n3, in[4]);
	and and22(out[21], in[0], n1, in[2], n3, in[4]);
	and and23(out[22], n0, in[1], in[2], n3, in[4]);
	and and24(out[23], in[0], in[1], in[2], n3, in[4]);
	
	and and25(out[24], n0, n1, n2, in[3], in[4]);
	and and26(out[25], in[0], n1, n2, in[3], in[4]);
	and and27(out[26], n0, in[1], n2, in[3], in[4]);
	and and28(out[27], in[0], in[1], n2, in[3], in[4]);
	
	and and29(out[28], n0, n1, in[2], in[3], in[4]);
	and and30(out[29], in[0], n1, in[2], in[3], in[4]);
	and and31(out[30], n0, in[1], in[2], in[3], in[4]);
	and and32(out[31], in[0], in[1], in[2], in[3], in[4]);


endmodule

module dffe_ref(q, d, clk, en, clr);
   
   //Inputs
   input d, clk, en, clr;
   
   //Internal wire
   wire clr;

   //Output
   output q;
   
   //Register
   reg q;

   //Intialize q to 0
   initial
   begin
       q = 1'b0;
   end

   //Set value of q on positive edge of the clock or clear
   always @(posedge clk or posedge clr) begin
       //If clear is high, set q to 0
       if (clr) begin
           q <= 1'b0;
       //If enable is high, set q to the value of d
       end else if (en) begin
           q <= d;
       end
   end
endmodule

module dffe_ref_neg(q, d, clk, en, clr);
   
   //Inputs
   input d, clk, en, clr;
   
   //Internal wire
   wire clr;

   //Output
   output q;
   
   //Register
   reg q;

   //Intialize q to 0
   initial
   begin
       q = 1'b0;
   end

   //Set value of q on positive edge of the clock or clear
   always @(negedge clk or posedge clr) begin
       //If clear is high, set q to 0
       if (clr) begin
           q <= 1'b0;
       //If enable is high, set q to the value of d
       end else if (en) begin
           q <= d;
       end
   end
endmodule

module register (
    data_out,
	 clock,
    ctrl_writeEnable,
    ctrl_reset, data_in
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [31:0] data_in;
	output [31:0] data_out;
	
	dffe_ref d0(data_out[0], data_in[0], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d1(data_out[1], data_in[1], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d2(data_out[2], data_in[2], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d3(data_out[3], data_in[3], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d4(data_out[4], data_in[4], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d5(data_out[5], data_in[5], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d6(data_out[6], data_in[6], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d7(data_out[7], data_in[7], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d8(data_out[8], data_in[8], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d9(data_out[9], data_in[9], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d10(data_out[10], data_in[10], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d11(data_out[11], data_in[11], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d12(data_out[12], data_in[12], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d13(data_out[13], data_in[13], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d14(data_out[14], data_in[14], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d15(data_out[15], data_in[15], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d16(data_out[16], data_in[16], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d17(data_out[17], data_in[17], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d18(data_out[18], data_in[18], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d19(data_out[19], data_in[19], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d20(data_out[20], data_in[20], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d21(data_out[21], data_in[21], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d22(data_out[22], data_in[22], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d23(data_out[23], data_in[23], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d24(data_out[24], data_in[24], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d25(data_out[25], data_in[25], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d26(data_out[26], data_in[26], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d27(data_out[27], data_in[27], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d28(data_out[28], data_in[28], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d29(data_out[29], data_in[29], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d30(data_out[30], data_in[30], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref d31(data_out[31], data_in[31], clock, ctrl_writeEnable, ctrl_reset);

endmodule

module register_neg (
    data_out,
	 clock,
    ctrl_writeEnable,
    ctrl_reset, data_in
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [31:0] data_in;
	output [31:0] data_out;
	
	dffe_ref_neg d0(data_out[0], data_in[0], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d1(data_out[1], data_in[1], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d2(data_out[2], data_in[2], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d3(data_out[3], data_in[3], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d4(data_out[4], data_in[4], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d5(data_out[5], data_in[5], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d6(data_out[6], data_in[6], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d7(data_out[7], data_in[7], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d8(data_out[8], data_in[8], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d9(data_out[9], data_in[9], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d10(data_out[10], data_in[10], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d11(data_out[11], data_in[11], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d12(data_out[12], data_in[12], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d13(data_out[13], data_in[13], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d14(data_out[14], data_in[14], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d15(data_out[15], data_in[15], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d16(data_out[16], data_in[16], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d17(data_out[17], data_in[17], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d18(data_out[18], data_in[18], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d19(data_out[19], data_in[19], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d20(data_out[20], data_in[20], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d21(data_out[21], data_in[21], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d22(data_out[22], data_in[22], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d23(data_out[23], data_in[23], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d24(data_out[24], data_in[24], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d25(data_out[25], data_in[25], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d26(data_out[26], data_in[26], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d27(data_out[27], data_in[27], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d28(data_out[28], data_in[28], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d29(data_out[29], data_in[29], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d30(data_out[30], data_in[30], clock, ctrl_writeEnable, ctrl_reset);
	dffe_ref_neg d31(data_out[31], data_in[31], clock, ctrl_writeEnable, ctrl_reset);

endmodule

module tri_buf (in, out, enable);
	
	input in, enable;
	output out;
	
	assign out = enable ? in : 1'bz;
	
endmodule

module tri_reg (in, out, enable);
	
	input enable;
	input [31:0] in;
	output [31:0] out;
	
	assign out = enable ? in : 32'bz;
	
endmodule 
