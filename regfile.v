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
	
	wire [31:0] write_address_decoder_out;
	
	// setting the output of the decoder using the write addresses
	
	Decoder decoder (.out(write_address_decoder_out),
							.in(ctrl_writeReg));
	
	
	wire [31:0] reg_write_enable_input;
	wire [1023:0] register_output; 
	
	and a0 (reg_write_enable_input[0], ctrl_writeEnable, write_address_decoder_out[0]);
	and a1 (reg_write_enable_input[1], ctrl_writeEnable, write_address_decoder_out[1]);
	and a2 (reg_write_enable_input[2], ctrl_writeEnable, write_address_decoder_out[2]);
	and a3 (reg_write_enable_input[3], ctrl_writeEnable, write_address_decoder_out[3]);
	and a4 (reg_write_enable_input[4], ctrl_writeEnable, write_address_decoder_out[4]);
	and a5 (reg_write_enable_input[5], ctrl_writeEnable, write_address_decoder_out[5]);
	and a6 (reg_write_enable_input[6], ctrl_writeEnable, write_address_decoder_out[6]);
	and a7 (reg_write_enable_input[7], ctrl_writeEnable, write_address_decoder_out[7]);
	and a8 (reg_write_enable_input[8], ctrl_writeEnable, write_address_decoder_out[8]);
	and a9 (reg_write_enable_input[9], ctrl_writeEnable, write_address_decoder_out[9]);
	and a10 (reg_write_enable_input[10], ctrl_writeEnable, write_address_decoder_out[10]);
	and a11 (reg_write_enable_input[11], ctrl_writeEnable, write_address_decoder_out[11]);
	and a12 (reg_write_enable_input[12], ctrl_writeEnable, write_address_decoder_out[12]);
	and a13 (reg_write_enable_input[13], ctrl_writeEnable, write_address_decoder_out[13]);
	and a14 (reg_write_enable_input[14], ctrl_writeEnable, write_address_decoder_out[14]);
	and a15 (reg_write_enable_input[15], ctrl_writeEnable, write_address_decoder_out[15]);
	and a16 (reg_write_enable_input[16], ctrl_writeEnable, write_address_decoder_out[16]);
	and a17 (reg_write_enable_input[17], ctrl_writeEnable, write_address_decoder_out[17]);
	and a18 (reg_write_enable_input[18], ctrl_writeEnable, write_address_decoder_out[18]);
	and a19 (reg_write_enable_input[19], ctrl_writeEnable, write_address_decoder_out[19]);
	and a20 (reg_write_enable_input[20], ctrl_writeEnable, write_address_decoder_out[20]);
	and a21 (reg_write_enable_input[21], ctrl_writeEnable, write_address_decoder_out[21]);
	and a22 (reg_write_enable_input[22], ctrl_writeEnable, write_address_decoder_out[22]);
	and a23 (reg_write_enable_input[23], ctrl_writeEnable, write_address_decoder_out[23]);
	and a24 (reg_write_enable_input[24], ctrl_writeEnable, write_address_decoder_out[24]);
	and a25 (reg_write_enable_input[25], ctrl_writeEnable, write_address_decoder_out[25]);
	and a26 (reg_write_enable_input[26], ctrl_writeEnable, write_address_decoder_out[26]);
	and a27 (reg_write_enable_input[27], ctrl_writeEnable, write_address_decoder_out[27]);
	and a28 (reg_write_enable_input[28], ctrl_writeEnable, write_address_decoder_out[28]);
	and a29 (reg_write_enable_input[29], ctrl_writeEnable, write_address_decoder_out[29]);
	and a30 (reg_write_enable_input[30], ctrl_writeEnable, write_address_decoder_out[30]);
	and a31 (reg_write_enable_input[31], ctrl_writeEnable, write_address_decoder_out[31]);
	
	generate
		genvar i;
		for (i=0; i<32; i=i+1)
			begin: gen1
				Register reg1 (
					.out(register_output[32*i+31:32*i]),
					.in(data_writeReg),
					.clock(clock),
					.input_enable(reg_write_enable_input[i]),
					.output_enable(1'b1),
					.clear(ctrl_reset));
			end
	endgenerate
	
	wire [31:0] readA_bits_enable;
	Decoder dcA (readA_bits, ctrl_readRegA);
	
	generate
		genvar j;
		for (j=0; j<32; j=j+1) begin: gen2
			TriStateBuffer32 tristate (data_readRegA, register_output[32*j+31:32*j], readA_bits_enable[j]);
		end
	endgenerate
	
	wire [31:0] readB_bits_enable;
	Decoder dcB (readB_bits, ctrl_readRegB_enable);
	
	generate
		genvar k;
		for (k=0; k<32; k=k+1) begin: gen3
			TriStateBuffer32 tristate2 (data_readRegB, register_output[32*k+31:32*k], readB_bits_enable[k]);
		end
	endgenerate
	
endmodule

module Decoder (out, in);

	input [31:0] in;
	output [31:0] out;
	
	and a1 (out[0], ~in[4], ~in[3], ~in[2], ~in[1], ~in[0]);
	and a2 (out[1], ~in[4], ~in[3], ~in[2], ~in[1], in[0]);
	and a3 (out[2], ~in[4], ~in[3], ~in[2], in[1], ~in[0]);
	and a4 (out[3], ~in[4], ~in[3], ~in[2], in[1], in[0]);
	and a5 (out[4], ~in[4], ~in[3], in[2], ~in[1], ~in[0]);
	and a6 (out[5], ~in[4], ~in[3], in[2], ~in[1], in[0]);
	and a7 (out[6], ~in[4], ~in[3], in[2], in[1], ~in[0]);
	and a8 (out[7], ~in[4], ~in[3], in[2], in[1], in[0]);
	and a9 (out[8], ~in[4], in[3], ~in[2], ~in[1], ~in[0]);
	and a0 (out[9], ~in[4], in[3], ~in[2], ~in[1], in[0]);
	and a11 (out[10], ~in[4], in[3], ~in[2], in[1], ~in[0]);
	and a12 (out[11], ~in[4], in[3], ~in[2], in[1], in[0]);
	and a13 (out[12], ~in[4], in[3], in[2], ~in[1], ~in[0]);
	and a14 (out[13], ~in[4], in[3], in[2], ~in[1], in[0]);
	and a15 (out[14], ~in[4], in[3], in[2], in[1], ~in[0]);
	and a16 (out[15], ~in[4], in[3], in[2], in[1], in[0]);
	and a17 (out[16], in[4], ~in[3], ~in[2], ~in[1], ~in[0]);
	and a18 (out[17], in[4], ~in[3], ~in[2], ~in[1], in[0]);
	and a19 (out[18], in[4], ~in[3], ~in[2], in[1], ~in[0]);
	and a20 (out[19], in[4], ~in[3], ~in[2], in[1], in[0]);
	and a21 (out[20], in[4], ~in[3], in[2], ~in[1], ~in[0]);
	and a22 (out[21], in[4], ~in[3], in[2], ~in[1], in[0]);
	and a23 (out[22], in[4], ~in[3], in[2], in[1], ~in[0]);
	and a24 (out[23], in[4], ~in[3], in[2], in[1], in[0]);
	and a25 (out[24], in[4], in[3], ~in[2], ~in[1], ~in[0]);
	and a26 (out[25], in[4], in[3], ~in[2], ~in[1], in[0]);
	and a27 (out[26], in[4], in[3], ~in[2], in[1], ~in[0]);
	and a28 (out[27], in[4], in[3], ~in[2], in[1], in[0]);
	and a29 (out[28], in[4], in[3], in[2], ~in[1], ~in[0]);
	and a30 (out[29], in[4], in[3], in[2], ~in[1], in[0]);
	and a31 (out[30], in[4], in[3], in[2], in[1], ~in[0]);
	and a32 (out[31], in[4], in[3], in[2], in[1], in[0]);

endmodule 

module Register (out, in, clock, input_enable, output_enable, clear);

	input [31:0] in;
	input clock, input_enable, output_enable, clear;
	output [31:0] out;
	
	wire clock_input;
	and (clock_input, clock, input_enable);
	
	wire [31:0] dffe_output;
	
	generate
		genvar i;
		for (i=0; i<32; i=i+1)
			begin: gen1
				dffe_ref dffe(dffe_output[i], in[i], clock, input_enable, clear);
			end
	endgenerate
	
	generate
		genvar j;
		for (j=0; j<32; j=j+1)
			begin: gen2
				TriStateBuffer tristate(out[j], dffe_output[j], output_enable);
			end
	endgenerate

endmodule 

module TriStateBuffer(out, in, enable);

	input in, enable;
	output out;
	
	assign out = enable ? in : 1'bz;
	
endmodule 

module TriStateBuffer32(out, in, enable);

	input [31:0] in;
	input enable;
	output [31:0] out;
	
	assign out[31:0] = enable ? in[31:0] : 32'bz;
	
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
