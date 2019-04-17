module snake_register (value_in, index, clock, reset, enable, value_out);

input [31:0] value_in, index;
input clock, reset, enable;

output [423:0] value_out;

genvar i;

generate
	for (i=0; i<=99; i=i+1) begin: loop1
		register_2 snake_2_bit_reg (
			 .data_out(value_out[2*i+1 : 2*i]),
			 .clock(clock),
			 .ctrl_writeEnable((index == i) && enable),
			 .ctrl_reset(reset),
			 .data_in(value_in[1:0])
		);
	end
endgenerate



register snake_reg_head1position (
	 .data_out(value_out[231:200]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 100) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_head2position (
	 .data_out(value_out[263:232]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 101) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_length1 (
	 .data_out(value_out[295:264]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 102) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_length2 (
	 .data_out(value_out[327:296]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 103) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_stage (
	 .data_out(value_out[359:328]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 104) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_head1 (
	 .data_out(value_out[391:360]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 105) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);

register snake_reg_head2 (
	 .data_out(value_out[423:392]),
	 .clock(clock),
	 .ctrl_writeEnable((index == 106) && enable),
	 .ctrl_reset(reset),
	 .data_in(value_in)
);



endmodule 