module snake_register (value_in, index, clock, reset, enable, value_out);

input [31:0] value_in, index;
input clock, reset, enable;

output [250*32-1:0] value_out;

genvar i;
	generate
	for (i=0; i<250; i=i+1) begin: loop1
		register snake_reg (
			 .data_out(value_out[32*(i+1)-1 : 32*i]),
			 .clock(clock),
			 .ctrl_writeEnable((index == i) && enable),
			 .ctrl_reset(reset),
			 .data_in(value_in)
		);
	end
endgenerate


endmodule 