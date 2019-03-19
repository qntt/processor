module latch_dx (ir_in, pc_in, a_in, b_in, clock, reset, enable, ir_out, pc_out, a_out, b_out);

input [31:0] ir_in, pc_in, a_in, b_in;
input clock, reset, enable;

output [31:0] ir_out, pc_out, a_out, b_out;

register ir (
    .data_out(ir_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(ir_in)
);

register pc (
    .data_out(pc_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(pc_in)
);

register a (
    .data_out(a_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(a_in)
);

register b (
    .data_out(b_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(b_in)
);


endmodule 