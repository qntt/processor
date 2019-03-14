module latch_mw (ir_in, o_in, d_in, clock, reset, ir_out, o_out, d_out);

input [31:0] ir_in, o_in, d_in;
input clock, reset;

output [31:0] ir_out, o_out, d_out;

register ir (
    .data_out(ir_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(ir_in)
);

register o (
    .data_out(o_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(o_in)
);

register d (
    .data_out(d_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(d_in)
);


endmodule 