module latch_xm (ir_in, o_in, b_in, clock, reset, ir_out, o_out, b_out);

input [31:0] ir_in, o_in, b_in;
input clock, reset;

output [31:0] ir_out, o_out, b_out;

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

register b (
    .data_out(b_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(b_in)
);


endmodule 