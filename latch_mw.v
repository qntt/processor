module latch_mw (ir_in, o_in, d_in, isRStatus_in, rStatus_in, clock, reset, ir_out, 
	o_out, d_out, isRStatus_out, rStatus_out);

input [31:0] ir_in, o_in, d_in, rStatus_in;
input clock, reset, isRStatus_in;

output [31:0] ir_out, o_out, d_out, rStatus_out;
output isRStatus_out;

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

register rStatus (
    .data_out(rStatus_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(rStatus_in)
);

dflipflop isRStatus (.d(isRStatus_in), .clk(clock), .clrn(~reset), .prn(1'b1), 
	.ena(1'b1), .q(isRStatus_out));


endmodule 
