module latch_xm (ir_in, o_in, b_in, isRStatus_in, rStatus_in, clock, reset, enable, ir_out, 
	o_out, b_out, isRStatus_out, rStatus_out);

input [31:0] ir_in, o_in, b_in, rStatus_in;
input clock, reset, enable, isRStatus_in;

output [31:0] ir_out, o_out, b_out, rStatus_out;
output isRStatus_out;

register ir (
    .data_out(ir_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(ir_in)
);

register o (
    .data_out(o_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(o_in)
);

register b (
    .data_out(b_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(b_in)
);

register rStatus (
    .data_out(rStatus_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(rStatus_in)
);

dflipflop isRStatus (.d(isRStatus_in), .clk(clock), .clrn(~reset), .prn(1'b1), 
	.ena(enable), .q(isRStatus_out));


endmodule 
