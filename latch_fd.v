module latch_fd (ir_in, pc_in, clock, reset, enable, ir_out, pc_out);

input [31:0] ir_in, pc_in;
input clock, reset, enable;

output [31:0] ir_out, pc_out;

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


endmodule 