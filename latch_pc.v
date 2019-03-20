module latch_pc (pc_in, clock, reset, enable, pc_out);

input [31:0] pc_in;
input clock, reset, enable;

output [31:0] pc_out;

register pc (
    .data_out(pc_out),
	 .clock(clock),
    .ctrl_writeEnable(enable),
    .ctrl_reset(reset),
	 .data_in(pc_in)
);

endmodule 