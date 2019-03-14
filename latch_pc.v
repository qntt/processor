module latch_pc (pc_in, clock, reset, pc_out);

input [31:0] pc_in;
input clock, reset;

output [31:0] pc_out;

register_neg pc (
    .data_out(pc_out),
	 .clock(clock),
    .ctrl_writeEnable(1'b1),
    .ctrl_reset(reset),
	 .data_in(pc_in)
);

endmodule 