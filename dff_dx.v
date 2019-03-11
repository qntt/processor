module dff_dx(ir_in, pc_in, a_in, b_in, clk, clrn, prn, ena, ir, pc, a, b);
    input clk, ena, clrn, prn;
	 input [31:0] ir_in, pc_in, a_in, b_in;
    wire clr;
    wire pr;

    output [31:0] ir, pc, a, b;
    reg [31:0] ir, pc, a, b;

    assign clr = ~clrn;
    assign pr = ~prn;

    initial
    begin
        ir = 32'b0;
    end

    always @(posedge clk) begin
        if (ir == 32'bx) begin
            ir <= 32'b0;
        end else if (clr) begin
            ir <= 32'b0;
        end else if (ena) begin
            ir <= ir;
        end
		  
		  pc <= pc_in;
		  a <= a_in;
		  b <= b_in;
    end
endmodule 