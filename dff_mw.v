module dff_mw(ir_in, o_in, d_in, clk, clrn, prn, ena, ir, o, d);
    input clk, ena, clrn, prn;
	 input [31:0] ir_in, o_in, d_in;
    wire clr;
    wire pr;

    output [31:0] ir, o, d;
    reg [31:0] ir, o, d;

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
		  
		  o <= o_in;
		  d <= d_in;
    end
endmodule