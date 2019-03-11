module dff_fd(ir_in, pc_in, clk, clrn, prn, ena, ir, pc);
    input clk, ena, clrn, prn;
	 input [31:0] ir_in, pc_in;
    wire clr;
    wire pr;

    output [31:0] ir, pc;
    reg [31:0] ir, pc;

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
    end
endmodule 