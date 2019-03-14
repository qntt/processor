`timescale 1 ns / 100 ps

module proc_tb();

	 integer CYCLE_LIMIT = 20;
	 
    // inputs to the proc are reg type
    reg            clock, reset;

    skeleton dut (clock, reset);

    wire[31:0] pc = dut.my_processor.pc;
	 wire[31:0] o_xm = dut.my_processor.o_xm;
	 wire[31:0] a_dx = dut.my_processor.a_dx;
	 wire isI_x = dut.my_processor.isI_x;
	 wire[31:0] signextend = dut.my_processor.signextend;
	 wire[31:0] alu_input_2 = dut.my_processor.alu_input_2;
	 
    initial
    begin
        $display($time, "<< Starting the Simulation >>");
        clock = 1'b0;    // at time 0
        
        $monitor("clock: %d, pc: %d, a_dx: %d, o_xm: %b, isI_x: %d, signextend: %b, alu_input_2: %b", clock, pc, a_dx, o_xm, isI_x, signextend, alu_input_2);
		  
		  #(20*(CYCLE_LIMIT+1.5))

        $stop;
    end

    // Clock generator
    always
         #10     clock = ~clock;
endmodule