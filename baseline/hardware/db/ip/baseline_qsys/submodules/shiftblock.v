module shiftblock(
		  input 	 clk,
		  input 	 reset,
		  input [7:0] 	 datain,
		  input 	 shift,
		  output 	 valid,
		  output [127:0] taps
		  );

   reg [127:0] 			 frontreg;

   // 112-position shift reg input  = backin
   // 112-position shift reg output = backtaps[55:48]   
   wire 			 backregclken;
   wire [7:0] 			 backin;
   wire [7:0] 			 backout;
   wire [63:0] 			 backtaps; 			 
   shift112 backreg( backregen, clk, backin, backout, backtaps);

   reg [7:0] 			 count;
   wire 			 fill;
   
   always @(posedge clk or posedge reset)
     begin
	if (reset == 1'b1) 
	  begin
	     frontreg <= 128'h0;
	     count    <= 8'h0;
	  end
	else
	  begin
	     frontreg <= (fill & shift)    ? {frontreg[119:0], datain} :
			 (~fill)           ? {frontreg[119:0], backtaps[55:48]} :
			 frontreg;
	     count    <= (count == 8'd143) ? 8'h0 :
			 (fill & shift)    ? (count + 8'h1) :
			 (~fill)           ? (count + 8'h1) :
			 count;
	  end
     end
   
   assign fill      = (count < 8'd128);
   assign valid     = (count > 8'd15);
   assign taps      = frontreg;
   assign backregen = (fill & shift) | (~fill);
   assign backin    = frontreg[127:120];
 
endmodule
