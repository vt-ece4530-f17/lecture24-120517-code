module shiftblockcp(
		    input wire 	       clk,
		    input wire 	       reset,
		    input wire 	[3:0]  address,
		    input wire 	       read,
		    output wire [31:0] readdata,
		    input wire 	       write, 
		    input wire [31:0]  writedata,
		    input wire [3:0]   byteenable
		    );

   wire [7:0] 			       datain;
   wire 			       shift;
   wire 			       valid;
   wire [127:0] 		       taps;
   
   shiftblock M1(clk,
		 reset,
		 datain,
		 shift,
		 valid,
		 taps);
   
   // writing into shiftblock
   wire 			       loaddata;   
   assign loaddata  = (write & (address == 4'h0) & (byteenable == 4'h1));
   assign shift     = loaddata;
   assign datain    = writedata[7:0];

   // max-accumulator logic
   reg [31:0] 			       maxsum;
   wire [31:0] 			       tapsum;
   wire 			       clearsum;
   
   assign tapsum =  taps[0  +: 8] + taps[  8 +: 8] + taps[ 16 +: 8] + taps[ 24 +: 8] +
		    taps[32 +: 8] + taps[ 40 +: 8] + taps[ 48 +: 8] + taps[ 56 +: 8] +
		    taps[64 +: 8] + taps[ 72 +: 8] + taps[ 80 +: 8] + taps[ 88 +: 8] +
		    taps[96 +: 8] + taps[104 +: 8] + taps[112 +: 8] + taps[120 +: 8];
   assign clearsum = (write & (address == 4'h1));
   
   always @(posedge clk or posedge reset)
     if (reset)
       maxsum <= 32'h0;
     else
       maxsum <= clearsum                    ? 32'h0  :
		 (valid & (tapsum > maxsum)) ? tapsum :
		 maxsum;
   
   // reading from shiftblock
   wire 			       readtap0;
   wire 			       readtap1;
   wire 			       readtap2;
   wire 			       readtap3;
   wire                                readmax;
   wire                                readvalid;
   
   assign readtap0  = (read & (address == 4'h0));
   assign readtap1  = (read & (address == 4'h1));
   assign readtap2  = (read & (address == 4'h2));
   assign readtap3  = (read & (address == 4'h3));
   assign readmax   = (read & (address == 4'h8));
   assign readvalid = (read & (address == 4'h9));

   assign readdata = readmax   ? maxsum         :
		     readvalid ? {31'h0, valid} :
		     readtap0  ? taps[ 31:  0]  :
		     readtap1  ? taps[ 63: 32]  :
		     readtap2  ? taps[ 95: 64]  :
		     readtap3  ? taps[127: 96]  :
		     32'h0;

endmodule
