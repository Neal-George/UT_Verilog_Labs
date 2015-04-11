module top(clk100MHz,digit_enable,segment,DP,dIn,clkIn,LED);
  input clk100MHz,dIn,clkIn;
  output DP,digit_enable,segment,LED;
  wire [3:0] digit_enable;
  wire [6:0] segment;
  wire [3:0] sevSegData1, sevSegData0;
  wire [3:0] BCD;
  wire [3:0] EN3;
  wire [3:0] zero;
  wire fast7SegCLK;


  begin
  // fast7SegClk is used to alternate the 7 segment display output enables to make them all visible at once
  seven_segment_mux  mux(fast7SegCLK,sevSegData0,sevSegData1,zero,zero,BCD,digit_enable,DP,EN3);
  speedup fast(clk100MHz, fast7SegCLK);
  seven_segment display(segment,BCD);
  ps2Controller control(clk100MHz, dIn, clkIn, LED, sevSegData1, sevSegData0);
  assign EN3=4'b0000;
  assign zero=4'b0000;
  end
endmodule


module ps2Controller(clk100MHz, dIn, clkIn, LED, sevSegData1, sevSegData0);
 input clk100MHz, dIn, clkIn;
 output reg [3:0] sevSegData1;
 output reg [3:0] sevSegData0;
 output wire LED;
 wire [7:0] dataIn;
 reg [21:0] shiftReg;
 reg keyRelease;
 wire singlePulse;

 
 // shift in the data on the falling edge of PS2 clk
 always @ (negedge clkIn)
 begin
   shiftReg <= {dIn, shiftReg[21:1]};
 end

 always@(negedge clk100MHz)
  begin
  if(shiftReg[8:1]==8'b11110000)
  begin
  sevSegData1<=shiftReg[19:16];
  sevSegData0<=shiftReg[15:12];
  keyRelease<=1;
  end
  else
  begin
  sevSegData0<=sevSegData0;
  sevSegData1<=sevSegData1;
  keyRelease<=0;
  end
 

  end

 synchCircuit sync(keyRelease, clk100MHz, singlePulse);
 freqMul fm(singlePulse, clk100MHz, LED);


endmodule

module seven_segment(segment,BCD);
 input [3:0] BCD;
 output [6:0] segment;
 reg [6:0] segment;


 always@(BCD)
 begin
 case (BCD)
 4'b0000:segment=7'b1000000;
 4'b0001:segment=7'b1111001;
 4'b0010:segment=7'b0100100;
 4'b0011:segment=7'b0110000;
 4'b0100:segment=7'b0011001;
 4'b0101:segment=7'b0010010;
 4'b0110:segment=7'b0000010;
 4'b0111:segment=7'b1111000;
 4'b1000:segment=7'b0000000;
 4'b1001:segment=7'b0010000;
 4'b1010:segment=7'b0001000;//A
 4'b1011:segment=7'b0000011;//B
 4'b1100:segment=7'b1000110;//C
 4'b1101:segment=7'b0100001;//D
 4'b1110:segment=7'b0000110;//E
 4'b1111:segment=7'b0001110;//F
 default:segment=7'b1111111;
 endcase
 end

endmodule



module seven_segment_mux(CLK,digit1,digit2,digit3,digit4,BCD,digit_enable,DP,EN);
 input CLK;
 input [3:0] digit1,digit2,digit3,digit4,EN;
 output DP;
 output [3:0] BCD,digit_enable;
 reg [3:0] digit_enable,BCD;
 reg [1:0] switch;
 reg DP;

 always@(posedge CLK)
 begin
 case(switch)
  2'b00: begin
  BCD<=digit1;
  switch<=2'b01;
  digit_enable<=4'b1110|EN;
  DP<=1;
  end
  2'b01: begin
  BCD<=digit2;
  switch<=2'b10;
  digit_enable<=4'b1101|EN;
  DP<=1;
  end
  2'b10: begin
  BCD<=digit3;
  switch<=2'b11;
  digit_enable<=4'b1011|EN;
  DP<=1;
  end
  2'b11: begin
  BCD<=digit4;
  switch<=2'b00;
  digit_enable<=4'b0111|EN;
  DP<=1;
  end
  default: begin
  BCD<=digit1;
  switch<=2'b00;
  digit_enable<=4'b1111|EN;
  DP<=1;
  end
  endcase


 end
endmodule




module speedup(clk100MHz, slowClk);
 
  input clk100MHz; //fast clock
  output slowClk; //slow clock
 
  reg [15:0] counter;
 
  // switch to 27 for visible division
  assign slowClk = counter[15]; //(2^27 / 100E6) = 1.34seconds
 
  initial
  begin
  counter <= 0;
  end


  always @ (posedge clk100MHz)
  begin
  counter <= counter + 1; //increment the counter every 10ns (1/100 Mhz) cycle.
  end

endmodule

// cause pulse in to be output for the same amount of time
// as the divider period
module freqMul (pulseIn, clk100MHz, pulse1SecOut);
  input pulseIn, clk100MHz;
  output pulse1SecOut;
  reg [25:0] counter; // needs to be one more than that number of bits in freqMulSmallPulse
  // wire inClk;

  assign pulse1SecOut = counter [25];
  // assign inClk = clk100MHz || pulseIn;
   
  always @ (posedge clk100MHz, posedge pulseIn)
  begin
  if (pulseIn == 1)
  counter <= -2;
  else
  if (counter != 0)
  counter <= counter - 1;
  else
  counter <= 0;
  end

endmodule


module synchCircuit(switch, clk100MHz, synchPulse);
  input wire switch, clk100MHz;
  output wire synchPulse; // synchOut is the debounced output from second DFF
  wire s1, s0;
  wire q1, q1n, synchOutN;

  DFF d3(switch, clk100MHz,  s1,  s0);  // clock period is what outer module
                                              // is synchronized to

  assign synchPulse = (s0 && switch);  

endmodule


module DFF(d, clk, out, outN);
  input wire d, clk;
  output wire out, outN;
  reg qInternal;

  always @(posedge clk)
  begin
  qInternal <= d;
  end

  assign out = qInternal;
  assign outN = ~qInternal;

endmodule


