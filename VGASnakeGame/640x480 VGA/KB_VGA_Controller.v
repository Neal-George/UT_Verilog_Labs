`define RED_BLACK     0
`define RED_BLUE      0
`define RED_GREEN     0
`define RED_CYAN      0
`define RED_RED       15
`define RED_MAGENTA   15
`define RED_YELLOW    15
`define RED_WHITE     15
`define GREEN_BLACK   0
`define GREEN_BLUE    0
`define GREEN_GREEN   15
`define GREEN_CYAN    15
`define GREEN_RED     0
`define GREEN_MAGENTA 0
`define GREEN_YELLOW  15
`define GREEN_WHITE   15
`define BLUE_BLACK    0
`define BLUE_BLUE     15
`define BLUE_GREEN    0
`define BLUE_CYAN     15
`define BLUE_RED      0
`define BLUE_MAGENTA  15
`define BLUE_YELLOW   0
`define BLUE_WHITE    15


module top(clk100MHz,digit_enable,segment,DP,dIn,clkIn,LED,hsynch,vsynch,ROut,GOut,BOut,switch);
  input clk100MHz,dIn,clkIn;
  input [7:0] switch;
  output DP,digit_enable,segment,LED,hsynch,vsynch;
  output[3:0] ROut,GOut,BOut;
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
  VGA vgaoutput(clk100MHz,hsynch,vsynch,ROut,GOut,BOut,switch);
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


module VGA(clk100MHz,hsynch,vsynch,Rout,Gout,Bout,switch);
input clk100MHz;
input [7:0] switch;
output wire hsynch,vsynch;
output [3:0] Rout,Gout,Bout;
wire clk25MHz;
reg [9:0] hcount,vcount;
wire visenable;

initial
begin
hcount<=0;
vcount<=0;
end

always@(posedge clk25MHz)
begin
  
  if(hcount==799)
  begin
    hcount<=0;  
    if(vcount==524)
      vcount<=0;
    else
      vcount<=vcount+1;
  end  
  else
  begin
    hcount<=hcount+1;
    vcount<=vcount;
  end  

 
end


vgacontsiggen gen(hcount,vcount,visenable,hsynch,vsynch,clk25MHz);
colorOutGenerator color(switch, clk100MHz, visenable, Rout,Gout,Bout);
clock25MHz divide(clk100MHz,clk25MHz);
endmodule




module vgacontsiggen(hcount,vcount,visenable,hsynch,vsynch,clk25MHz);
input [9:0] hcount,vcount;
input clk25MHz;
output reg visenable,hsynch,vsynch;

begin
always@(hcount, vcount)
begin
  if((hcount<=639)&&(vcount<=479))
  begin
  visenable<=1'b1;
  hsynch <= 1'b1;
  vsynch <= 1'b1;
  end
  else
  begin
  visenable<=1'b0;
  if((hcount>=659)&&(hcount<=755))
  hsynch<=1'b0;
  else
  hsynch<=1'b1;
  if((vcount>=493)&&(vcount<=494))
  vsynch <= 1'b0;
  else
  vsynch <= 1'b1;
  end
 
end
end
endmodule

module colorOutGenerator (switch, clk100MHz, visenable, rOut, gOut, bOut);
input [7:0]switch;
input clk100MHz, visenable;
output [3:0] rOut;
output [3:0] gOut;
output [3:0] bOut;

 reg [3:0] rRegOut;
 reg [3:0] gRegOut;
 reg [3:0] bRegOut;

 reg [3:0] rValIn;
 reg [3:0] gValIn;
 reg [3:0] bValIn;


always @ (switch)
begin
  if (switch == 1)
  begin
  rValIn <= `RED_BLACK;
  gValIn <= `GREEN_BLACK;
  bValIn <= `BLUE_BLACK;
  end
  else if (switch == 2)
  begin
  rValIn <= `RED_BLUE;
  gValIn <= `GREEN_BLUE;
  bValIn <= `BLUE_BLUE;
  end
  else if (switch == 4)
  begin
  rValIn <= `RED_GREEN;
  gValIn <= `GREEN_GREEN;
  bValIn <= `BLUE_GREEN;
  end
  else if (switch == 8)
  begin
  rValIn <= `RED_CYAN;
  gValIn <= `GREEN_CYAN;
  bValIn <= `BLUE_CYAN;
  end
  else if (switch == 16)
  begin
  rValIn <= `RED_RED;
  gValIn <= `GREEN_RED;
  bValIn <= `BLUE_RED;
  end
  else if (switch == 32)
  begin
  rValIn <= `RED_MAGENTA;
  gValIn <= `GREEN_MAGENTA;
  bValIn <= `BLUE_MAGENTA;
  end
  else if (switch == 64)
  begin
  rValIn <= `RED_YELLOW;
  gValIn <= `GREEN_YELLOW;
  bValIn <= `BLUE_YELLOW;
  end
  else if (switch == 128)
  begin
  rValIn <= `RED_WHITE;
  gValIn <= `GREEN_WHITE;
  bValIn <= `BLUE_WHITE;
  end
  else
  begin
  rValIn <= 0;
  gValIn <= 0;
  bValIn <= 0;
  end
end

always @ (negedge clk100MHz)
begin
  rRegOut <= rValIn;
  gRegOut <= gValIn;
  bRegOut <= bValIn;
end

assign rOut[0] = rRegOut[0] && visenable;
assign rOut[1] = rRegOut[1] && visenable;
assign rOut[2] = rRegOut[2] && visenable;
assign rOut[3] = rRegOut[3] && visenable;

assign gOut[0] = gRegOut[0] && visenable;
assign gOut[1] = gRegOut[1] && visenable;
assign gOut[2] = gRegOut[2] && visenable;
assign gOut[3] = gRegOut[3] && visenable;

assign bOut[0] = bRegOut[0] && visenable;
assign bOut[1] = bRegOut[1] && visenable;
assign bOut[2] = bRegOut[2] && visenable;
assign bOut[3] = bRegOut[3] && visenable;

endmodule


module clock25MHz(clk100MHz,clk25MHz);
input clk100MHz;
output clk25MHz;
reg [2:0] count;

initial
begin
count<=0;
end

always@(posedge clk100MHz)
begin
count<=count+1;
end
assign clk25MHz= count[1];

endmodule


