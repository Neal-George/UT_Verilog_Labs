`define ADD 0
`define SUB 1
`define INC_DAR_DAR  0
`define DEC_DAR_DAR  1
`define SPR_DAR   2
`define INC_SPR_DAR2 3
`define RES_SPR   0
`define INC_SPR_SPR  1
`define DEC_SPR_SPR  2
`define DAR_OUT   0
`define SPR_OUT   1
`define INC_SPR_OUT  2
`define BUS_DVR   0
`define ADD_DVR   1
`define DVR_CLR   2
`define DAR_CLR   4
`define INC_SPR_DAR1 5
`define BUS_SWITCHES 8'b11111111
`define BUS_ARITH 8'b00000000


module top(clk, btns, swtchs, leds, segs, an,DP);
  input clk;
  input[3:0] btns;
  input[7:0] swtchs;
  output[7:0] leds;
  output[6:0] segs; // TODO 
  output[3:0] an;
 
  //might need to change some of these from wires to regs wire cs;
  wire we;
  wire[6:0] addr;
  wire[7:0] data_out_mem;
  wire[7:0] data_out_ctrl;
  reg[7:0] data_bus;
  wire synchbtnL;
  wire synchbtnR;
  wire [3:0] synchbtns;
  wire clk10Hz;
  wire fast7SegCLK;
  output DP;
  wire [3:0] BCD;
  wire [3:0] EN3;
  wire [3:0] unused;
  wire [3:0] digit1,digit2,digit3;
  wire [7:0] binary8bits;
  // assigning buttons to send to controller
  assign synchbtns[2] = btns[2];
  assign synchbtns[3] = btns[3];  
 //           switch, fastClk, debcounceClk,              synchPulse
 synchCircuit sC1(btns[0],  clk,  clk10Hz /* 10Hz */, synchbtns[0]);
 synchCircuit sC2(btns[1],  clk,  clk10Hz /* 10Hz */, synchbtns[1]);
 
 divider10Hz d1(clk, clk10Hz);



  always @(*)
  begin
  if (we)
    data_bus <= data_out_ctrl;
  else
    data_bus <= data_out_mem;
  end

    
    // fast7SegClk is used to alternate the 7 segment display output enables to make them all visible 
  seven_segment_mux  mux(fast7SegCLK,digit1,digit2,digit3,unused,BCD,an,DP,EN3);
  speedup fast(clk, fast7SegCLK);
  seven_segment display(segs,BCD);
  BCDConverter convert(binary8bits, digit3, digit2, digit1); 
//  assign digit3 = 4'b1000; 
//  assign digit2 = 4'b0101;
//  assign digit1 = 4'b0010;  

 
  assign EN3=4'b0000;
  assign unused=4'b0000;
 
  controller ctrl(clk, cs, we, addr, data_bus, data_out_ctrl,
  synchbtns, swtchs, leds, binary8bits);
 
  memory mem(clk, cs, we, addr, data_bus, data_out_mem);
 
  //add any other functions you need
  //(e.g. debouncing, multiplexing, clock-division, etc)
 
endmodule


// useful BCD Code converter code snipet from www.eng.utah.edu
module BCDConverter(binary, hundreds, tens, ones); 
  input [7:0] binary; 
  output reg [3:0] hundreds, tens, ones; 

  integer i; 
  always @ (binary)
  begin
    hundreds = 0; 
    tens = 0; 
    ones = 0; 

    for (i=7; i>=0; i = i - 1)
    begin
      if (hundreds >= 5)
        hundreds = hundreds + 3; 
      if (tens >= 5)
        tens = tens + 3; 
      if (ones >= 5)
        ones = ones + 3; 
      // 
      hundreds = hundreds << 1; 
      hundreds[0] = tens[3]; 
      tens = tens << 1; 
      tens[0] = ones[3]; 
      ones = ones << 1; 
      ones[0] = binary[i]; 
    end 
  end
endmodule 


module controller(clk, cs, we, address, data_in, data_out, btns, swtchs, leds, segs);
  input clk;
  output reg cs;
  output reg we;
  output reg [6:0] address;
  input[7:0] data_in;
  output[7:0] data_out;
  input[3:0] btns;
  input[7:0] swtchs;
  output[7:0] leds; // leds[7] isEmpty
  output[7:0] segs; // 7 segment bits TODO   
 
  reg [7:0] arithReg1;
  reg [7:0] arithReg2;
  reg [4:0] state,nextstate;
  reg [6:0] SPR,DAR;
  reg [7:0] DVR;
  wire [6:0] nextSPR, nextDAR;
  wire [7:0] nextDVR;
  wire[7:0] arithOut;
  reg [1:0] SPRsel;
  reg [2:0] DARsel;
  reg ArithSEL;
  reg [1:0] addressSel;
  reg aren1;
  reg aren2;
  reg DVRen;
  reg DARen;
  reg SPRen;
  reg [1:0] DVRSel;
  reg [7:0] busSEL;
  
  assign leds = {&SPR, DAR}; 

  // BCD to segs for DVR
  assign segs = DVR; 

 
//instantiated modules
DVRMUX dvr(data_in, arithOut, DVRSel, nextDVR);
DARMUX darmux1(DAR,SPR,nextDAR,DARsel);
SPRMux sprmux1(SPR,SPRsel,nextSPR);
arithRegAdder arithadder1(arithReg1, arithReg2, ArithSEL, arithOut);

assign data_out = ((~busSEL & arithOut)
                  | (busSEL & swtchs));

always @ (*)
begin
  case (addressSel)
  `DAR_OUT: address <= DAR;
  `SPR_OUT: address <= SPR;
  `INC_SPR_OUT: address <= SPR + 1;
  default: address <= 0;
  endcase
end

// udpating state with next state
// moore machine, so outputs taken care of in next always block
always @(posedge clk) // use posedge of clock and single pulse of input is set on posedge
begin
  state<=nextstate;  

  // write new SPR
  if (SPRen)
  SPR <= nextSPR;
  else
  SPR <= SPR;  

  // write new DAR
  if (DARen)
  DAR <= nextDAR;
  else
  DAR <= DAR;

  // write new DVR
  if (DVRen)
  DVR <= nextDVR;
  else
  DVR <= DVR;

  // mux reg inputs
  if (aren1)
  arithReg1 <= data_in;
  else
  arithReg1<=arithReg1;
  if(aren2)
  arithReg2<=data_in;
  else
  arithReg2 <=arithReg2;
end


always @ (state, btns[0], btns[1])
  begin
    
  case(state)

  0: begin //Default
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 0;
  SPRsel <= 0;
  ArithSEL<=0;  
  DVRSel<=0;  
  DARsel<=0;   
  addressSel<=0;
  busSEL<=`BUS_ARITH;
   
  // branch out of default state
  case(btns)

  0: begin //Default
  nextstate <= 0;
  end

  4'b0010: begin //Pop
  nextstate <= 1;
  end

  4'b0110: begin //Subtract
    nextstate <= 2;
  end

  4'b1010: begin //Clear
  nextstate<=3;
  end

  4'b1110: begin  //Decrement Address
  nextstate<=4;
  end

  4'b0001: begin  //Push
  nextstate<=5;
  end

  4'b0101: begin  //Add
  nextstate<=6;
  end

  4'b1001: begin  //Top
  nextstate<=7;
  end

  4'b1101: begin // Increment Address
    nextstate<=8;
  end
  default:begin
    nextstate<=0;
    end
  endcase
  end
  1: begin //Pop
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 1 ;
  DVRSel <= 0;
  DARsel <= `INC_SPR_DAR2;
  SPRsel <= `INC_SPR_SPR;
  ArithSEL <=0 ;   
  addressSel <= 0;
  nextstate <= 19;
    busSEL<=`BUS_ARITH;
  end

  2: begin //Subtract
  cs <=0 ;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 1;
  DARsel <= `INC_SPR_DAR2;
  DVRSel <= 0;
  SPRsel <= `INC_SPR_SPR;
  ArithSEL <= 0;
  nextstate <= 9;
  addressSel<= `INC_SPR_OUT;
    busSEL<=`BUS_ARITH;

  end

  3: begin //Clear
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 1;
  SPRen <= 1;
  DARsel <= `DAR_CLR;
  DVRSel <= `DVR_CLR;
  SPRsel <= `RES_SPR;
  ArithSEL <= 0;   
  nextstate <= 0;
  addressSel<= 0;
    busSEL<=`BUS_ARITH;
  end

  4: begin  //Decrement Address
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 0;
  DARsel <= `INC_DAR_DAR;
  DVRSel <=0 ;
  SPRsel <= 0;
  ArithSEL <=0 ;   
  nextstate <= 12;
  addressSel<= `DAR_OUT;
    busSEL<=`BUS_ARITH;
  end

  5: begin  //Push
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 1;
  DVRSel <= 0;
  DARsel <= `SPR_DAR;
  SPRsel <= `DEC_SPR_SPR;
  ArithSEL <=0 ;   
  addressSel <= `SPR_OUT; // only because this is the address that will be read
                          // and will be written to DAR end of cycle
   
  nextstate <= 13;
    busSEL <= `BUS_SWITCHES;
  end

  6: begin  //Add
  cs <=0 ;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 1;
  DARsel <= `INC_SPR_DAR2;
  DVRSel <= 0;
  SPRsel <= `INC_SPR_SPR;
  ArithSEL <= 0;
  nextstate <= 14;
  addressSel<= `INC_SPR_OUT;
    busSEL<=`BUS_ARITH;
   
  end

  7: begin  //Top
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 0;
  DARsel <= `INC_SPR_DAR1;
  DVRSel <=0 ;
  SPRsel <= 0;
  ArithSEL <=0 ;   
  nextstate <= 12;
  addressSel<= `INC_SPR_OUT;
    busSEL<=`BUS_ARITH;
     
  end

  8: begin  //Increment Adder
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 1;
  SPRen <= 0;
  DARsel <= `DEC_DAR_DAR;
  DVRSel <=0 ;
  SPRsel <= 0;
  ArithSEL <=0 ;   
  nextstate <= 12;
  addressSel<= `DAR_OUT;
    busSEL<=`BUS_ARITH;
  end

  9: begin  //Subtract Memory1 Read
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= 0;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 21;
  addressSel <= `SPR_OUT;
    busSEL<=`BUS_ARITH;
   
  end

  10: begin  //Subtract Memory2 Read
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 1;
  DVRen <= 0;
  DARen <= 0;
  DVRSel<=0;
  SPRen <= 0;
  DARsel <= 0;
  SPRsel <= 0;  
  ArithSEL <= 0;
  addressSel <= `SPR_OUT;    
  nextstate <= 22;    
    busSEL<=`BUS_ARITH;
  end

  11: begin // mem write from add

  cs <= 1;
  we <= 1;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  DVRSel<=0;
  SPRen <= 1;
  DARsel <= 0;
  SPRsel <= `DEC_SPR_SPR;  
  ArithSEL <= `ADD;
  addressSel <= `SPR_OUT;    
  nextstate <=0;   
    busSEL<=`BUS_ARITH;
  end

  12: begin // mem read from dec adder
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  DVRSel<=0;
  SPRen <= 0;
  DARsel <= 0;
  SPRsel <= 0;  
  ArithSEL <= 0;
  addressSel <= `DAR_OUT;    
  nextstate <=23;
    busSEL<=`BUS_ARITH;
  end
 
    13: begin // push write
  cs <= 1;
  we <= 1;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= `BUS_DVR;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 0;
  addressSel<= `DAR_OUT;
    busSEL<=`BUS_SWITCHES;

    end
 
  14: begin // 1st mem read from add
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= 0;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 24;
  addressSel <= `SPR_OUT;  
  busSEL<=`BUS_ARITH;
  end
    
  15: begin // 2nd mem read from add
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 1;
  DVRen <= 0;
  DARen <= 0;
  DVRSel<=0;
  SPRen <= 0;
  DARsel <= 0;
  SPRsel <= 0;  
  ArithSEL <= 0;
  addressSel <= `SPR_OUT;    
  nextstate <= 25;
    busSEL<=`BUS_ARITH;

  end
  
  16: begin // mem write from subtract
  
      cs <= 1;
      we <= 1;
      aren1 <= 0;
      aren2 <= 0;
      DVRen <= 0;
      DARen <= 0;
      DVRSel<=0;
      SPRen <= 1;
      DARsel <= 0;
      SPRsel <= `DEC_SPR_SPR;  
      ArithSEL <= `SUB;
      addressSel <= `SPR_OUT;    
      nextstate <=0;   
        busSEL<=`BUS_ARITH;
    end
  19: begin // mem read from pop
  cs <= 1;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  DVRSel<=0;
  SPRen <= 0;
  DARsel <= 0;
  SPRsel <= 0;
  ArithSEL <= 0;
  addressSel <= `DAR_OUT;    
  nextstate <= 20;
    busSEL<=`BUS_ARITH;
  end
  20: begin // mem read2 from pop
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= `BUS_DVR;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 0;
  addressSel<=0;
    busSEL<=`BUS_ARITH;
  end
  21: begin // mem read2 from subtract
  cs <= 1;
  we <= 0;
  aren1 <= 1;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 1;
  DARsel <= 0;
  DVRSel <= 0;
  SPRsel <= `INC_SPR_SPR;
  ArithSEL <= 0;   
  nextstate <= 10;
  addressSel<= `SPR_OUT;
    busSEL<=`BUS_ARITH;
  end
  22: begin // mem read2 from subtract
  cs <= 1;
  we <= 1;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= `ADD_DVR;
  SPRsel <= 0;
  ArithSEL <= `SUB;    
  nextstate <= 16;
  addressSel<= `SPR_OUT;
    busSEL<=`BUS_ARITH;
  end
  23: begin // mem latch from DEC ADDR
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= `BUS_DVR;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 0;
  addressSel<= `DAR_OUT;
  busSEL<=`BUS_ARITH;
  end
  24: begin // mem read2 from add
  cs <= 1;
  we <= 0;
  aren1 <= 1;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 1;
  DARsel <= 0;
  DVRSel <= 0;
  SPRsel <= `INC_SPR_SPR;
  ArithSEL <= 0;   
  nextstate <= 15;
  addressSel<= `SPR_OUT;
    busSEL<=`BUS_ARITH;
  end
  25: begin // mem read2 from add
  cs <= 1;
  we <= 1;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 1;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= `ADD_DVR;
  SPRsel <= 0;
  ArithSEL <= `ADD;    
  nextstate <= 11;
  addressSel<= `SPR_OUT;
    busSEL<=`BUS_ARITH;
  end

    
  default:begin
  cs <= 0;
  we <= 0;
  aren1 <= 0;
  aren2 <= 0;
  DVRen <= 0;
  DARen <= 0;
  SPRen <= 0;
  DARsel <= 0;
  DVRSel <= 0;
  SPRsel <= 0;
  ArithSEL <= 0;   
  nextstate <= 0;
  addressSel<= 0;
    busSEL<=0;
   
  end
    
  endcase
  end
endmodule

 

// controller component module
module DARMUX(OLDDAR,SPR,NEWDAR,DARsel);
  input [6:0] SPR,OLDDAR;
  input [2:0] DARsel;
  output reg [6:0] NEWDAR;

always@(*)
  begin
  case(DARsel)
  `SPR_DAR: NEWDAR<=SPR;  //2
  `DEC_DAR_DAR: NEWDAR<=OLDDAR-1;//1
  `INC_DAR_DAR: NEWDAR<=OLDDAR+1;//0
  `INC_SPR_DAR2: NEWDAR<= SPR+2;//3
  `DAR_CLR: NEWDAR <= 0;//4
  `INC_SPR_DAR1: NEWDAR <= SPR + 1;//4
  default: NEWDAR<=0;
  endcase
  end

endmodule

module DVRMUX(dataIn, arithOut, DVRSel, nextDVR);
input [7:0] dataIn, arithOut;
input [1:0] DVRSel;
 
output reg [7:0] nextDVR;

always @ (*)
begin
  case (DVRSel)
  `BUS_DVR: nextDVR <= dataIn;
  `ADD_DVR: nextDVR <= arithOut;
  `DVR_CLR: nextDVR<=0;
  default: nextDVR<=0;
  endcase
end

endmodule


// assumes that arithReg1 is the higher addressed data
module arithRegAdder(arithReg1, arithReg2, arithAddSub, arithOut);
  input [7:0] arithReg1;
  input [7:0] arithReg2;
  input arithAddSub;
  output reg [7:0] arithOut;

  always @ (*)
  begin
  case (arithAddSub)
  `ADD: begin
    arithOut = arithReg1 + arithReg2;
  end
  `SUB: begin
    arithOut = arithReg1 - arithReg2;
  end
  endcase
  end
endmodule

module SPRMux(oldSPR,SPRsel,newSPR);
input [6:0] oldSPR;
input [1:0] SPRsel;
output reg [6:0] newSPR;
 
always@(*)
begin
  case (SPRsel)
  0: newSPR<=7'b1111111;
  1: newSPR<=oldSPR+1;
  2: newSPR<= oldSPR-1;
  default: newSPR <= 0; 
  endcase
end
endmodule

module synchCircuit(switch, clk100MHz, clk10Hz, synchPulse);
  input wire switch, clk100MHz, clk10Hz;
  output wire synchPulse; // synchOut is the debounced output from second DFF
 
  wire s1, s0;
  wire q1, q1n, synchOut, synchOutN;

  DFF d1(switch,   clk10Hz, q1,   q1n);   // clock period is max debounce time
  DFF d2(q1,  clk10Hz,  synchOut, SynchOutN); // clock period is max debounce time
  DFF d3(synchOut, clk100MHz,  s1,  s0);  // clock period is what outer module
                                            // is synchronized to

  assign synchPulse = (s0 && synchOut);  

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


module divider10Hz(clk100MHz, clk10Hz);
  input  clk100MHz;
  output clk10Hz;

  reg[22:0] counter;
  assign clk10Hz = counter[22]; // slightly over 40Hz

   initial
  begin
  counter = 0;
  end

  always @ (posedge clk100MHz)
  begin
  counter <= counter + 1;
  end

endmodule


module memory(clock, cs, we, address, data_in, data_out);
  input clock;
  input cs;
  input we;
  input[6:0] address;
  input[7:0] data_in;
  output[7:0] data_out;
 
  reg[7:0] data_out;
 
  reg[7:0] RAM[0:127];
 
  always @ (negedge clock)
  begin
  if((we == 1) && (cs == 1))
  RAM[address] <= data_in[7:0];
 
  data_out <= RAM[address];
  end
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
  default:segment=7'b1111111;
  endcase
  end

endmodule


// clock used to alternate 7 seg outputs
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



