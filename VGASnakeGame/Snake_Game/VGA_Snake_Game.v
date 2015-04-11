`define RED_BLACK   0
`define RED_BLUE  0
`define RED_GREEN   0
`define RED_CYAN  0
`define RED_RED   15
`define RED_MAGENTA   15
`define RED_YELLOW  15
`define RED_WHITE   15
`define GREEN_BLACK   0
`define GREEN_BLUE  0
`define GREEN_GREEN   15
`define GREEN_CYAN  15
`define GREEN_RED   0
`define GREEN_MAGENTA 0
`define GREEN_YELLOW  15
`define GREEN_WHITE   15
`define BLUE_BLACK  0
`define BLUE_BLUE   15
`define BLUE_GREEN  0
`define BLUE_CYAN   15
`define BLUE_RED  0
`define BLUE_MAGENTA  15
`define BLUE_YELLOW   0
`define BLUE_WHITE  15

`define RED_ORANGE    255
`define GREEN_ORANGE  140
`define BLUE_ORANGE   0

`define LEFT  0
`define RIGHT 1
`define UP  2
`define DOWN  3

`define LEFT_PRESS  8'b01101011
`define RIGHT_PRESS 8'b01110100
`define UP_PRESS  8'b01110101
`define DOWN_PRESS  8'b01110010
`define S_KEY   8'b00011011
`define P_KEY   8'b01001101
`define R_KEY   8'b00101101
`define ESC_KEY   8'b01110110
`define Q_KEY   8'b00010101
`define W_KEY   8'b00011101

module top(clk100MHz,digit_enable,segment,DP,dIn,clkIn,LED,hsynch,vsynch,ROut,GOut,BOut);
  input clk100MHz,dIn,clkIn;
  
  output DP,digit_enable,segment,LED,hsynch,vsynch;
  output[3:0] ROut,GOut,BOut;
  
  reg  gameWon; 
  reg [2:0] snakecolor,backgroundcolor;
  reg [6:0] tenSecCount; 
  reg enableMove;
  reg enableSnake;
  reg inEscape;
  reg inPause;
  reg restartGame;
  reg snakecolormodified,backgroundcolormodied; 
  
  wire [3:0] BCD;
  wire [3:0] digit_enable;
  wire [3:0] EN3;
  wire [3:0] sevSegData1, sevSegData0;
  wire [3:0] zero;
  wire [6:0] segment;
  wire [7:0] keyBoard;
  wire [9:0] cornerNWx, cornerNEx, cornerSWx, cornerSEx, cornerNWy, cornerNEy, cornerSWy, cornerSEy;
  wire [9:0] hcount,vcount;
  wire clk25Hz;
  wire clk50Hz;
  wire colorsync;
  wire Escape;
  wire fast7SegCLK;
  wire gameover;
  wire Pause;
  wire Qpressed,Wpressed;
  wire restartPulse;
  wire snakemodified,backgroundmodified;
  wire visenable; 
  reg snakeSpeed; 
  reg [2:0] speedCount; 

  begin
  // fast7SegClk is used to alternate the 7 segment display output enables to make them all visible at once
  seven_segment_mux  mux(fast7SegCLK,sevSegData0,sevSegData1,zero,zero,BCD,digit_enable,DP,EN3);
  speedup fast(clk100MHz, fast7SegCLK);
  seven_segment display(segment,BCD);
  ps2Controller control(clk100MHz, dIn, clkIn, LED, sevSegData1, sevSegData0);
  VGA vgaoutput(clk100MHz,hsynch,vsynch,switch,visenable,hcount,vcount);
  colorOutGenerator color(clk100MHz, visenable, ROut, GOut, BOut,
                      cornerNWx, cornerNEx, cornerSWx, cornerSEx,
                      cornerNWy, cornerNEy, cornerSWy, cornerSEy,
                      hcount, vcount,snakecolor,backgroundcolor, 
                      gameWon);
  
  clock50Hz snakeclock(clk100MHz,clk50Hz);
  clock100Hz snakeclock2(clk100MHz,clk100Hz);
  clock200Hz snakeclock3(clk100MHz,clk200Hz);
  clock400Hz snakeclock4(clk100MHz,clk400Hz);
  clock800Hz snakeclock5(clk100MHz,clk800Hz);
  clock1600Hz snakeclock6(clk100MHz, clk1600Hz); 

  topSnake tS(snakeSpeed,
        cornerNWx, cornerNEx, cornerSWx, cornerSEx,
        cornerNWy, cornerNEy, cornerSWy, cornerSEy,
        keyBoard, enableMove, gameover, enableSnake, restartPulse);
       
  clock25Hz resClk(clk100MHz, clk25Hz);


  synchCircuit colorsynch(LED, clk100MHz, colorsync);

  assign restartPulse = (LED && restartGame);
  assign snakemodified=snakecolormodified;
  assign backgroundmodified=backgroundcolormodied;
  assign Qpressed= (keyBoard==`Q_KEY)&&(!colorsync);
  assign Wpressed= (keyBoard==`W_KEY)&&(!colorsync);

   // mux snake speed for position module
  always @ (*)
  begin
    if (speedCount == 0)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk50Hz;
    end 
    else if (speedCount == 1)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk100Hz;
    end
    else if (speedCount == 2)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk200Hz;
    end
    else if (speedCount == 3)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk400Hz;
    end 
    else if (speedCount == 4)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk800Hz; 
    end
    else if (speedCount == 4)
    begin
      gameWon <= 0; 
      snakeSpeed <= clk1600Hz; 
    end
    else if ((speedCount  == 5) 
             && (Pause    == 0)
             && (Escape   == 0)
             && (gameover == 0))
    begin
      gameWon <= 1; 
      snakeSpeed <= clk50Hz;
    end   
    else
    begin
      gameWon <= gameWon; 
      snakeSpeed <= snakeSpeed; 
    end
  end
  
  // block to increment speedCount every 10 seconds
  always @ (posedge clk50Hz)
  begin
    if (restartPulse)
    begin
      speedCount <= 0;
      tenSecCount <= -1; // hits 0 after 512 counts. start at 511 
    end
    else 
    begin
      if (tenSecCount == 0) // every 5 seconds
      begin
        speedCount <= speedCount + 1; 
        tenSecCount <= tenSecCount - 1; // every .02 seconds
      end
      else
      begin
        speedCount <= speedCount; 
        if ((Escape      == 0)
            && (Pause    == 0)
            && (gameover == 0)
            && (gameWon == 0)) 
          tenSecCount <= tenSecCount - 1;
        else
          tenSecCount <= tenSecCount;  
      end
    end
  end

  always @ (posedge clk100MHz)
    begin
      if((keyBoard==`Q_KEY)&&(snakemodified==0))
        begin
        snakecolormodified<=1;
        snakecolor<=snakecolor+1;
        end
      else
        begin
        snakecolormodified<=Qpressed;
        snakecolor<=snakecolor;
        end

      if((keyBoard==`W_KEY)&&(backgroundmodified==0))
        begin
        backgroundcolormodied<=1;
        backgroundcolor<=backgroundcolor+1;
        end
      else
        begin
        backgroundcolormodied<=Wpressed;
        backgroundcolor<=backgroundcolor;
        end


      if (keyBoard == `ESC_KEY)
      begin
          enableSnake <= 0;
          enableMove <= 1;
          restartGame <= 0;
          inPause <= 0;  
          inEscape<=1;
      end
      else if ((keyBoard == `P_KEY)&&(Escape==0))
      begin
          enableMove <= 0;
          enableSnake <= 1;
          restartGame <= 0;
          inPause <= 1;  
          inEscape<=0;
      end
      else if ((keyBoard == `R_KEY)&&(Pause==1)&&(Escape==0))
      begin
          enableMove <= 1;
          enableSnake <= 1;
          restartGame <= 0;
          inPause <= 0;
         inEscape<=0;
      end
      else if (keyBoard == `S_KEY)
      begin
          restartGame <= 1;
          enableSnake <= 1;
          enableMove <= 1;
          inPause <= 0;
        inEscape<=0;  
      end
      else if(Escape==1)
      begin
          enableSnake <= 0;
          enableMove <= 1;
          restartGame <= 0;
          inPause <= 0;  
          inEscape<=1;
        
      end
      else if(Pause==1)
      begin
          enableMove <= 0;
          enableSnake <= 1;
          restartGame <= 0;
          inPause <= 1;  
          inEscape<=0;
      end

      else
      begin
        restartGame <= 0;
        enableMove <= 1;
        enableSnake <= 1;
        inPause <= 0;
        inEscape<=0;
      end
    end

    assign Escape=inEscape;
    assign Pause=inPause;
    assign EN3=4'b0000;
    assign zero=4'b0000;
    assign keyBoard={sevSegData1, sevSegData0};
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


module VGA(clk100MHz,hsynch,vsynch,switch,visenable,hcount,vcount);
input clk100MHz;
input [7:0] switch;
output wire hsynch,vsynch;
wire clk25MHz;
output reg [9:0] hcount,vcount;
output visenable;

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

module colorOutGenerator (clk100MHz, visenable, rOut, gOut, bOut,
                      cornerNWx, cornerNEx, cornerSWx, cornerSEx,
                      cornerNWy, cornerNEy, cornerSWy, cornerSEy,
                      hcount, vcount,snakecolor,backgroundcolor, 
                        gameWon, gameover);
input [9:0] cornerNWx, cornerNEx, cornerSWx, cornerSEx, cornerNWy, cornerNEy, cornerSWy, cornerSEy;
input [9:0] hcount, vcount;
input clk100MHz, visenable;
input [2:0] snakecolor,backgroundcolor;
input gameWon, gameover; 

output [3:0] rOut;
output [3:0] gOut;
output [3:0] bOut;

 reg [3:0] rRegOut;
 reg [3:0] gRegOut;
 reg [3:0] bRegOut;

 reg [3:0] rValIn;
 reg [3:0] gValIn;
 reg [3:0] bValIn;

initial
begin
 rRegOut<=0;
 gRegOut<=0;
 bRegOut<=0;
 rValIn<=0;
 gValIn<=0;
 bValIn<=0;

end


always @ (hcount, vcount)
begin
  

  if (gameover)
  begin
    rValIn <= `RED_BLACK;
    gValIn <= `GREEN_BLACK;
    bValIn <= `BLUE_BLACK;
  end
  if (gameWon)
  begin
    rValIn <= `RED_ORANGE;
    gValIn <= `GREEN_ORANGE;
    bValIn <= `BLUE_ORANGE;
  end
  else if ((hcount >= cornerNWx)
  && (hcount <= cornerNEx)
  && (vcount >= cornerNEy)
  && (vcount <= cornerSEy))
  begin
  case(snakecolor)
  3'b000:begin
  rValIn <= `RED_BLUE;
  gValIn <= `GREEN_BLUE;
  bValIn <= `BLUE_BLUE;
  end
  3'b001:begin
  rValIn <= `RED_BLACK;
  gValIn <= `GREEN_BLACK;
  bValIn <= `BLUE_BLACK;
  end
  3'b010:begin
  rValIn <= `RED_GREEN;
  gValIn <= `GREEN_GREEN;
  bValIn <= `BLUE_GREEN;
  end
  3'b011:begin
  rValIn <= `RED_CYAN;
  gValIn <= `GREEN_CYAN;
  bValIn <= `BLUE_CYAN;
  end
  3'b100:begin
  rValIn <= `RED_RED;
  gValIn <= `GREEN_RED;
  bValIn <= `BLUE_RED;
  end
  3'b101:begin
  rValIn <= `RED_MAGENTA;
  gValIn <= `GREEN_MAGENTA;
  bValIn <= `BLUE_MAGENTA;
  end
  3'b110:begin
  rValIn <= `RED_YELLOW;
  gValIn <= `GREEN_YELLOW;
  bValIn <= `BLUE_YELLOW;
  end
  3'b111:begin
  rValIn <= `RED_WHITE;
  gValIn <= `GREEN_WHITE;
  bValIn <= `BLUE_WHITE;
  end
endcase

  end
  else
  begin
  case(backgroundcolor)
  3'b000:begin
  rValIn <= `RED_WHITE;
  gValIn <= `GREEN_WHITE;
  bValIn <= `BLUE_WHITE;
  end
  3'b001:begin
  rValIn <= `RED_BLACK;
  gValIn <= `GREEN_BLACK;
  bValIn <= `BLUE_BLACK;
  end
  3'b010:begin
  rValIn <= `RED_GREEN;
  gValIn <= `GREEN_GREEN;
  bValIn <= `BLUE_GREEN;
  end
  3'b011:begin
  rValIn <= `RED_CYAN;
  gValIn <= `GREEN_CYAN;
  bValIn <= `BLUE_CYAN;
  end
  3'b100:begin
  rValIn <= `RED_RED;
  gValIn <= `GREEN_RED;
  bValIn <= `BLUE_RED;
  end
  3'b101:begin
  rValIn <= `RED_MAGENTA;
  gValIn <= `GREEN_MAGENTA;
  bValIn <= `BLUE_MAGENTA;
  end
  3'b110:begin
  rValIn <= `RED_YELLOW;
  gValIn <= `GREEN_YELLOW;
  bValIn <= `BLUE_YELLOW;
  end
  3'b111:begin
  rValIn <= `RED_BLUE;
  gValIn <= `GREEN_BLUE;
  bValIn <= `BLUE_BLUE;
  end
endcase
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

module clock50Hz(clk100MHz,clk50Hz);
  input clk100MHz;
  output clk50Hz;
  reg [20:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk50Hz= count[20];

endmodule

module clock100Hz(clk100MHz,clk100Hz);
  input clk100MHz;
  output clk100Hz;
  reg [19:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk100Hz= count[19];
endmodule

module clock200Hz(clk100MHz,clk200Hz);
  input clk100MHz;
  output clk200Hz;
  reg [18:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk200Hz= count[18];
endmodule

module clock400Hz(clk100MHz,clk400Hz);
  input clk100MHz;
  output clk400Hz;
  reg [17:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk400Hz= count[17];
endmodule

module clock800Hz(clk100MHz,clk800Hz);
  input clk100MHz;
  output clk800Hz;
  reg [16:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk800Hz= count[16];
endmodule

module clock1600Hz(clk100MHz,clk1600Hz);
  input clk100MHz;
  output clk1600Hz;
  reg [15:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk1600Hz= count[15];
endmodule

module clock25Hz(clk100MHz,clk25Hz);
  input clk100MHz;
  output clk25Hz;
  reg [21:0] count;

  initial
  begin
  count<=0;
  end

  always@(posedge clk100MHz)
  begin
  count<=count+1;
  end
  assign clk25Hz= count[21];

endmodule

/*************** SNAKE MODULES ***************/
module topSnake (clk50Hz,cornerNWx, cornerNEx, cornerSWx, cornerSEx,cornerNWy, cornerNEy, cornerSWy, cornerSEy,
              keyBoard, enableMove, gameover, enableSnake, restartSnake, gameWon);
input [7:0] keyBoard;
input clk50Hz;
//input [2:0] gamestate;
output [9:0] cornerNWx, cornerNEx, cornerSWx, cornerSEx,cornerNWy, cornerNEy, cornerSWy, cornerSEy;
wire [9:0] headX,headY;
wire [1:0] direction;
input wire restartSnake;
input wire enableMove;
output wire gameover;
input enableSnake;
input gameWon;

buildsnake build(headX,headY,direction,cornerNWx, cornerNEx, cornerSWx, cornerSEx,cornerNWy, cornerNEy, cornerSWy, cornerSEy, enableSnake);
positionSnake position(direction,headX,headY,clk50Hz,keyBoard, enableMove, gameover, restartSnake, enableSnake);

endmodule



module buildsnake(headx,heady,direction,
                cornerNWx, cornerNEx, cornerSWx, cornerSEx,
                cornerNWy, cornerNEy, cornerSWy, cornerSEy,
                enableSnake);
input [9:0] headx, heady;
input [1:0] direction;
output reg [9:0] cornerNWx,cornerNEx,cornerSWx,cornerSEx;
output reg [9:0] cornerNWy,cornerNEy,cornerSWy,cornerSEy;
input enableSnake;

initial
begin
      cornerNWx <= 0;
      cornerNEx <= 0;
      cornerSWx <= 0;
      cornerSEx <= 0;
      cornerNWy <= 0;
      cornerNEy <= 0;
      cornerSWy <= 0;
      cornerSEy <= 0;
end


always@(headx,heady,direction, enableSnake)
begin
  // creates corners based on head coord and direction
  if (enableSnake == 0)
  begin
      cornerNWx <= 700;
      cornerNEx <= 700;
      cornerSWx <= 700;
      cornerSEx <= 700;
      cornerNWy <= 700;
      cornerNEy <= 700;
      cornerSWy <= 700;
      cornerSEy <= 700;
  end
  else
  begin
    case (direction)
        `LEFT:
    begin
    cornerNWx <= headx;
    cornerNEx <= headx + 39;
    cornerSWx <= headx;
    cornerSEx <= headx + 39;
    cornerNWy <= heady - 5;
    cornerNEy <= heady - 5;
    cornerSWy <= heady + 4;
    cornerSEy <= heady + 4;
    
    end
        `RIGHT:
    begin
          cornerNWx <= headx - 39;
          cornerNEx <= headx;
          cornerSWx <= headx - 39;
          cornerSEx <= headx;
          cornerNWy <= heady - 5;
          cornerNEy <= heady - 5;
          cornerSWy <= heady + 4;
          cornerSEy <= heady + 4;
    
    end
        `UP:
      begin
          cornerNWx <= headx - 5;
          cornerNEx <= headx + 4;
          cornerSWx <= headx - 5;
          cornerSEx <= headx + 4;
          cornerNWy <= heady;
          cornerNEy <= heady;
          cornerSWy <= heady + 39;
          cornerSEy <= heady + 39;
    
    end
        `DOWN:
    begin
          cornerNWx <= headx - 5;
          cornerNEx <= headx + 4;
          cornerSWx <= headx - 5;
          cornerSEx <= headx + 4;
          cornerNWy <= heady - 39;
          cornerNEy <= heady - 39;
          cornerSWy <= heady;
          cornerSEy <= heady;
    
    end
    
    default:
    begin
      cornerNWx <= cornerNWx;
      cornerNEx <= cornerNEx;
      cornerSWx <= cornerSWx;
      cornerSEx <= cornerSEx;
      cornerNWy <= cornerNWy;
      cornerNEy <= cornerNEy;
      cornerSWy <= cornerSWy;
      cornerSEy <= cornerSEy;
    end
    endcase
  end
end


endmodule


module positionSnake(direction,headXOut,headYOut,clk50Hz,keyBoard,
                  enableMove, gameover, restart, enableSnake);
input clk50Hz;
input [7:0] keyBoard;
input wire enableMove;
input wire enableSnake;

input wire restart;
output reg [1:0] direction;
output reg [9:0] headXOut, headYOut;
output reg gameover;


initial
begin
  direction <= `RIGHT;
  headXOut <= 0;
  headYOut <= 0;
end

always @ (posedge clk50Hz)
begin

  if ((restart == 1) || (enableSnake == 0))
  begin
    headXOut <= 39;
    headYOut <= 40;
    direction <= `RIGHT;
    gameover <= 0;
  end
  else if (enableMove == 0)
  begin
  headXOut <= headXOut;
  headYOut <= headYOut;
  direction <= direction;
  gameover <= 0;
  end
  else if ((headXOut == 0 && direction == `LEFT)
    || (headYOut == 0 && direction == `UP)
    || (headXOut == 639 && direction == `RIGHT)
    || (headYOut == 479 && direction == `DOWN))
  begin
  headXOut <= headXOut;
  headYOut <= headYOut;
  direction <= direction;
  gameover <= 1;
  end
  else if ((headXOut >= 640)
        || (headYOut >= 480))
  begin
  gameover <= 1;
  case (direction)
  `UP:
  begin
      headYOut <= 0;
      headXOut <= headXOut;
  end
  `DOWN:
  begin
      headYOut <= 479;
      headXOut <= headXOut;
  end
  `LEFT:
  begin
      headXOut <= 0;
      headYOut <= headYOut;
  end
  `RIGHT:
  begin
      headXOut <= 639;
      headYOut <= headYOut;
  end
  endcase
  direction <= direction;
  end
 else
 begin
  case(direction)
  `LEFT:
    begin
    if (keyBoard == `DOWN_PRESS)
      // turn left
      begin
        direction <= `DOWN;
        headXOut <= headXOut + 5;
        headYOut <= headYOut + 34;
        gameover <= 0;
      end
      else if (keyBoard == `UP_PRESS)
      // turn right
      begin
      direction <= `UP;
        headXOut <= headXOut + 5;
        headYOut <= headYOut - 34;
        gameover <= 0;
      end
      else
      // stay straight: increment position
      begin
        headXOut <= headXOut - 1;
        headYOut <= headYOut;
        direction<= direction;
        gameover <= 0;
      end
    end
  `RIGHT:
    begin
    if (keyBoard == `DOWN_PRESS)
      // turn right
      begin
        direction <= `DOWN;
        headXOut <= headXOut - 4;
        headYOut <= headYOut + 34;
        gameover <= 0;
      end
      else if (keyBoard == `UP_PRESS)
      // turn left
      begin
        direction <= `UP;
        headXOut <= headXOut - 4;
        headYOut <= headYOut - 34;
        gameover <= 0;
      end
      else
      // stay straight: increment position
      begin
        headXOut <= headXOut + 1;
        headYOut <= headYOut;
        direction<= direction;
        gameover <= 0;
      end
    end
  `UP:
    begin
    if (keyBoard == `LEFT_PRESS)
      // turn left
      begin
      direction <= `LEFT;
        headXOut <= headXOut - 35;
        headYOut <= headYOut + 5;
        gameover <= 0;
      end
      else if (keyBoard == `RIGHT_PRESS)
      // turn right
      begin
      direction <= `RIGHT;
        headXOut <= headXOut + 34;
        headYOut <= headYOut + 5;
        gameover <= 0;
      end
      else
      // stay straight: increment position
      begin
        headXOut <= headXOut;  
        headYOut <= headYOut - 1;
        direction<= direction;
        gameover <= 0;
      end
    end
  `DOWN:
    begin
    if (keyBoard == `LEFT_PRESS)
      // turn right
      begin
      direction <= `LEFT;
        headXOut <= headXOut - 35;
        headYOut <= headYOut - 4;
        gameover <= 0;
      end
    else if (keyBoard == `RIGHT_PRESS)
      // turn left
      begin
        direction <= `RIGHT;
        headXOut <= headXOut + 34;
        headYOut <= headYOut - 4;
        gameover <= 0;
      end
    else
    begin
      // stay straight: increment position
      headXOut <= headXOut;
      headYOut <= headYOut + 1;
      direction<= direction;
      gameover <= 0;
    end
    end
    default:
    begin
      headXOut <= headXOut;
      headYOut <= headYOut;
      direction <= direction;
      gameover <= 0;
    end
  endcase
  end
end

endmodule



