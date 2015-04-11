

module BCD_ADDER(X,Y,Z);
  input [15:0] X,Y;
  output [15:0] Z;
  wire [4:0] S0,S1,S2,S3;
  wire [15:0] tempZ;
  wire C1,C2,C3;

  assign S0= X[3:0]+Y[3:0];
  assign tempZ[3:0]= (S0>9)?S0[3:0]+6: S0[3:0];
  assign C1=(S0>9)? 1'b1:1'b0;

  assign S1=X[7:4]+Y[7:4]+C1;
  assign tempZ[7:4]=(S1>9)?S1[3:0]+6:S1[3:0];
  assign C2=(S1>9)? 1'b1:1'b0;

  assign S2=X[11:8]+Y[11:8]+C2;
  assign tempZ[11:8]=(S2>9)?S2[3:0]+6:S2[3:0];
  assign C3=(S2>9)? 1'b1:1'b0;


  assign S3=X[15:12]+Y[15:12]+C3;

  assign Z[15:0]=(S3>9)? 16'b1001100110011001: {S3[3:0],tempZ[11:0]};

endmodule


module BCD_Decrementer(Z,Decremented);
  input [15:0] Z;
  output [15:0] Decremented;
  reg [15:0] Decremented;
  
  always @ (Z)
  begin
    if(Z==0)
      Decremented <= 0;
    else if (Z[3:0]>0 )
      Decremented <= {Z[15:4],(Z[3:0]-1'b1)};
    else if (Z[7:4]>0 )
      Decremented <= {Z[15:8],(Z[7:4]-1'b1),4'b1001};
    else if (Z[11:8]>0)
      Decremented <= {Z[15:12],(Z[11:8]-1'b1),8'b10011001};
    else if( Z>39321 )
      Decremented <= 16'b1001100110011001;
    else
      Decremented <= {Z[15:12]-1'b1,12'b100110011001};
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



module decDivider(clk100MHz, slowClk);
 
  input clk100MHz; //fast clock
  output slowClk;  //slow clock
  reg slowClk;
  reg [26:0] counter;
 
  // switch to 27 for visible division
  //assign slowClk = & counter; //(2^27 / 100E6) = 1.34seconds
  //assign slowClk = counter[26];
  //assign slowClk = ! (|counter); 
  //assign slowClk = (counter && 27'b000110000000110000001100000); 
  initial
  begin
  counter <= 0;
  end

  always @ (posedge clk100MHz)
  begin
    counter <= counter + 1; //increment the counter every 10ns (1/100 Mhz) cycle.
    if (counter == 0)
      slowClk = 1; 
    else 
      slowClk = 0; 
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



// for flashing light at 0
module flashclock(clk100MHz, slowClk);
 
  input clk100MHz; //fast clock
  output slowClk; //slow clock
 
  reg [25:0] counter;
 
  // switch to 27 for visible division
  assign slowClk = counter[25]; //(2^27 / 100E6) = 1.34seconds
 
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



// used for initial debugging of BCD circuit
module switching(clk,switch,inputval);
  input clk, switch;
  output [15:0] inputval;
  reg[15:0] inputval;
    
  always @(posedge clk)
  begin
  if(switch)
  begin
  inputval<=16'b0000001001010000;
  end
  else
  inputval<=0;
  end

endmodule



module flash(decrementer, EN);
  input [15:0] decrementer;
  output [3:0] EN;
  reg [3:0] EN;
    
  always@(decrementer)
  begin
  if(decrementer)
    begin
    if(decrementer<512)
    begin
    if(decrementer[0])
    EN=4'b1111;
    else
    EN=4'b0000;
    end
    else
    EN=4'b0000;
    end
  else
  EN=4'b0000;   //~EN;
  end

endmodule // end flash


module flashzero(clk,EN);
  input clk;
  output [3:0] EN;
  reg [3:0] EN;
  always@(clk)
  begin
  if(clk==1)
  EN<=4'b1111;
  else
  EN<=4'b0000;
  end

endmodule // end flashzero



module top (clk100MHz,digit_enable,segment,DP,
        BU, BL, BR, BD, s0, s1);
 
  input clk100MHz, BU, BL, BR, BD, s0, s1;
  output DP,digit_enable,segment;
  wire slowDecClk;
  wire fast7SegCLK;
  wire [3:0] BCD;
  wire [15:0] Decremented;
  wire [15:0] outputSeg;
  wire [15:0] selDecSw;
  wire [6:0] segment;
  wire [15:0] Z;
  wire [3:0] digit_enable;
  wire [3:0] EN1,EN2,EN3;
  wire flashclk;
  wire buo, blo, bro, bdo, s0o, s1o;
  // wire buoM, bloM, broM, bdoM, s0oM, s1oM;
  reg addSel;
  reg [15:0] X;
  wire inputSinglePulse;
  wire s1oDB, s0oDB; 

  assign inputSinglePulse = buo || blo || bro || bdo || s0o || s1o;

  // output single pulse for each input at clk100MHz
  topInput debounce(BU, BL, BR, BD, s0, s1, clk100MHz,
                buo, blo, bro, bdo, s0o, s1o, s0oDB, s1oDB);

  // need the single pulse to last at least as
  // long as the period of the decrementer
  freqMul fm1 (buo, clk100MHz, buoM);
  freqMul fm2 (blo, clk100MHz, bloM);
  freqMul fm3 (bro, clk100MHz, broM);
  freqMul fm4 (bdo, clk100MHz, bdoM);
  freqMul fm5 (s0o, clk100MHz, s0oM);
  freqMul fm6 (s1o, clk100MHz, s1oM);
 
  // assign inputs to binary adder
  // if using freqMul's, use their output <name>
  always @ (buoM, bloM, broM, bdoM, s0oDB, s1oDB)
  begin
  if (s0oDB != 0) // debounced switch
  begin
  X <= 16'b0000000000010000; // 10
  addSel <= 1;
  end
  else if (s1oDB != 0) // debounced switch
  begin
  X <= 16'b0000001000000101; // 205
  addSel <= 1;
  end
  else if (buoM != 0) // buoM
  begin
  X <= 16'b0000000001010000; // 50
  addSel <= 0;
  end
  else if (bloM != 0) // bloM
  begin
  X <= 16'b0000000101010000; // 150
  addSel <= 0;
  end
  else if (broM != 0) // broM
  begin
  X <= 16'b0000001000000000; // 200
  addSel <= 0;
  end
  else if (bdoM != 0) // bdoM
  begin
  X <= 16'b0000010100000000; // 500
  addSel <= 0;
  end
  else // default
  begin
  X <= 0;
  addSel <= 0;
  end
  end // end always @ (*)

  // either the button adds time to the decremeneted time, or a switch
  // is reseting the value: i.e. adding time to 0
  assign selDecSw = (addSel == 0) ? (Decremented) : (16'b0000000000000000);
 
  // clock to flash is only used for when timer is at 0
  // EN1 is asserted at time 0, and flashes at 1Hz, 50% duty cycle
  flash  flashing(outputSeg,EN1);
  flashzero fz(flashclk,EN2);
  flashclock fc (clk100MHz, flashclk);
 
  // used for initial debugging of BCD circuit
  // switching init(fast7SegCLK,switch,X);
 
  // fast7SegClk is used to alternate the 7 segment display output enables to make them all visible at once
  seven_segment_mux  mux(fast7SegCLK,outputSeg[3:0],outputSeg[7:4],outputSeg[11:8],outputSeg[15:12],BCD,digit_enable,DP,EN3);
  speedup fast(clk100MHz, fast7SegCLK);
  seven_segment display(segment,BCD);
 
  // slowDecClk is used by decrementer to decrement output every second
  decDivider div(clk100MHz, slowDecClk);
  BCD_Decrementer sub(outputSeg,Decremented); // Z[16] in, Z[16] - 1 out
 
  // output either adds X (input) to what was last decremented(button) or 0(switch)
  BCD_ADDER add(X,selDecSw,Z);        // Z <= X + Decremented?0

  asynchOutput aO1(Z, slowDecClk, inputSinglePulse, outputSeg, clk100MHz);

  // EN3 enables 7 seg display during proper flashing times
  assign EN3=(outputSeg==0)?EN2:EN1;
 
endmodule


module asynchOutput (adderIn, decClk, inSig, to7Seg, clk100MHz);
  input [15:0] adderIn;
  input clk100MHz;
  input decClk, inSig;
  output [15:0] to7Seg;
  wire pulse;
  reg [15:0] to7Seg;
  wire inclk; 

  freqMulSmallPulse f1 (inSig, pulse, clk100MHz);

  // after input single pulse is seen, we want to 
  // output the result from the adder. we do this by 
  // using freqMulSmallPulse 
  // update output on poedge of decClk with decrementer  
 
  assign inclk = decClk || pulse; 
  always @ (posedge inclk)
  begin
    to7Seg <= adderIn;  
  end

endmodule


// insig needs to be single pulse
module freqMulSmallPulse (insig, outSig, clk100MHz);
  input insig, clk100MHz;
  reg [3:0] outpulse;
  output wire outSig; 
  // wire inClk; 

  assign outSig = outpulse[3]; 

  // assign inClk = clk100MHz || insig; 
  
  always @ (posedge clk100MHz, posedge insig)
  begin
    if (insig != 0)
      outpulse <= 4'b0001;
    else if (outpulse != 0)
      outpulse <= outpulse + 4'b0001;
    else
      outpulse <= 0;  
  end

endmodule


// cause pulse in to be output for the same amount of time
// as the divider period
module freqMul (pulseIn, clk100MHz, pulse1SecOut);
  input pulseIn, clk100MHz;
  output pulse1SecOut;
  reg [4:0] counter; // needs to be one more than that number of bits in freqMulSmallPulse
  // wire inClk; 

  assign pulse1SecOut = counter [4];
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


module topInput (BU,  BL,  BR,  BD,  s0,  s1, clk100MHz,
            BUo, BLo, BRo, BDo, s0oPU, s1oPU, s0oDB, s1oDB);
  input wire BU, BL, BR, BD, s0, s1, clk100MHz;
  output wire BUo, BLo, BRo, BDo, s0oDB, s1oDB, s1oPU, s0oPU;  
  wire clk10Hz;
  wire outDB1, outDB2, outDB3, outDB4, outDB5,outDB6;

  // wire clkDEBUG;

  divider10Hz d1(clk100MHz, clk10Hz);
  // dividerDEBUGHz (clk100MHz, clkDEBUG);

  // single pulse is synchronous with clk100MHz
  synchCircuit sC1(BU, clk100MHz, clk10Hz, BUo, outDB1); // changed clk100MHz to clkDEBUG
  synchCircuit sC2(BL, clk100MHz, clk10Hz, BLo, outDB2);
  synchCircuit sC3(BR, clk100MHz, clk10Hz, BRo, outDB3);
  synchCircuit sC4(BD, clk100MHz, clk10Hz, BDo, outDB4);
  // switches need to output their debounced values
  synchCircuit sC5(s1, clk100MHz, clk10Hz, s1oPU, s1oDB);
  synchCircuit sC6(s0, clk100MHz, clk10Hz, s0oPU, s0oDB);  


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



module synchCircuit(switch, clk100MHz, clk10Hz, synchPulse, synchOut);
  input wire switch, clk100MHz, clk10Hz;
  output wire synchPulse, synchOut; // synchOut is the debounced output from second DFF
  wire s1, s0;
  wire q1, q1n, synchOutN;

  DFF d1(switch,   clk10Hz, q1,   q1n);   // clock period is max debounce time
  DFF d2(q1,  clk10Hz,  synchOut, SynchOutN); // clock period is max debounce time
  DFF d3(synchOut, clk100MHz,  s1,  s0);    // clock period is what outer module
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


