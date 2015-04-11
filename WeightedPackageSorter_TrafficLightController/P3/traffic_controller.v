`define S_RED    4
`define S_YELLOW 2
`define S_GREEN  1
`define S_OFF    0


`define W_RED   2
`define W_GREEN 1
`define W_OFF   0




module traffic_controller(RST, CLK, lighta, lightb, lightw);






  input RST, CLK;
  output lighta, lightb, lightw;
  
  reg rstCount; 
  reg [3:0] state,nextstate;
  reg [3:0] delay,newdelay;
  reg [2:0] lighta,lightb; //red,yellow,green
  reg [1:0] lightw; //red,green


  initial
  begin
    state <= 0;
    nextstate <= 0;
    delay <= 1;
    newdelay <=15; 
    lighta <= `S_GREEN;
    lightb <= `S_RED;
    lightw <= `W_RED;
  end








  always @(negedge CLK, posedge RST)
  begin
    if (RST == 0)
    begin
      rstCount <= 0; 
    end
    else
    begin
      rstCount <= 1; 
    end


    if ( (rstCount == 0) && (RST == 1) )
    begin
      state <= 13; 
      newdelay <= 15; 
    end 
    else 
    begin
      if (delay == 1)
      begin
        state <= nextstate;
        newdelay<=15;
      end
      
      else if (delay<=newdelay)
        begin
          newdelay<=delay-1;
          state<=state;
        end


    else if (newdelay <= 1)
      begin
        state <= nextstate;
        newdelay<=15;
      end
      else begin
        state <= state;
        newdelay<=newdelay-1;
      end
    end
  end




  always @ (state) // should we include RST? Probably 
  begin
   
    case(state)


    0: begin
      lighta = `S_GREEN;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 12;
        nextstate = 1; 
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    1: begin
      lighta = `S_YELLOW;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 8;
        nextstate = 2;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end


    end
    2: begin
      lighta = `S_RED;
      lightb = `S_GREEN;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 12;
        nextstate = 3;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    3: begin
      lighta = `S_RED;
      lightb = `S_YELLOW;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 4;
        nextstate = 4;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    4: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_GREEN;
      if (~RST)
      begin
        delay = 8;
        nextstate = 5;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    5: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 1;
        nextstate = 6;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    6: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_OFF;
      if (~RST)
      begin
        delay = 1;
        nextstate = 7;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    7: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 1;
        nextstate = 8;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    8: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_OFF;
      if (~RST)
      begin
        delay = 1;
        nextstate = 9;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    9: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 1;
        nextstate = 10;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    10: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_OFF;
      if (~RST)
      begin
        delay = 1;
        nextstate = 11;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    11: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_RED;
      if (~RST)
      begin
        delay = 1;
        nextstate = 12;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    12: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_OFF;
      if (~RST)
      begin
        delay = 1;
        nextstate = 0;
      end
      else
      begin
        nextstate = 13;
        delay = 1;
      end
    end


    13: begin
      lighta = `S_RED;
      lightb = `S_RED;
      lightw = `W_RED;
      if (RST)
      begin
        delay = 2;
        nextstate = 14;
      end
      else
      begin
        nextstate = 0;
        delay = 1;
      end
    end


    14: begin
      lighta = `S_OFF;
      lightb = `S_OFF;
      lightw = `W_OFF;
      if(RST)
      begin
        delay = 2;
        nextstate = 13;
      end
      else
      begin
        nextstate = 0;
        delay = 1;
      end
    end


    
    default:begin
      lighta = 7;
      lightb = 7;
      lightw = 3;
      nextstate = 1;
      delay = 1;
      
    end
    
  endcase
  end


endmodule




module divider(clk100Mhz, slowClk);
  
  input clk100Mhz; //fast clock
  output slowClk; //slow clock
  
  reg [27:0] counter;
  
  // switch to 27 for visible division
  assign slowClk = counter[24]; //(2^27 / 100E6) = 1.34seconds
  
  initial
  begin
    counter <= 0;
  end


  always @ (posedge clk100Mhz)
  begin
    counter <= counter + 1; //increment the counter every 10ns (1/100 Mhz) cycle.
  end


endmodule




module top (RST,lighta,lightb,lightw,clk100Mhz);
  
  input RST,clk100Mhz;
  output lighta, lightb, lightw;
  wire slowClk;
  wire [2:0] lighta,lightb;
  wire [1:0] lightw;


  divider divide(clk100Mhz, slowClk);
  traffic_controller traffic(RST,slowClk,lighta,lightb,lightw);


endmodule