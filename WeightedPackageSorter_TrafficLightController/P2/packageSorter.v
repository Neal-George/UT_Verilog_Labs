// update: 0 weight must be measured before group count can be updated: reset does not
//         allow group to be updated
module packageSorter (weight, clk, reset, grp1, grp2, grp3, grp4, grp5, grp6, currentGroup); 
  
  input clk, reset; 
  input [11:0] weight; 
  
  output reg [7:0] grp1, grp2, grp3, grp4, grp5, grp6; 
  output reg [2:0] currentGroup;


  reg hasBeenWeighed; 


  initial 
  begin 
    currentGroup = 0; 
    hasBeenWeighed = 0;
    grp1 <= 0; 
    grp2 <= 0; 
    grp3 <= 0; 
    grp4 <= 0; 
    grp5 <= 0; 
    grp6 <= 0; 
  end 


  always @ (*)
  begin 
    if (weight == 0)
    begin
      currentGroup <= 0;
      hasBeenWeighed <= 0; 
    end
    else if (weight <= 250)
      currentGroup <= 1; 
    else if (weight <= 500)
      currentGroup <= 2; 
    else if (weight <= 750)
      currentGroup <= 3; 
    else if (weight <= 1500)
      currentGroup <= 4; 
    else if (weight <= 2000)
      currentGroup <= 5; 
    else 
      currentGroup <= 6; 
  end


  always @ (negedge clk, posedge reset)
  begin 
    if (reset == 1) // if (reset == 0) then do the rest of the block? 
    begin
      grp1 <= 0;
      grp2 <= 0; 
      grp3 <= 0; 
      grp4 <= 0; 
      grp5 <= 0; 
      grp6 <= 0; 
    end 
    else if (weight == 0)
      hasBeenWeighed <= 0; 
    else if (hasBeenWeighed == 0)
    begin
      hasBeenWeighed <= 1; 
      if (weight <= 250)
        grp1 <= grp1 + 1'b1; 
      else if (weight <= 500)
        grp2 <= grp2 + 1'b1;  
      else if (weight <= 750)
        grp3 <= grp3 + 1'b1;  
      else if (weight <= 1500)
        grp4 <= grp4 + 1'b1;  
      else if (weight <= 2000)
        grp5 <= grp5 + 1'b1;  
      else 
        grp6 <= grp6 + 1'b1;
    end
    else
    begin
      grp1 <= grp1; 
      grp2 <= grp2; 
      grp3 <= grp3; 
      grp4 <= grp4; 
      grp5 <= grp5; 
      grp6 <= grp6; 
    end
  end
endmodule