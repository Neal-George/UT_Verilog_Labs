module pSortTB;


parameter NUMCASES = 17;
parameter NUMGROUPS = 6;
reg reset;
reg[11:0] weight;
reg CLK;


wire [7:0] grp[1:NUMGROUPS];
wire [2:0] currentGrp;


reg        resetArr[1:NUMCASES];
reg [11:0] weightArr[1:NUMCASES];
reg [7:0]  grpArr[1:NUMGROUPS][1:NUMCASES];
reg [2:0]  currentGrpArr[1:NUMCASES];  


always
begin
  #5 CLK <= ~CLK;
end


initial
begin
  // reg inits
  weight <= 270;
  reset <= 1;  
  CLK <= 0;


  // resetArr
  resetArr[1] <= 1;
  resetArr[2] <= 0;
  resetArr[3] <= 0;
  resetArr[4] <= 0;
  resetArr[5] <= 0;
  resetArr[6] <= 0;
  resetArr[7] <= 0;
  resetArr[8] <= 0;
  resetArr[9] <= 0;
  resetArr[10]<= 0;
  resetArr[11]<= 0;
  resetArr[12]<= 0;
  resetArr[13]<= 0;
  resetArr[14]<= 0;
  resetArr[15]<= 0;
  resetArr[16]<= 1; 
  resetArr[17]<= 0; 






  // weightArr
  weightArr[1] <= 270;
  weightArr[2] <= 270;
  weightArr[3] <= 0;
  weightArr[4] <= 300;
  weightArr[5] <= 0;
  weightArr[6] <= 501;
  weightArr[7] <= 1013;
  weightArr[8] <=0;
  weightArr[9] <= 2015;
  weightArr[10] <= 0;
  weightArr[11] <= 1700;
  weightArr[12] <=0;
  weightArr[13]<= 69;
  weightArr[14]<= 0;
  weightArr[15]<= 777;
  weightArr[16]<= 0;
  weightArr[17]<= 777;


  // grpArr
  // case 1
  grpArr[1][1] <= 0;
  grpArr[2][1] <= 0;
  grpArr[3][1] <= 0;
  grpArr[4][1] <= 0;
  grpArr[5][1] <= 0;
  grpArr[6][1] <= 0;
  // case 2
  grpArr[1][2] <= 0;
  grpArr[2][2] <= 1;
  grpArr[3][2] <= 0;
  grpArr[4][2] <= 0;
  grpArr[5][2] <= 0;
  grpArr[6][2] <= 0;
  // case 3
  grpArr[1][3] <= 0;
  grpArr[2][3] <= 1;
  grpArr[3][3] <= 0;
  grpArr[4][3] <= 0;
  grpArr[5][3] <= 0;
  grpArr[6][3] <= 0;
  // case 4
  grpArr[1][4] <= 0;
  grpArr[2][4] <= 2;
  grpArr[3][4] <= 0;
  grpArr[4][4] <= 0;
  grpArr[5][4] <= 0;
  grpArr[6][4] <= 0;
  // case 5
  grpArr[1][5] <= 0;
  grpArr[2][5] <= 2;
  grpArr[3][5] <= 0;
  grpArr[4][5] <= 0;
  grpArr[5][5] <= 0;
  grpArr[6][5] <= 0;
  // case 6
  grpArr[1][6] <= 0;
  grpArr[2][6] <= 2;
  grpArr[3][6] <= 1;
  grpArr[4][6] <= 0;
  grpArr[5][6] <= 0;
  grpArr[6][6] <= 0;
  // case 7
  grpArr[1][7] <= 0;
  grpArr[2][7] <= 2;
  grpArr[3][7] <= 1;
  grpArr[4][7] <= 0;
  grpArr[5][7] <= 0;
  grpArr[6][7] <= 0;
  // case 8
  grpArr[1][8] <= 0;
  grpArr[2][8] <= 2;
  grpArr[3][8] <= 1;
  grpArr[4][8] <= 0;
  grpArr[5][8] <= 0;
  grpArr[6][8] <= 0;
  // case 9
  grpArr[1][9] <= 0;
  grpArr[2][9] <= 2;
  grpArr[3][9] <= 1;
  grpArr[4][9] <= 0;
  grpArr[5][9] <= 0;
  grpArr[6][9] <= 1;
  // case 10
  grpArr[1][10] <= 0;
  grpArr[2][10] <= 2;
  grpArr[3][10] <= 1;
  grpArr[4][10] <= 0;
  grpArr[5][10] <= 0;
  grpArr[6][10] <= 1;
  // case 11
  grpArr[1][11] <= 0;
  grpArr[2][11] <= 2;
  grpArr[3][11] <= 1;
  grpArr[4][11] <= 0;
  grpArr[5][11] <= 1;
  grpArr[6][11] <= 1;
  // case 12
  grpArr[1][12] <= 0;
  grpArr[2][12] <= 2;
  grpArr[3][12] <= 1;
  grpArr[4][12] <= 0;
  grpArr[5][12] <= 1;
  grpArr[6][12] <= 1;
  // case 13
  grpArr[1][13] <= 1;
  grpArr[2][13] <= 2;
  grpArr[3][13] <= 1;
  grpArr[4][13] <= 0;
  grpArr[5][13] <= 1;
  grpArr[6][13] <= 1;
  // case 14
  grpArr[1][14] <= 1;
  grpArr[2][14] <= 2;
  grpArr[3][14] <= 1;
  grpArr[4][14] <= 0;
  grpArr[5][14] <= 1;
  grpArr[6][14] <= 1;
  // case 15
  grpArr[1][15] <= 1;
  grpArr[2][15] <= 2;
  grpArr[3][15] <= 1;
  grpArr[4][15] <= 1;
  grpArr[5][15] <= 1;
  grpArr[6][15] <= 1;
  // case 16
  grpArr[1][16] <= 0;
  grpArr[2][16] <= 0;
  grpArr[3][16] <= 0;
  grpArr[4][16] <= 0;
  grpArr[5][16] <= 0;
  grpArr[6][16] <= 0;
  // case 17
  grpArr[1][17] <= 0;
  grpArr[2][17] <= 0;
  grpArr[3][17] <= 0;
  grpArr[4][17] <= 4;
  grpArr[5][17] <= 0;
  grpArr[6][17] <= 0;




  // currentGrpArr
  currentGrpArr[1] <= 2;
  currentGrpArr[2] <= 2;
  currentGrpArr[3] <= 0;
  currentGrpArr[4] <= 2;
  currentGrpArr[5] <= 0;
  currentGrpArr[6] <= 3;
  currentGrpArr[7] <= 4;
  currentGrpArr[8] <= 0;
  currentGrpArr[9] <= 6;
  currentGrpArr[10] <= 0;
  currentGrpArr[11] <= 5;
  currentGrpArr[12] <= 0;
  currentGrpArr[13] <= 1;
  currentGrpArr[14] <= 0;
  currentGrpArr[15] <= 4;
  currentGrpArr[16] <= 0;
  currentGrpArr[17] <= 4;



end


integer i;
integer j;
always
begin
  for (i = 1; i <= NUMCASES; i = i + 1)
  begin
        $display("Case %d", i);
    
        // set inputs at rising edge (assuming clock initially 0)
        wait(CLK == 1)
        weight <= weightArr[i];
        reset  <= resetArr[i];


        // read outputs after falling edge
        wait(CLK == 0)
    
        #(1); // stabalize outputs


        // check all group outputs
        for (j = 1; j <= NUMGROUPS; j = j + 1)
        begin
          if (grp[j] != grpArr[j][i])
          begin
            $display("Test failed for group%d, test case %d.", j, i);  
          end
        end


        // check current group output
        if (!(currentGrp == currentGrpArr[i]))
        begin
          $display("Test failed for current group, test case %d", i);
        end


  end // end for loop
  $display("Test complete.") ;
end


packageSorter pS1(weight, CLK, reset, grp[1], grp[2], grp[3], grp[4], grp[5], grp[6], currentGrp);


endmodule