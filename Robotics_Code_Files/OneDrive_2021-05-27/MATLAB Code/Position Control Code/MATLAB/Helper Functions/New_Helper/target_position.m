%This function outputs the target position according to the given target
%row and column numbers

function [output_ma,offset] = target_position(column,row,type)
    z = 0;
    one = 26.2;
    if row == 1
        x = one;
    elseif row == 11
        x = one+2; 
    elseif row == 12
        x = one+1.5;
     elseif row == 13
        x = one+5;
    elseif row == 2
        x = one+7; 
    elseif row == 27
        x = one+6-1; 
    elseif row == 21
        x = one+6; 
    elseif row == 3
        x = one+4*2+1;
    elseif row == 31
        x = one+4*2;
    elseif row == 4
        x = one+4*3-3;
    elseif row == 5
        x =one+4*4 - 3;
    elseif row == 6
        x = one+4*5;
    elseif row == 7
        x = one+4*6;
    elseif row == 8
        x = one+4*7;
    elseif row == 51
         x =one+4*4+1;
     elseif row == 55
         x =one+4*4-1;
    elseif row == 52
        x = one+4*4+1.5; 
    elseif row == 61
         x = one+4*5 - 1;
    elseif row == 41
        x = one+4*3-1;
    elseif row == 53
        x = one+4*4; 
      elseif row == 54
        x = one+4*4+1.5; 
    elseif row == 42
        x = one+4*3+1;
     elseif row == 44
        x = one+4*3-2;
            
    end
    if column == "H"
        y = 13.1+1;
    elseif column == "G"
        y = 9.1;
    elseif column == "F"
        y = 5.1;
    elseif column == "E"
        y = 1.1+0.5;
    elseif column == "E1"
        y = 1.1+1;
    elseif column == "D"
        y = 1.1 - 2.5;
    elseif column == "D5"
        y = 1.1 -6.5;
    elseif column == "D1"
        y = 1.1 - 2.5-1;
    elseif column == "D4"
        y = 1.1 - 2.5-3;
    elseif column == "C"
        y = 1.1 - 4*2+1;
    elseif column == "B"
        y = 1.1 - 4*3-3;
    elseif column == "B2"
        y = 1.1 - 4 * 3;
    elseif column == "A"
        y = 1.1 - 4*4;
    elseif column == "F2"
        y = 5.1 -2;
    elseif column == "D2"
        y = 1.1 - 4 -3;
    elseif column == "D3"
        y = 1.1 - 4 -1.5;
    elseif column == "C2"
        y = 1.1 - 4*2-2;
    elseif column == "capture_pool"
        y = -30;
        x = 20;
        z = 0;
        offset = 0;
    elseif column == "F1"
        y = 5.1+2;
    elseif column == "F3"
        y = 5.1+0.5;
    elseif column == "H1"
        y = 13.1+2;
    elseif column == "H2"
        y = 13.1+1;
    elseif column == "G1"
        y = 9.1+1.5;
    elseif column == "G2"
        y = 9.1+2;
     elseif column == "F3"
        y = 5.1+1;
    end
    if type == "pawn"
        z = 5;
        offset = (z - 10)*0.5;
    elseif type == "knight"
        z = 3;
        offset = (z - 10)*0.5;
    elseif type == "release_knight"
        z = 6;
        offset = (z - 10)*0.5;
    elseif type == "release_knight5"
        z = 8;
        offset = (z - 10)*0.5;
    elseif type == "release_pawn"
        z = 3;
        offset = 0;
    
    elseif type == "bishop"
        z = 5;
        offset = (z - 10)*0.5;
    elseif type == "release_bishop"
        z = 14;
        offset = (z - 10)*0.5;
    elseif type == "king"
        z = 7;
        offset = (z - 10) * 0.9;
    elseif type == "release_king"
        z = 8;
        offset = (z - 10) *0.7;
    elseif type == "rook"
        z = 4;
        offset = (z - 10) * 0.7;
    elseif type == "release_rook"
        z = 3;
        offset = (z - 10) * 0.5;
    elseif type == "pawn2"
        z = 7;
        offset = 0;
    elseif type == "release_pawn2"
        z = 10;
        offset = 0;
    elseif type == "release_pawn3"
        z = 5;
        offset = 0;
    elseif type == "knight2"
        z = 8;
        offset = (z - 10)*0.8;
    elseif type == "release_knight2"
        z = 7;
        offset = (z - 10)*0.5;
    elseif type == "release_knight6"
        z = 11;
        offset = (z - 10)*0.5;
    elseif type == "knight3"
        z = 5;
        offset = (z - 10)*0.8;
    elseif type == "bishop2"
        z = 11;
        offset = (z - 10)*0.8;
    elseif type == "release_bishop2"
        z = 8;
        offset = 0;
    elseif type == "pawn3"
        z = 4;
        offset = (z - 10)*0.8;
    elseif type == "knight4"
        z = 5;
        offset = (z - 10)*0.5;
    elseif type == "release_knight4"
        z = 4;
        offset = (z - 10)*0.5;
    elseif type == "queen"
        z = 6;
        offset = (z - 10)*0.5;
 
    elseif type == "release_queen"
        z = 10;
        offset = (z - 10)*0.5;
    elseif type == "bishop3"
        z = 1;
        offset = (z - 10)*0.8;
    elseif type == "queen2"
        z = 10;
        offset = 0;
    elseif type == "pawn4"
        z = 4;
        offset = (z - 10)*0.5;
    elseif type == "release_pawn4"
        z = 5;
        offset = (z - 10)*0.5;
    elseif type == "knight5"
        z = 3+3+2;
        offset = (z - 10)*0.5;
    elseif type == "release_rook2"
        z = 1;
        offset = (z - 10) * 0.8;
    end
    
    output_ma = [x;y;z];
end