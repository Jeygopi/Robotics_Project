%this function calculates the via points
function [viapoint1,viapoint2] = gen_viapoint(startpoint, endpoint)
    viapoint1 = startpoint;
    viapoint1(3) = viapoint1(3) + 16.5;
    viapoint2 = endpoint;
    viapoint2(3) = viapoint2(3) + 16.5;
end