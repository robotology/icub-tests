% iCub Robot Unit Tests (Robot Testing Framework)
%
% Copyright (C) 2015-2019 Istituto Italiano di Tecnologia (IIT)
%
% This library is free software; you can redistribute it and/or
% modify it under the terms of the GNU Lesser General Public
% License as published by the Free Software Foundation; either
% version 2.1 of the License, or (at your option) any later version.
%
% This library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public
% License along with this library; if not, write to the Free Software
% Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

function torqueStiffDamp_plotAll(partname, numofjoint, stiffness, damping, jointlist)


p=mfilename("fullpath"); #get current funtion file name with full path

in =  rindex (p, "/"); #get the index of last occurence of /
onlydir = strtrunc(p, in); #cat the name of file from full path. in Only dit i have the path to scripts

#I need to add the path to script torqueStiffDump_plot
addpath(onlydir);

figure(1);

for i= 1:1:numofjoint
    
    subplot(numofjoint, 1, i, "align");
    filename = strcat("posVStrq_", partname, "_j",  num2str(jointlist(i)), ".txt");
    printf("I'm going to plot file %s\n", filename);
    torqueStiffDamp_plot(filename, stiffness(i));
    refresh();
    if(i==1)
    title("Position vs Torque");
    endif
endfor



figure(2);

for i= 1:1:numofjoint
    
    subplot(numofjoint, 1, i, "align");
    filename = strcat("velVStrq_", partname, "_j",  num2str(jointlist(i)), ".txt");
    printf("I'm going to plot file %s\n", filename);
    torqueStiffDamp_plot(filename, damping(i));
    refresh();
    if(i==1)
    title("Velocity vs Torque");
    endif
endfor

endfunction

