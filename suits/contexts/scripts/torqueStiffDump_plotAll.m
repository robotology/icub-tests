
function torqueStiffDump_plotAll(partname, numofjoint, stiffness, dumping, jointlist)


p=mfilename("fullpath"); #get current funtion file name with full path

in =  rindex (p, "/"); #get the index of last occurence of /
onlydir = strtrunc(p, in); #cat the name of file from full path. in Only dit i have the path to scripts

#I need to add the path to script torqueStiffDump_plot
addpath(onlydir);

figure(1);

for i= 1:1:numofjoint
    
    subplot(numofjoint, 1, i, "align");
    filename = strcat("posVStrq_", partname, "_j",  num2str(jointlist(i)), ".txt");
    printf("index %d file=%s\n", i, filename);
    torqueStiffDump_plot(filename, stiffness(i));
    refresh();
    if(i==1)
    title("Position vs Torque");
    endif
endfor



figure(2);

for i= 1:1:numofjoint
    
    subplot(numofjoint, 1, i, "align");
    filename = strcat("velVStrq_", partname, "_j",  num2str(jointlist(i)), ".txt");
    printf("index %d file=%s\n", i, filename);
    torqueStiffDump_plot(filename, stiffness(i));
    refresh();
    if(i==1)
    title("Velocity vs Torque");
    endif
endfor

endfunction

