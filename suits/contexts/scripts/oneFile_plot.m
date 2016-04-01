#define functio to plot one file
function oneFile_plot(filename, titleStr, numofjoint)

data = load(filename);
for i= 1:1:numofjoint
    if(numofjoint>4)
        subplot(3, 2, i, "align");
    else
        subplot(numofjoint, 1, i, "align");
    endif
    printf("index %d\n", i);
    plot(data(:, 0+i), "r", data(:,numofjoint+i), "b");
    refresh();
    if(i==1)
    title(titleStr);
    endif
endfor

endfunction
