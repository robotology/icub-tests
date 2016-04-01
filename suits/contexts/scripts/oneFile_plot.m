#define functio to plot one file
function oneFile_plot(filename, titleStr)
#if (nargin != 1)
#    error ("usage: one_plot(filename)");
#endif
data = load(filename);
for i= 1:4
    subplot(4, 1, i);
    plot(data(:, 0+i), "r", data(:,4+i), "b");
    if(i==1)
    title(titleStr);
    endif
endfor

endfunction
