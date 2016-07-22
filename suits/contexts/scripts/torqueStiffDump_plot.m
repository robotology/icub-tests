#define function to plot one file
function torqueStiffDump_plot(filename, linearfact)
printf("sono nella plot");
data = load(filename);
m=min(data(:,1));
mx=max(data(:,1));

x=m:1:mx;
y=x*linearfact;
plot(data(:,1), data(:,2), "ro;data;", x, y, "-;y=x*linearfactor;");

endfunction
