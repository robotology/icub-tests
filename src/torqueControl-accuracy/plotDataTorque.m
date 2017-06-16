hold on;
data = load ('torqueControlAccuracy_plot_head0.txt');
cycles = max(data(:,1));
for cyc=0:cycles;
    cyc
    vec = find(data(:,1)==cyc);
    plot (data(vec,2), data(vec,3),'b');
    plot (data(vec,2), data(vec,4),'r');
end
xlim ([-0.9 3]);
ylim ([-1 6]);
grid on;