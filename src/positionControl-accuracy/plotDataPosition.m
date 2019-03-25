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

hold on;
data = load ('positionControlAccuracy_plot_head0.txt');
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
