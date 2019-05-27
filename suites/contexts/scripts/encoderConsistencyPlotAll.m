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

function encoderConsistencyPlotAll(partname, numofjoint)


figure(1);
filename = strcat("encConsis_jointPos_MotorPos_", partname, ".txt");
oneFile_plot(filename, "jointPos vs MotorPos", numofjoint);

figure(2);
filename = strcat("encConsis_jointVel_motorVel_", partname, ".txt");
oneFile_plot(filename, "jointVel vs MotorVel", numofjoint);

figure(3);
filename = strcat("encConsis_joint_derivedVel_vel_", partname, ".txt");
oneFile_plot(filename, "joint: derivedVel vs misuredVel", numofjoint);

figure(4);
filename = strcat("encConsis_motor_derivedVel_vel_", partname, ".txt");
oneFile_plot(filename, "motor: derivedVel vs misuredVel", numofjoint);

endfunction

