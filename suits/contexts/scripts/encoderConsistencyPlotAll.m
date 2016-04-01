
function encoderConsistencyPlotAll(partname)


figure(1);
filename = strcat("encConsis_jointPos_MotorPos_", partname, ".txt");
oneFile_plot(filename, "jointPos vs MotorPos");

figure(2);
filename = strcat("encConsis_jointVel_motorVel_", partname, ".txt");
oneFile_plot(filename, "jointVel vs MotorVel");

figure(3);
filename = strcat("encConsis_joint_derivedVel_vel_", partname, ".txt");
oneFile_plot(filename, "joint: derivedVel vs misuredVel");

figure(4);
filename = strcat("encConsis_motor_derivedVel_vel_", partname, ".txt");
oneFile_plot(filename, "motor: derivedVel vs misuredVel");

endfunction

