block battery
// modulated input signal 
Modelica.Blocks.Interfaces.RealInput R1;
Real r1_6;
// modulated input signal 
Modelica.Blocks.Interfaces.RealInput R2;
Real r2_7;
// modulated input signal 
Modelica.Blocks.Interfaces.RealInput R3;
Real r3_8;
// modulated input signal 
Modelica.Blocks.Interfaces.RealInput InputB;
Real output_17;
// modulated input signal 
Modelica.Blocks.Interfaces.RealInput PB;
Real sf_19;
Modelica.Blocks.Interfaces.RealOutput theta;
Modelica.Blocks.Interfaces.RealOutput q;
Modelica.Blocks.Interfaces.RealOutput v1;
Real e_6;
Modelica.Blocks.Interfaces.RealOutput i1;
Real f_6;
Modelica.Blocks.Interfaces.RealOutput v2;
Real e_7;
Modelica.Blocks.Interfaces.RealOutput i2;
Real f_7;
Modelica.Blocks.Interfaces.RealOutput v3;
Real e_8;
Modelica.Blocks.Interfaces.RealOutput i3;
Real f_8;
Modelica.Blocks.Interfaces.RealOutput vp;
Real e_9;
Modelica.Blocks.Interfaces.RealOutput ip;
Real f_9;

Real e_0;
Real e_1;
Real e_2;
Real e_3;
Real e_4;
equation
(der(e_0)) = ((((0.000000) + ((-1.000000) * ((e_0) / (r1_6)))) + (((0.000000) + ((-1.000000) * (((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4)))) / (500.000000)))) + (output_17))) / (51.079000));
(der(e_1)) = ((((0.000000) + ((-1.000000) * ((e_1) / (r2_7)))) + (((0.000000) + ((-1.000000) * (((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4)))) / (500.000000)))) + (output_17))) / (51.216000));
(der(e_2)) = ((((0.000000) + ((-1.000000) * ((e_2) / (r3_8)))) + (((0.000000) + ((-1.000000) * (((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4)))) / (500.000000)))) + (output_17))) / (567.560000));
(der(e_3)) = ((((0.000000) + (sf_19)) + ((-1.000000) * ((((0.000000) + (e_3)) + (22.000000)) / (0.010000)))) / (615.300000));
(der(e_4)) = ((((0.000000) + ((-1.000000) * (((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4)))) / (500.000000)))) + (output_17)) / (106360.000000));
r1_6=R1;
r2_7=R2;
r3_8=R3;
output_17=InputB;
sf_19=PB;
theta = e_3;
q = e_4;
v1 = e_6;
i1 = f_6;
v2 = e_7;
i2 = f_7;
v3 = e_8;
i3 = f_8;
vp = e_9;
ip = f_9;
(e_6) = (e_0);
(f_6) = ((e_0) / (r1_6));
(e_7) = (e_1);
(f_7) = ((e_1) / (r2_7));
(e_8) = (e_2);
(f_8) = ((e_2) / (r3_8));
(e_9) = ((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4))));
(f_9) = (((-1.000000) * (((((0.000000) + ((-1.000000) * (e_0))) + ((-1.000000) * (e_1))) + ((-1.000000) * (e_2))) + ((-1.000000) * (e_4)))) / (500.000000));
end battery;