block batteryEnv
  Modelica.Blocks.Interfaces.RealOutput R1;
  Modelica.Blocks.Interfaces.RealOutput R2;
  Modelica.Blocks.Interfaces.RealOutput R3;
  Modelica.Blocks.Interfaces.RealOutput PB;
  Modelica.Blocks.Interfaces.RealInput theta;
  Modelica.Blocks.Interfaces.RealInput q;
  Modelica.Blocks.Interfaces.RealInput v1;
  Modelica.Blocks.Interfaces.RealInput i1;
  Modelica.Blocks.Interfaces.RealInput v2;
  Modelica.Blocks.Interfaces.RealInput i2;
  Modelica.Blocks.Interfaces.RealInput v3;
  Modelica.Blocks.Interfaces.RealInput i3;
  Modelica.Blocks.Interfaces.RealInput vp;
  Modelica.Blocks.Interfaces.RealInput ip;
  Modelica.Blocks.Interfaces.RealInput iB;

  parameter Real R10 = 0.05582;
  parameter Real R20 = 0.001847;
  parameter Real R30 = 0.3579;
  parameter Real A11 = -0.025025;
  parameter Real A32 = 0.2208;
  parameter Real A31 = -2.5315;
  parameter Real Kc = 1.33;
  parameter Real epsilon = 0.642;
  parameter Real c0star = 270720;
  parameter Real qmax = 2765360;
  parameter Real thetaf = -35;
  parameter Real delta = 0.61;
  parameter Real istar = 5;

  Real SOC;
  Real DOC;
equation
  SOC = 1 - ((qmax - q)/(Kc * c0star * ((1 - theta/thetaf) ^ epsilon)));
  DOC = 1 - ((qmax - q)/(Kc * c0star * (1 - theta/thetaf) ^ epsilon)) * (1 + (Kc - 1) * (iB/istar)^delta);
  R1 = R10 + A11 * SOC;
  // R2 = -R20 * log(DOC);
  R2 = -R20 * (DOC);
  R3 = (R30 * exp(A31)* (1- SOC))/(1 + exp(A32)*iB);
  PB = ip*vp + abs(i3*v3) + abs(i2*v2) + abs(i1*v1);
end batteryEnv;
