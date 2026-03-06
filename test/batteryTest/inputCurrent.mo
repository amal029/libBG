block inputCurrent

  Modelica.Blocks.Sources.Pulse pulse(amplitude = 6,
                                      width = 20,
                                      period = 100);
  // Modelica.Blocks.Sources.Sine amplitudeSignal(amplitude = 0.5, f = 0.5, offset = 1);
  // Modelica.Blocks.Sources.Sine widthSignal(amplitude = 20, f = 0.3, offset = 50);
  Modelica.Blocks.Interfaces.RealOutput y;
  
equation
  y = pulse.y;
end inputCurrent;

