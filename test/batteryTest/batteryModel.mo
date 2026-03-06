model batteryModel

  inputCurrent pulse;
  // my environment
  batteryEnv env;
  // The battery bond graph model
  battery bat;
equation
  // connect(amplitudeSignal.y, pulse.amplitude);
  connect(pulse.y, env.iB);
  connect(pulse.y, bat.InputB);
  connect(env.R1, bat.R1);
  connect(env.R2, bat.R2);
  connect(env.R3, bat.R3);
  connect(env.PB, bat.PB);
  connect(bat.theta, env.theta);
  connect(bat.q, env.q);
  connect(bat.v1, env.v1);
  connect(bat.i1, env.i1);
  connect(bat.v2, env.v2);
  connect(bat.i2, env.i2);
  connect(bat.v3, env.v3);
  connect(bat.i3, env.i3);
  connect(bat.ip, env.ip);
  connect(bat.vp, env.vp);

end batteryModel;
