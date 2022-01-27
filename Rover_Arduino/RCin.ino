/******************************** RC Controller ************************************************/
void RCin() {
  SwitchA = pulseIn(4, HIGH, 25000);
  if (SwitchA > 1000) {
    RCoff = 0;
    Ystick = pulseIn(3, HIGH, 25000);  // Y Stick
    Xstick = pulseIn(2, HIGH, 25000);  // X Stick
    Ystick = map(Ystick, 1000, 2000, 127, 0);
    Xstick = map(Xstick, 1000, 2000, 127, 0);

    if (Ystick > 59 && Ystick < 69)
    {
      Ystick = 64;
    }
    if (Xstick > 59 && Xstick < 69)
    {
      Xstick = 64;
    }

    roboclaw.ForwardBackwardMixed(address, Ystick);
    roboclaw.LeftRightMixed(address, Xstick);
  }
  else {
    RCoff++;
    if (RCoff > 21){
      RCoff = 21;
    }
  }


}
