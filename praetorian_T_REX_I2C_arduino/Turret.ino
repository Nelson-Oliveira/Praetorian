void Turret()
{

    //Read the pulse from Pin10 and 11 either HIGH or LOW
  channel0 = pulseIn(10, HIGH);
  Serial.print(channel0);Serial.print("\t");
  channel1 = pulseIn(11, HIGH);
  Serial.print(channel1);Serial.print("\t");
  Serial.print("\n");
/*  
  valpan = channel0;            // reads the value of the RCinput (value between 1000 and 2000)
  valpan = map(valpan, 1050, 1950, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  pan.write(valpan);                  // sets the servo position according to the scaled value
  delay(250);                           // waits for the servo to get there

  valtilt = channel1;            // reads the value of the RCinput (value between 1000 and 2000)
  valtilt = map(valtilt, 1130, 1750, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  tilt.write(valtilt);                  // sets the servo position according to the scaled value
  delay(250);                           // waits for the servo to get there
*/
  valpan = channel0;            // reads the value of the RCinput (value between 1000 and 2000)
  if (valpan > 0) {
    valpan = map(valpan, 1200, 1900, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 1050 1950
    if (abs(valpanprev-valpan)>15) {
      pan.write(valpan);                  // sets the servo position according to the scaled value
      valpanprev = valpan;
    }
    delay(5);                           // waits for the servo to get there
  }
  else {
    valpan = 90;
  }

  valtilt = channel1;            // reads the value of the RCinput (value between 1000 and 2000)
  if (valtilt > 0) {
    valtilt = map(valtilt, 1200, 1700, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 1130 1750
    if (abs(valtiltprev-valtilt)>15) {
      tilt.write(valtilt);                  // sets the servo position according to the scaled value
      valtiltprev = valtilt;
    }
    delay(5);                           // waits for the servo to get there
  }
  else {
    valtilt =90;
  }
 
}
