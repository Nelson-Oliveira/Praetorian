/*
void Encoders()
{
  static byte lencold,lencnew,rencold,rencnew;                      // old encoder values
  lencold=lencnew;                                                  // store previous value for left  encoder
  rencold=rencnew;                                                  // store previous value for right encoder
  lencnew=digitalRead(lmencpin);                                    // read new left  encoder value
  rencnew=digitalRead(rmencpin);                                    // read new right encoder value
  if (lencold!=lencnew) lmenc+=lmspeed/abs(lmspeed);                // if old does not equal new then add or subtract 1 depending on motor direction
  if (rencold!=rencnew) rmenc+=lmspeed/abs(lmspeed);                // if old does not equal new then add or subtract 1 depending on motor direction
}
*/


void Encoders()
{
  static byte lencolda,lencnewa,rencolda,rencnewa,lencoldb,lencnewb,rencoldb,rencnewb;                      // old encoder values
  lencolda=lencnewa;                                                  // store previous value for left  encoder
  rencolda=rencnewa;                                                  // store previous value for right encoder
  lencoldb=lencnewb;                                                  // store previous value for left  encoder
  rencoldb=rencnewb;                                                  // store previous value for right encoder
  lencnewa=digitalRead(lmencpina);                                    // read new left  encoder value
  lencnewb=digitalRead(lmencpinb);                                    // read new left  encoder value
  rencnewa=digitalRead(rmencpina);                                    // read new right encoder value
  rencnewb=digitalRead(rmencpinb);                                    // read new right encoder value

  if((lencolda != lencnewa) || (lencoldb != lencnewb)){
    if (lencolda == 0 && lencoldb == 0){
      if(lencolda != lencnewa){
        lmenc++;
      } else {
        lmenc--;
      }
    } else if (lencolda == 1 && lencoldb == 0){
      if(lencolda != lencnewa){
        lmenc--;
      } else {
        lmenc++;
      }      
    } else if (lencolda == 0 && lencoldb == 1){
      if(lencolda != lencnewa){
        lmenc--;
      } else {
        lmenc++;
      }
    } else if ((lencolda == 1) && (lencoldb == 1)){
      if(lencolda != lencnewa){
        lmenc++;
      } else {
        lmenc--;
      }
    }
  }  
  if((rencolda != rencnewa) || (rencoldb != rencnewb)){
    if (rencolda == 0 && rencoldb == 0){
      if(rencolda != rencnewa){
        rmenc++;
      } else {
        rmenc--;
      }
    } else if (rencolda == 1 && rencoldb == 0){
      if(rencolda != rencnewa){
        rmenc--;
      } else {
        rmenc++;
      }      
    } else if (rencolda == 0 && rencoldb == 1){
      if(rencolda != rencnewa){
        rmenc--;
      } else {
        rmenc++;
      }
    } else if ((rencolda == 1) && (rencoldb == 1)){
      if(rencolda != rencnewa){
        rmenc++;
      } else {
        rmenc--;
      }
    }
  }  
}
/*
void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(lmencpina) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(lmencpinb) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
      lmenc = encoder0Pos;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
      lmenc = encoder0Pos;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(lmencpinb) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
      lmenc = encoder0Pos;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
      lmenc = encoder0Pos;
    }
  }
//  lmenc = encoder0Pos;
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(lmencpinb) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(lmencpina) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
      lmenc = encoder0Pos;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
      lmenc = encoder0Pos;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(lmencpina) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
      lmenc = encoder0Pos;
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
      lmenc = encoder0Pos;
    }
  }
  
//  lmenc = encoder0Pos;
}

// ************** encoder 2 *********************

void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(rmencpina) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(rmencpinb) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
      rmenc = encoder1Pos; 
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
      rmenc = encoder1Pos; 
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(rmencpinb) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
      rmenc = encoder1Pos; 
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
      rmenc = encoder1Pos; 
    }
  }
//  rmenc = encoder1Pos; 
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(rmencpinb) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(rmencpina) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
      rmenc = encoder1Pos; 
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
      rmenc = encoder1Pos; 
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(rmencpina) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
      rmenc = encoder1Pos; 
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
      rmenc = encoder1Pos; 
    }
  }
  
//  rmenc = encoder1Pos; 
}*/
