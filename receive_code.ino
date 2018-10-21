uint8_t receive_function(char* buff, uint8_t sizevar)
{
  static uint8_t ctr = 0;     // store the current position in the buff array
  uint8_t ch;                 // store the last character received
  if (Serial.available() > 0) // true when characters in serial input buffer
  {
    ch = Serial.read();       // store character from buffer in ch
    if( ctr < sizevar) {      // if the ctr is less than your buffer yet
      buff[ctr++] = ch;       // add it to your buffer
    }
    if (ch == '\r')           // if that character is a carriage return
    {
      buff[ctr-1] = 0;        // replace '\r' with '\0' (string termination)
      ctr = 0;                // reset the pointer
      Serial.print("Command: "); // print a string and stay on the same line
      if(buff[0] != 0)
        Serial.println(buff); // print received followed by a new line
      else
        Serial.print("\n");
      return 1;
    }
    else
      return 0;
  }                           //end serial was available
  return 0;
}

void reset(void) {
#if defined(__AVR__)
  Serial.println("Do not know how to reset an AVR processor, try chipKIT...");
#else
  Serial.println("Close serial terminal, resetting board in...");
  uint8_t sec = 5;
  while (sec >= 1)
  {
    Serial.print(sec, DEC);
    Serial.println(" seconds...");
    delay(1000);
    sec--;
  }
  executeSoftReset(ENTER_BOOTLOADER_ON_BOOT);
#endif
}


