#define GPS_IN Serial1
#define GPS_OUT Serial1
//String data = "$GNRMC,163648,A,4216.55055,N,7148.01165,W,0.031,301019,A*72,,,,,,,,\n$GNVTG,T,M,0.031,N,0.057,K,A*3D,,,,,,,,,,\n$GNGGA,163648,4216.55055,N,7148.01165,W,1,7,1.22,191.3,M,-33.4,M,*73,,,,\n$GNGSA,A,3,20,21,29,15,5,2.24,1.22,1.88*13,,,,,,,\n$GNGSA,A,3,82,66,2.24,1.22,1.88*12,,,,,,,,,,\n$GPGSV,2,1,6,5,35,78,30,13,29,15,80,203,37,20,23,284,47*4D\n$GPGSV,2,2,6,21,47,307,37,29,31,219,45*7D,,,,,,\n$GLGSV,2,1,5,66,20,51,32,76,29,82,59,204,39,29*6F,,,\n$GLGSV,2,2,05*60,,,,,,,,,,,,,,\n$GNGLL,4216.55055,N,7148.01165,W,163648,A,A*63,,,,,,,,,,\n";
// dGPS J8.10 to Teensy pin 0
// Teensy pin 1 or 10 to Pixhawk middle pin
void setup() {
  GPS_IN.begin(19200);
  GPS_OUT.begin(19200);
  Serial.begin(19200);
}

void loop() {
  if(GPS_IN.available()) {
    GPS_OUT.println(GPS_IN.readStringUntil('\n'));
  }
}
