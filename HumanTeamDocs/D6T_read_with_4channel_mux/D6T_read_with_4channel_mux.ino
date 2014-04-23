#include <Wire.h>

int readtohi[8]; // object temperature hi bytes
int readtolo[8]; // object temperature low bytes
float readto[8]; // object temp converted to double
int readtahi; // ambient temperature hi byte
int readtalo; // ambient temperature low byte
float readta; // ambient temperature converted to double
byte pec;     // packet error code
int sensaddr = 0x0a; // sensor slave address
int muxaddr = B1110000; // mux slave address
int chan0 = B00000100; // mux address of channel 0
int chan1 = B00000101; // mux address of channel 1
int chan2 = B00000110; // mux address of channel 2
int chan3 = B00000111; // mux address of channel 3

void i2c_declare_chan(byte channel) {
  Wire.beginTransmission(muxaddr); //address of i2c mux in write mode
  Wire.write(channel); 
  Wire.endTransmission(); // release i2c mux
}

void i2c_read_sensor() {
  Wire.beginTransmission(sensaddr); //write mode
  Wire.write(0x4c); // write temp getting instruction
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(sensaddr, 19); //request 19 bytes, release device after transmission
  int numavail = Wire.available();
  readtalo = Wire.read(); // read ambient temp
  readtahi = Wire.read();
  for (int i = 0 ; i < 8 ; i++) { // read object temps
    readtolo[i] = Wire.read();
    readtohi[i] = Wire.read();
  }
  pec = Wire.read();  
  Serial.print("Ambient temp: ");
  readta=(readtahi*256+readtalo)/10;
  Serial.println(readta);
  for (int i = 0; i < 8; i++) {
    Serial.print("Pixel ");
    Serial.print(i);
    Serial.print(":");
    readto[i]=(readtohi[i]*256+readtolo[i])/10;
    Serial.println(readto[i]);
  }
}
  
void setup() {
  Serial.begin(9600); // allow serial output
  Wire.begin(); // begin i2c mode
}

void loop() {
  i2c_declare_chan(chan0);
  i2c_read_sensor();
  i2c_declare_chan(chan1);
  i2c_read_sensor();
  i2c_declare_chan(chan2);
  i2c_read_sensor();  
  i2c_declare_chan(chan3);
  i2c_read_sensor(); 
  delay(3500);
}
