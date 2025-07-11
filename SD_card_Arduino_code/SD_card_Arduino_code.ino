#include <SD.h>
const int chipSelect = 10;
File myFile;
void setup() {
  Serial.begin(9600);
  while (!Serial);
  check_and_create_file();
  write_text();
}
void loop() {
}
void check_and_create_file()
{
  Serial.print("Initializing SD card...");
  /*Check SD detecting or not*/
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  if (SD.exists("data_log.txt"))
    Serial.println("data_log.txt exists.");
  else
  {
    Serial.println("data_log.txt doesn't exist.");
    /* open a new file and immediately close it:
      this will create a new file */
    Serial.println("Creating data_log.txt...");
    myFile = SD.open("data_log.txt", FILE_WRITE);
    myFile.close();
    /* Now Chec agin if the file exists in
      the SD card or not */
    if (SD.exists("data_log.txt"))
      Serial.println("data_log.txt exists.");
    else
      Serial.println("data_log.txt doesn't exist.");
  }
}
void write_text()
{
  myFile = SD.open("data_log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to data_log.txt...");
    myFile.println("SD Card detected and can able to write data into it");
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data_log.txt");
  }
  // re-open the file for reading:
  myFile = SD.open("data_log.txt");
  if (myFile) {
    Serial.println("data_log.txt:");
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data_log.txt");
  }
}