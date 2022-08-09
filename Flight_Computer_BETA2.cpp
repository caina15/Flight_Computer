#include <Arduino.h>
/*
Programa para foguete de propelente solido PASIVO (sem controle do motor)

>Acelerômetro e Giroscópio 3 Eixos 6 DOF MPU-6050 - GY-521
>Sensor De Pressão e Temperatura BMP280
>Módulo GPS NEO-6M
>Módulo Micro SD Card
>Módulo Wireless NRF24L01 Transceiver com Antena SMA
*/

// Inclusão das Bibliotecas
  #include<Wire.h>
  #include <SPI.h>
  #include <Adafruit_BMP280.h>
  #include <SD.h>
  #include <nRF24L01.h>
  #include <RF24.h>
  #include <SoftwareSerial.h>
  #include <TinyGPS.h>
 
// Endereços I2C
  const int MPU = 0x68;
  Adafruit_BMP280 bmp;

//estrutura de bytes do transmissor RF
struct RF {
  byte AccxRF;
  byte AccyRF;
  byte AcczRF;
  byte GyrxRF;
  byte GyryRF;
  byte GyrzRF;
  byte tempRF;
  byte presRF;
  byte altRF; 
  byte latRF; //latitude
  byte lonRF; //longitude
  byte satsRF; //quantidade de sats
  byte precRF; //Precisão

};
RF dadosRFdata;
int dadosRF = 0;

//GPS
TinyGPS gps;
SoftwareSerial ss(3, 2);
float flat, flon;

//Pinos de conexão do pino CE e CSN do NR24L01 
  RF24 radio(8, 7); // CE, CSN

// Endereço SPI do NR24L01 
  const byte address[6] = "00001";

//Pino de conexão do pino CS do modulo SD 
  const int chipSelect = 10;

// Variaveis para armazenar valores do sensor MPU-6050
  float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;

void setup() {
Serial.begin(9600);
  Serial.println(F("Inicializando cartao SD..."));

  //Inicia a comunicação com o modulo SD
  if (SD.begin(chipSelect))
    {
      Serial.println(F("Cartao iniciado corretamente!"));
    }else{
      Serial.println(F("Falha ao acessar o cartao!"));
      Serial.println(F("Verifique o cartao/reinicie o computador de voo"));
      delay(100);
      exit(0);
    }
    
  //Inicializa o nRF24L01
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    
    if (radio.begin()) {
    Serial.println(F("RF24 iniciado corretamente!"));
    }else{
      Serial.println(F("Falha ao inicializar o RF24!"));
      delay(100);
      exit(0);
    }

  // Inicializa o MPU-6050
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

  // Configura Giroscópio para fundo de escala desejado
    /*
    Wire.write(0b00000000); // fundo de escala em +/-250°/s
    Wire.write(0b00001000); // fundo de escala em +/-500°/s
    Wire.write(0b00010000); // fundo de escala em +/-1000°/s
    Wire.write(0b00011000); // fundo de escala em +/-2000°/s
    */
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);
    Wire.write(0x00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
    Wire.endTransmission();

  // Configura Acelerometro para fundo de escala desejado
    /*
      Wire.write(0b00000000); // fundo de escala em +/-2g
      Wire.write(0b00001000); // fundo de escala em +/-4g
      Wire.write(0b00010000); // fundo de escala em +/-8g
      Wire.write(0b00011000); // fundo de escala em +/-16g
    */
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);
    Wire.write(0b00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
    Wire.endTransmission();

  //bmp280
    // configuração padrão do datasheet
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //modo de operação
                  Adafruit_BMP280::SAMPLING_X2,     //sobreamostragem de temp.
                  Adafruit_BMP280::SAMPLING_X16,    //sobreamostragem de pressão
                  Adafruit_BMP280::FILTER_X16,      //filtragem
                  Adafruit_BMP280::STANDBY_MS_500); //tempo de espera

  if (bmp.begin()) {
    Serial.println(F("BMP280 inicializado corretamente!"));
  }else{
    Serial.println(F("Falha ao inicializar o BMP280!"));
    delay(100);
    exit(0);
  }

  ss.begin(9600);
  Serial.println(F("Inicializando GPS")); 
}

void loop() {

  //GPS
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (ss.available())
      {
        char c = ss.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
      }
    }

    if (newData)
    {
      //float flat, flon; coloquei em global
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    }
  
    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  // Comandos para iniciar transmissão de dados do MCU6050
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true); // Solicita os dados ao sensor

  // Armazena o valor dos sensores nas variaveis correspondentes do MCU6050
    AccX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AccY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AccZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Temp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyrX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyrY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyrZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

//RF
    /* Alterar divisão conforme fundo de escala escolhido:
    Acelerômetro
    +/-2g = 16384
    +/-4g = 8192
    +/-8g = 4096
    +/-16g = 2048

    Giroscópio
    +/-250°/s = 131
    +/-500°/s = 65.6
    +/-1000°/s = 32.8
    +/-2000°/s = 16.4
    */
  // Leitura dos dadosRF
  dadosRF = AccX / 2048;
  dadosRFdata.AccxRF;
  dadosRF = AccY / 2048;
  dadosRFdata.AccyRF;
  dadosRF = AccZ / 2048;
  dadosRFdata.AcczRF;


  dadosRF = GyrX / 16.4;
  dadosRFdata.GyrxRF;
  dadosRF = GyrY / 16.4;
  dadosRFdata.GyryRF;
  dadosRF = GyrZ / 16.4;
  dadosRFdata.GyrzRF;


  dadosRF = bmp.readTemperature();
  dadosRFdata.tempRF;
  dadosRF = bmp.readPressure();
  dadosRFdata.presRF;
  dadosRF = bmp.readAltitude(1013.25);// ajuste a alt no local
  dadosRFdata.altRF;

  dadosRF = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
  dadosRFdata.latRF;  
  dadosRF = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
  dadosRFdata.lonRF;  
  dadosRF = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
  dadosRFdata.satsRF;  
  dadosRF = gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop();
  dadosRFdata.precRF;

  // Envia dados via RF
  radio.write(&dadosRFdata, sizeof(dadosRF));

//SD

  //Abre o arquivo datalog.txt
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  //Grava as informacoes no arquivo
  if (dataFile)
  {
    //printAcc 
    dataFile.print("AccX");
    dataFile.print(AccX / 2048);
    dataFile.print("   ");
    dataFile.print("AccY");
    dataFile.print(AccY / 2048);
    dataFile.print("   ");
    dataFile.print("AccZ");
    dataFile.print(AccZ / 2048);

    dataFile.print(" / ");

    //printGyr
    dataFile.print("GyrX");
    dataFile.print(GyrX / 16.4);
    dataFile.print("   ");
    dataFile.print("GyrY");
    dataFile.print(GyrY / 16.4);
    dataFile.print("   ");
    dataFile.print("GyrZ");
    dataFile.print(GyrZ / 16.4);

    dataFile.print(" / ");

    //BMP280
    dataFile.print("Temp = ");
    dataFile.print(bmp.readTemperature());
    dataFile.print(" *C");
    dataFile.print("   ");
    dataFile.print("Pres = ");
    dataFile.print(bmp.readPressure());
    dataFile.print(" Pa");
    dataFile.print("   ");
    dataFile.print("alt = ");
    dataFile.print(bmp.readAltitude(1013.25)); // ajuste a alt no local
    dataFile.print(" m");

    dataFile.print(" / ");

    dataFile.print("LAT = ");
    dataFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    dataFile.print("   ");
    dataFile.print("LON = ");
    dataFile.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    dataFile.print("   ");
    dataFile.print("SAT = ");
    dataFile.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    dataFile.print("   ");
    dataFile.print("PREC = ");
    dataFile.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

    //Fecha o arquivo
    dataFile.close();
  }
  //Exibe um erro se nao conseguir abrir o arquivo
  else
  {
    Serial.println(F("Erro ao abrir o arquivo datalog.txt"));
    delay(100);
    exit(0);

  }

  if(bmp.readTemperature() == 0){
      Serial.println("erro ao medir a temperatura");
    delay(100);
    exit(0);
  }
  if(bmp.readPressure() == 0){
    Serial.println(F("erro ao medir a precao"));
    delay(100);
    exit(0);
  }
    /* Alterar divisão conforme fundo de escala escolhido:
    Acelerômetro
    +/-2g = 16384
    +/-4g = 8192
    +/-8g = 4096
    +/-16g = 2048

    Giroscópio
    +/-250°/s = 131
    +/-500°/s = 65.6
    +/-1000°/s = 32.8
    +/-2000°/s = 16.4
    */

  // Imprime na Serial os valores obtidos
  //printAcc  
    Serial.print(F("AccX:"));
    Serial.print(AccX / 2048);
    Serial.print(F(" AccY:"));
    Serial.print(AccY / 2048);
    Serial.print(F(" AccZ:"));
    Serial.print(AccZ / 2048);
    Serial.print(F("  "));
  
  //printGyr
    Serial.print(F("GyrX:"));
    Serial.print(GyrX / 16.4);
    Serial.print(F(" GyrY:"));
    Serial.print(GyrY / 16.4);
    Serial.print(F(" GyrZ:"));
    Serial.print(GyrZ / 16.4);
    Serial.print(F(" / "));

  //printBmp280
    Serial.print(F("Temp = "));
    Serial.print(bmp.readTemperature());
    Serial.print(" *C");
    Serial.print("   ");

    Serial.print(F("Pres = "));
    Serial.print(bmp.readPressure());
    Serial.print(" Pa");
    Serial.print("   ");

    Serial.print(F("alt = "));
    Serial.print(bmp.readAltitude(1013.25));// ajuste a alt no local
    Serial.print(" m");
    Serial.println("   ");

  delay(100);
}
