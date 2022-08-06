/*
Programa para foguete de propelente solido PASIVO

>Acelerômetro e Giroscópio 3 Eixos 6 DOF MPU-6050 - GY-521
>Sensor De Pressão e Temperatura BMP280
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
};
RF dadosRFdata;
int dadosRF = 0;

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

  Serial.println("/////////////////////////////////////////////////////////////////////");
  Serial.println("Inicializando cartao SD...");

  //Inicia a comunicação com o modulo SD
  if (!SD.begin(chipSelect))
    {
    Serial.println("Falha ao acessar o cartao!");
    Serial.println("Verifique o cartao/conexoes e reinicie o computador de voo...");
      return;
    }
    Serial.println("Cartao iniciado corretamente!");
    Serial.println();

  //Inicializa o nRF24L01
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    
    if (!radio.begin()) {
    Serial.println(F("Falha ao inicializar o RF24!!"));
      return;
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

  if (!bmp.begin()) {
    Serial.println(F("Falha ao inicializar o BMP280!"));
      return;
  }
}

void loop() {
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
    dataFile.println(AccZ / 2048);

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
    dataFile.println(" m");



    //Fecha o arquivo
    dataFile.close();
  }
  //Exibe um erro se nao conseguir abrir o arquivo
  else
  {
    Serial.println("Erro ao abrir o arquivo datalog.txt");
      delay(5000);
      return;
    
  }


  if(bmp.readTemperature() == 0){
    Serial.println("/////////////////////////////////////////////////////////////////////");
      Serial.println("erro ao medir a temperatura");
  }
  if(bmp.readPressure() == 0){
    Serial.println("erro ao medir a precao");
      Serial.println("/////////////////////////////////////////////////////////////////////");
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
    Serial.print("AccX:");
    Serial.print(AccX / 2048);
    Serial.print(" AccY:");
    Serial.print(AccY / 2048);
    Serial.print(" AccZ:");
    Serial.print(AccZ / 2048);
    Serial.print("  ");
  
  //printGyr
    Serial.print("GyrX:");
    Serial.print(GyrX / 16.4);
    Serial.print(" GyrY:");
    Serial.print(GyrY / 16.4);
    Serial.print(" GyrZ:");
    Serial.print(GyrZ / 16.4);
    Serial.print(" / ");

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