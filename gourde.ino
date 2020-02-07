/*
* Projet Arduino Gourde connectée v1.0 Safe-Bottle
* Samuel BARON
* Théo LEFEVRE
*/

/*=============================================== Initialisation ===============================================*/
#include <OneWire.h>         // Necessaire pour l'utilisation du capteur de temperature DS18B20
#include <SoftwareSerial.h>  // Necessaire afin de creer une deuxieme liaison Serial (1 -> Serial, 2 -> Bluetooth)

/* Creation connexion Serial Bluetooth */
#define TxD 3
#define RxD 2
SoftwareSerial bluetoothSerial(TxD, RxD);

#define TdsSensorPin A1              // Analog TDS pin 
#define VREF 5.0                     // analog reference voltage(Volt) of the ADC
#define SCOUNT  10                   // sum of sample point

/* Declaration des variables necessaires au calcul et à l'approximation du TDS */
int analogBuffer[SCOUNT];            // store the TDS analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
float EC = 0;
float temperature = 25;
float PPMconversion = 0.7;

/* Declaration des Pins de la led d'etat RGB*/
const byte PIN_LED_R = 4;
const byte PIN_LED_G = 5;
const byte PIN_LED_B = 6;

/* Definition des couleurs, principe de la synthese additive */
const byte COLOR_BLACK = 0b000;
const byte COLOR_RED = 0b100;
const byte COLOR_YELLOW = 0b110;
const byte COLOR_GREEN = 0b010;
const byte COLOR_CYAN = 0b011;

const byte BROCHE_ONEWIRE = 7;       // Broche du bus 1-Wire
OneWire temp_sonde(BROCHE_ONEWIRE);  // Création de l'objet OneWire pour manipuler le bus 1-Wire

/* Code de retour de la fonction getTemperature() */
enum DS18B20_RCODES {
  READ_OK,          // Lecture ok
  NO_SENSOR_FOUND,  // Pas de capteur
  INVALID_ADDRESS,  // Adresse reçue invalide
  INVALID_SENSOR    // Capteur invalide (pas un DS18B20)
};


/*=============================================== Setup ===============================================*/
void setup()
{
  Serial.begin(115200);        // Initialisation communication Serial
  bluetoothSerial.begin(9600); // Initialisation communication Serial Bluetooth
  
  pinMode(TdsSensorPin,INPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  
  displayColor(COLOR_BLACK);  // On eteint la led au demarrage
}


/*=============================================== Loop ===============================================*/
void loop(){
   
  /* Test presence capteur temperature */
  if (getTemperature(&temperature, true) != READ_OK) {
    Serial.println(F("Erreur de lecture du capteur temperature"));
    delay(1000);
    return;
  }

  tdsValue = getTDS(); // Recuperation valeur TDS

  EC = tdsValue / ( PPMconversion * 1000.00 ); // Calcul conductivite a partir du TDS

  displayValeursSerial(temperature, tdsValue, EC); // Affichage des données sur la console Serie
  displayValeursBT(temperature, tdsValue, EC);     // Affichage des données sur la console Serie Bluetooth
  displayEtat(tdsValue, EC);                       // Affichage de l'etat sur la led
}


/*=============================================== Fonctions ===============================================*/

/*=============================================== getTDS ===============================================*/
/* Calcul du TDS a partir du voltage recuperé sur le PIN A1 (analog)*/
float getTDS(){
  
  static unsigned long analogSampleTimepoint = millis();             // millis -> Returns the number of milliseconds passed since the Arduino board began running the current program.
   
  if(millis() - analogSampleTimepoint > 40U){                        // every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);      // read the analog value and store into the buffer
    analogBufferIndex++;
     
    if(analogBufferIndex == SCOUNT) 
      analogBufferIndex = 0;
  } 
   
  static unsigned long printTimepoint = millis();
   
  if(millis() - printTimepoint > 800U){
    printTimepoint = millis();
    
    for(copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;     // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1 + 0.02 * ( temperature - 25.0 );                 // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage = averageVoltage / compensationCoefficient;              // temperature compensation
    
    tdsValue = 0.5 * ( 133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage );
    //convert voltage value to tds value
  }
  return tdsValue;
}

/*=============================================== getTemperature ===============================================*/
/* Fonction de lecture de la température via un capteur DS18B20 numerique */
byte getTemperature(float *temperature, byte reset_search) {
  byte data[9], addr[8];
  // data[] : Données lues depuis le scratchpad
  // addr[] : Adresse du module 1-Wire détecté
  
  /* Reset le bus 1-Wire ci nécessaire (requis pour la lecture du premier capteur) */
  if (reset_search)
    temp_sonde.reset_search();
 
  /* Recherche le prochain capteur 1-Wire disponible */
  if (!temp_sonde.search(addr))
    return NO_SENSOR_FOUND;  // Pas de capteur
  
  /* Vérifie que l'adresse a été correctement reçue */
  if (OneWire::crc8(addr, 7) != addr[7])
    return INVALID_ADDRESS;  // Adresse invalide
 
  /* Vérifie qu'il s'agit bien d'un DS18B20 */
  if (addr[0] != 0x28)
    return INVALID_SENSOR;  // Mauvais type de capteur
 
  /* Reset le bus 1-Wire et sélectionne le capteur */
  temp_sonde.reset();
  temp_sonde.select(addr);
  
  /* Lance une prise de mesure de température et attend la fin de la mesure */
  temp_sonde.write(0x44, 1);
  delay(800);
  
  /* Reset le bus 1-Wire, sélectionne le capteur et envoie une demande de lecture du scratchpad */
  temp_sonde.reset();
  temp_sonde.select(addr);
  temp_sonde.write(0xBE);
 
 /* Lecture du scratchpad */
  for (byte i = 0; i < 9; i++)
    data[i] = temp_sonde.read();
   
  /* Calcul de la température en degré Celsius -> pointeur */
  *temperature = (int16_t) ((data[1] << 8) | data[0]) * 0.0625; 
  
  return READ_OK;  // Pas d'erreur
}

/*=============================================== getMedianNum ===============================================*/
/* On fait une valeur mediane de la temperature afin de stabiliser les valeurs de TDS */
int getMedianNum(int bArray[], int iFilterLen) 
{
  int bTab[iFilterLen];
    
  for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];

  int i, j, bTemp;
  
  for (j = 0; j < iFilterLen - 1; j++){
    for (i = 0; i < iFilterLen - j - 1; i++){
      if (bTab[i] > bTab[i + 1]){
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

/*=============================================== displayColor ===============================================*/
/* Affiche une couleur sur la led, version cathode commune */
void displayColor(byte color) {
  digitalWrite(PIN_LED_R, bitRead(color, 2));  // bitRead -> On recupere la valeur du bit 3 
  digitalWrite(PIN_LED_G, bitRead(color, 1));  // bitRead -> On recupere la valeur du bit 2
  digitalWrite(PIN_LED_B, bitRead(color, 0));  // bitRead -> On recupere la valeur du bit 1
}

/*=============================================== displayValeursBT ===============================================*/
/* Envoie les valeurs mesurées sur le Serial */
void displayValeursSerial(float temperature, float tdsValue, float EC){
  
  /* Affiche la temperature */
  Serial.print(F("Temperature : "));
  Serial.print(temperature, 2);
  Serial.write(176); // Caractère degré
  Serial.print("C ; ");
  
  /* Affiche le taux de TDS */
  Serial.print("TDS : ");
  Serial.print(tdsValue,2);
  Serial.print(" ppm ; ");
  
  /* Affiche la conductivite */
  Serial.print("EC : ");
  Serial.print(EC,2);
  Serial.print(" mSiemens ; ");
}

/*=============================================== displayValeursBT ===============================================*/
/* Envoie les valeurs mesurées sur le Serial Bluetooth */
void displayValeursBT(float temperature, float tdsValue, float EC){
  
  /* Affiche la temperature */
  bluetoothSerial.print(temperature, 2);
  bluetoothSerial.print(',');
  
  /* Affiche le taux de TDS */
  bluetoothSerial.print(tdsValue, 2);
  bluetoothSerial.print('!');
  
  /* Affiche la conductivite */
  bluetoothSerial.print(EC, 2);
  bluetoothSerial.print(';');
}

/*=============================================== displayEtat ===============================================*/
/* Change la couleur en fonction de la qualite de l'eau */
void displayEtat(float tdsValue, float EC){
  if(tdsValue == 0 || EC == 0){
    displayColor(COLOR_CYAN);
    Serial.println(" Cyan");
  }
  if(tdsValue > 0 && tdsValue < 100 && EC < 0.3){
    displayColor(COLOR_GREEN);
    Serial.println(" Vert");
  }
  if(tdsValue >= 200 && tdsValue < 500 || EC >= 0.3){
    displayColor(COLOR_YELLOW);
    Serial.println(" Jaune");
  }
  if(tdsValue >= 500 || EC >= 0.5){
    displayColor(COLOR_RED);
    Serial.println(" Red");
  }
}
