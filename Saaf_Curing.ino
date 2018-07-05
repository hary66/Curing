

/**************************************************************************/
/*!
    @file     trianglewave.pde
    @author   Adafruit Industries
    @license  BSD (see license.txt)

    This example will generate a triangle wave with the MCP4725 DAC.

    This is an example sketch for the Adafruit MCP4725 breakout board
    ----> http://www.adafruit.com/products/935

    Adafruit invests timer and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
*/
/**************************************************************************/
#include <Wire.h> //  pour l'I2C avec SDA sur pin A4 et scl sur A5
#include <Adafruit_MCP4725.h> //  I2C device
#include <SPI.h>  //  pour le SPI
#include "Adafruit_MAX31855.h"  //  SPI device
#include <Adafruit_ADS1015.h>
#include <PID_v1.h> //  bibliothèque régulateur PID


#define ledPin 13  //  pour débuguer

// Example creating a thermocouple instance with software
//SPI on any three digital IO pins.
#define MAXDO   4
#define MAXCS   2
#define MAXCLK  3
// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

enum sortie_capteur {Tc_K_ON, Pyro_ON};
sortie_capteur capteur_en_service = Tc_K_ON;
String capteur[2] = {"Tc_K_ON", "Pyro_ON"};

struct Temp {
  double Tc_K;              //  en °C
  double internal_C;        //  en °C
  double T_pyro;            //  en °C
  double feedback;          //  en °C
  double adc1_Voltage;            //  en V
  double adc2_Voltage;            //  en V
  sortie_capteur flag_pyro;
  unsigned int nb_of_errors; //  erreur sur lecture thermocouple
};


//  ###Déclaration des variables###
unsigned long now = 0;
bool DEBUG = false;
int timer_led = 150;
int taux_rafraichissement_Serial_monitor = 1000; // période de rafraichissement du moniteur série
int taux_rafraichissement_output (200);   // période de rafraichissement sortie commande
bool etat_pin_debug = false;
double c = 0;
//double internal_C = 0;
Temp temperatures;  //  struct Temp{double Tc_K; double internal_C; double T_pyro; double feedback; sortie_capteur flag_pyro; };
int Treatment = 0; //  type de traitement
int sortie_MCP = 0;
int RUN = 0;
unsigned int timer_update_sortie_MCP = 10;
int MCP_step = 5;
//  ################# PID ###################
//Define Variables we'll be connecting to
double setpoint = 0.0;
double input = 0;
double output = 0;
//Specify the links and initial tuning parameters
double Kp = 2;
double Ki = 0;
const double Kd = 0;
//char[] POn = P_ON_E);  //  pour les paliers
char* POn = P_ON_M; //   Proportional on Measurement (pour les rampes)
//PID myPID(&feedback, &output, &Consigne, Kp, Ki, Kd, POn, DIRECT);
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//  ##############################################

Adafruit_MCP4725 dac;   //avec SDA sur pin A4 et scl sur A5
Adafruit_ADS1115 ads(0x48); //  default I2C address (0x48)


//  ############" déclaration des fonctions #################
void setup_DAC_MCP4725(byte const &address);
void setup_ADC_ADS1115();
void set_PID();
Temp readTemp(const unsigned int &timer); //  lit les température ambiante et TC_K (module MAS18855)
void updateMonitor(int const &taux_rafraichissement);

void updateLed(int duree_blink);
int lecture_potar();
//int get_feedback(const unsigned long &now, unsigned long &last_lecture_feedback, const int timer);
void DoAllWhatNeeded();
void update_sortie_MCP(unsigned int timer);
void DoGradient(double const &temp_initiale, int const &rampe, unsigned int const &temp_finale);
void DoCooling(double const & temp_initiale, int const & rampe, unsigned int const & temp_finale);
void DoPalier(unsigned int const &duree_palier, unsigned int const &temp_palier);
void AskForParameter();
void WaitForParameter();
void updateMCP4725(double const &sortie);

void setup(void) {
  pinMode(A0, INPUT);
  //  pinMode(10, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Hello!");
  setup_DAC_MCP4725(0x60);  //  MCP4725 at 0x60 I2C adress
  setup_ADC_ADS1115(); //    ADS1115 at 0x48 I2C adress gain2/3
  dac.setVoltage(0, false);
  set_PID();
  AskForParameter();
  readTemp(0);
  Serial.print("temperatures.feedback : ");
  Serial.println(temperatures.feedback);
}

void loop(void) {
  static int temp_palier = 0,
             rampe = 0,
             duree_palier = 0,
             rampe_cooling = 0,
             rampe_cooling1 = 0,
             palier_intermediaire = 0,
             rampe_cooling2 = 0,
             temp_fin_de_cycle = 0;

  now = millis();

  if (!Treatment) {
    WaitForParameter();
  }
  if (Treatment == 1) {
    temp_palier = 1050;
    rampe = 24;
    duree_palier = 35;
    rampe_cooling1 = 30;
    palier_intermediaire = 900;
    rampe_cooling2 = 17;
    temp_fin_de_cycle = 150;
    RUN = 1;
  } else if (Treatment == 2) {
    temp_palier = 795;
    rampe = 24;
    duree_palier = 70;
    rampe_cooling = 16;
    temp_fin_de_cycle = 150;
    RUN = 1;
  }
  switch (RUN) {
    case 1 :
      DoGradient(temperatures.feedback, rampe, temp_palier);
      break;
    case 2 :
      DoPalier(temp_palier, duree_palier);
      break;
    case 3 :
      if (rampe_cooling > 0) {
        rampe_cooling = -rampe_cooling;
      }
      DoGradient(temperatures.feedback, rampe_cooling, temp_fin_de_cycle);
      break;
    case 4 :
      if (rampe_cooling1 > 0) {
        rampe_cooling1 = -rampe_cooling1;
      }
      DoGradient(temperatures.feedback, rampe_cooling1, palier_intermediaire);
      break;
    case 5 :
      if (rampe_cooling2 > 0) {
        rampe_cooling2 = -rampe_cooling2;
      }
      DoGradient(temperatures.feedback, rampe_cooling2, temp_fin_de_cycle);
      break;
    default:
      break;
  }

  updateLed(timer_led);
  Serial.println (" #####   BOUCLE LOOP !   ############     ");
}

//  ###############################################################
//  ##########  définition des fonctions    #######################
//  ###############################################################

//  ###############################################################
//  ##########            Setup function             ##############
//  ###############################################################
void  setup_DAC_MCP4725(byte const & address) {
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  //dac.begin(0x60); // adresse 96 (0x60) pour mes MCP4725 chinois.
  dac.begin(address); // adresse 96 (0x60) pour mes MCP4725 chinois.
}
void  setup_ADC_ADS1115() {
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
}
void set_PID() {
  //PID initialize the variables we're linked to
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(0, 1.570796327);  //  entre 0 et PI/2
  //myPID.SetOutputLimits(0, 255);  //  entre 0 et 255 (valeurs par defaut)
  //myPID.SetOutputLimits(0, 4095);  //  entre 0 et 4095, (la sortie du MCP4725)
  myPID.SetOutputLimits(0, 255);  //  entre 0 et 4095, (la sortie du MCP4725)

}
//  ###############################################################
//  ##########            Fonctions          #######################
//  ###############################################################

Temp readTemp(const unsigned int &timer) { //  renvoie un struct Temp !
  static unsigned int last_read_temp;
  if (now - last_read_temp >= timer) {
    int16_t adc0, adc1, adc2;//, adc3;
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    //        adc3 = ads.readADC_SingleEnded(3);
    double Tc_K_transitoire =  thermocouple.readCelsius();
    static double Tc_K_last;
    if (!isnan(Tc_K_transitoire)) {
      temperatures.Tc_K = Tc_K_transitoire;
      Tc_K_last = Tc_K_transitoire;
    } else if (isnan(Tc_K_transitoire)) {
      temperatures.Tc_K = Tc_K_last;
      static unsigned int ERR = 0;
      ERR ++;
      temperatures.nb_of_errors = ERR;
      Serial.println("");
      Serial.print("nb_of_errors");
      Serial.println(temperatures.nb_of_errors);
    }

    temperatures.internal_C = thermocouple.readInternal();
    // ####### calcul la pente et l'ordonnée a l'origine de la fonction de la sortie du pyro en fonction de 2 points
    typedef double Point[2];
    Point A = {0, 250}, B{4, 1200};
    static double pente = (B[1] - A[1]) / (B[0] - A[0]);
    static double offset = A[1] - A[0] * pente;
    //double const ads_step = 0.125;  //  mV/bit
    double const ads_step = 0.1875;  //  mV/bit
    double adc0_voltage = (adc0 * ads_step) / 1000;
    //  2 points A(250; 1) et B(1250; 5) y= a*adc0_voltage + b  a=(5-1)/(1250-250), a=1/250 1=250a+b, b=1-4a, b=1-4*1/250=0.984
    temperatures.T_pyro = adc0_voltage * pente + offset; //250°C=>4mA=>0.8V, 1200°C=>20mA=>4V (résistance de Shunt 200 Ohm pour 4Vmax@20mA
    //    if (DEBUG) {
    //      Serial.print(" m : ");
    //      Serial.print(m, 6);
    //      Serial.print("  p : ");
    //      Serial.print(p, 6);
    //      Serial.print("  adc0 : ");
    //      Serial.print(adc0);
    //      Serial.print("  adc0_voltage : ");
    //      Serial.print(adc0_voltage);
    //      Serial.print("  temperatures.T_pyro : ");
    //      Serial.println(temperatures.T_pyro);
    //      Serial.println();
    //      Serial.print(">");
    //      Serial.print("\t");
    temperatures.adc1_Voltage = (adc1 * ads_step) / 1000;
    temperatures.adc2_Voltage = (adc2 * ads_step) / 1000;
    //    Serial.print("\tpente : ");
    //    Serial.print(pente);
    //    Serial.print("\toffset : ");
    //    Serial.print(offset);
    //    Serial.print("\tadc1_V : ");
    //    Serial.println(temperatures.adc1_Voltage);
    //    Serial.print("\tadc2_V : ");
    //    Serial.println(temperatures.adc2_Voltage);
    //      Serial.print("\t");
    //      Serial.println();
    //    }
    if (temperatures.T_pyro > 253) {
      temperatures.feedback = temperatures.T_pyro;
      temperatures.flag_pyro = Pyro_ON;
    } else {
      temperatures.feedback = temperatures.Tc_K;
      //temperatures.feedback = thermocouple.readCelsius();
      temperatures.flag_pyro = Tc_K_ON;
    }
    // temperatures.feedback = analogRead(A0);
    last_read_temp = now;
  }
  //Serial.println("readTemp");
  return temperatures;
}

// ####### Rafraichit l'affichage moniteur #######
void updateMonitor(int const & taux_rafraichissement) { // ####### Rafraichit l'affichage moniteur #######
  static unsigned long last_update_monitor = 0;
  int timer = now - last_update_monitor;
  if (timer > taux_rafraichissement) {
    if (isnan(c)) {
      Serial.println("");
      Serial.println(F("Something wrong with thermocouple!"));
      //Serial.println("Something wrong with thermocouple!#######################################################");
    } else {
      Serial.print("<\t");
      Serial.print(now);
      Serial.print("\t");
      Serial.print(timer);
      Serial.print("\t");
      Serial.print(temperatures.internal_C);
      Serial.print("\t");
      Serial.print(temperatures.Tc_K);
      Serial.print("\t");
      Serial.print(setpoint);
      Serial.print("\t");
      Serial.print(input);
      Serial.print("\t");
      Serial.print(temperatures.flag_pyro);
      Serial.print("\t");
      //Serial.print("temperatures.T_pyro : ");
      Serial.print(temperatures.T_pyro);
      Serial.print("\t");
      Serial.print(sortie_MCP); //  0<MCP4725<4095
      Serial.print("\t");
      Serial.print(temperatures.adc1_Voltage);
      Serial.print("\t");
      Serial.println(temperatures.nb_of_errors);
      //      Serial.print("\t");
      //      Serial.println();
      last_update_monitor = now; //  pour le serial monitor
    }
  }
}

void updateMCP4725(double const & sortie) {
  static unsigned long last_update_MCP4725 = 0;
  int timer = now - last_update_MCP4725;
  if (timer > taux_rafraichissement_output) {
    dac.setVoltage(sortie, false);
    last_update_MCP4725 = now;
    //Serial.println("updateMCP4725 Updated Output");
  }
}

void DoGradient(double const &temp_initiale, int const &rampe, unsigned int const &temp_finale) {
  static bool INIT = true;
  static double rampe_ms = 0;
  static unsigned long duree_rampe = 0;
  static unsigned long start_time_gradient = 0;
  static double temp_depart = 0;
  static double temp_fin = 0;

  if (INIT) {
    rampe_ms = rampe / (60.000 * 1000); // rampe en °C/ms au lieu de °C/min
    //  Serial.print("rampe_ms : ");
    //  Serial.print(rampe_ms);
    temp_depart = temp_initiale;
    Serial.print("\ttemp_depart : ");
    Serial.println(temp_depart);
    temp_fin = temp_finale;
    duree_rampe = (temp_fin - temp_depart) / rampe_ms; //  en ms
    //  Serial.print("    duree_rampe_ms : ");
    //  Serial.print(duree_rampe);
    start_time_gradient = now;
    timer_led = 500;
    INIT = !INIT;
  }
  unsigned long elapsed_time = now - start_time_gradient;
  //    Serial.print("    elapsedd_time : ");
  //    Serial.println(elapsed_time);
  setpoint = temp_depart + elapsed_time * rampe_ms;
  if (setpoint > temp_fin) {
    setpoint = temp_fin;
  }
  DoAllWhatNeeded(now);
  //    Serial.print("DoGradient; now = ");
  //    Serial.print(now);
  //    Serial.print("    remaining_time gradient(ms) :  ");
  long  remaining_time = duree_rampe - elapsed_time;
  if (remaining_time > 0) {
    Serial.print("remaining_time : ");
    Serial.print(remaining_time);
  } else {
    Serial.print("Timer over gradient!");
    RUN ++;
    INIT = true;
    if (temp_fin == 900) {
      RUN ++;
    } else  if (temp_fin == 150) {
      RUN = 0;
      timer_led = 250;
    }
    Serial.print("\tRUN gradient= ");
    Serial.println(RUN);
    Serial.println("\tDoGradient endded");
  }
}

void DoPalier(unsigned int const &duree_palier, unsigned int const &temp_palier) {
  static bool INIT = true;
  static double duree_ms = 0;
  static unsigned long start_time = 0;
  if (INIT) {
    Serial.print("duree_min_palier: ");
    Serial.println(duree_palier);
    duree_ms = duree_palier * (60.0 * 1000); // duree du palier en ms
    Serial.print("duree_ms_palier: ");
    Serial.println(duree_ms);
    start_time = now;
    INIT = ! INIT;
  }
  unsigned long elapsed_time = now - start_time;
  setpoint = temp_palier;
  DoAllWhatNeeded(now);
  Serial.print("DoPalier; now = ");
  Serial.print(now);
  Serial.print("    remaining_time palier(ms) :  ");
  long  remaining_time = duree_ms - elapsed_time;
  if (remaining_time > 0) {
    Serial.println(remaining_time);
  } else {
    Serial.println("Timer over palier !");
    RUN ++;
    Serial.print("RUN_palier= ");
    Serial.println(RUN);
    Serial.println("DoPalier endded");
  }
}

void DoAllWhatNeeded(const unsigned long & now) {
  readTemp(150);  //  lit les température ambiante, TC_K (module MAX18855) et pyromètre et switch feedback sur le capteur adéquate
  input = temperatures.feedback;
  myPID.Compute();
  Serial.print("\tmyPID.Compute();");
  //  sortie_MCP = int(round(cos(output/255) * 4095));
  //sortie_MCP = int(round(output));
  //int sortie_commande = int(round((sin(output / 162.338041954)) * 4095));
  update_sortie_MCP(timer_update_sortie_MCP);
  Serial.println();
  Serial.print("feedback : ");
  Serial.print(input);
  Serial.print("\tConsigne : ");
  Serial.print(setpoint);
  Serial.print("\toutput : ");
  Serial.print(output);
  Serial.print("\tsortie_MCP : ");
  Serial.println(sortie_MCP);
  updateMonitor(taux_rafraichissement_Serial_monitor);  // ####### Rafraichit l'affichage moniteur #######
  updateMCP4725(sortie_MCP); // myPID.Compute() return True if something has changed, False if nothing has been done
  //Serial.println("DoAllWhatNeeded");
}

void update_sortie_MCP(unsigned int timer) {
  static unsigned long last_update_sortie_MCP = 0;
  int sortie_commande = int(round((sin(output / 162.338041954)) * 4095));
  if (now - last_update_sortie_MCP > timer_update_sortie_MCP) {
    if (abs(sortie_MCP - sortie_commande) <= MCP_step) {
    }
    else if (sortie_MCP < sortie_commande) {
      sortie_MCP += MCP_step;
    } else if (sortie_MCP > sortie_commande) {
      sortie_MCP -= MCP_step;
    }
    last_update_sortie_MCP = now;
  }
}

void AskForParameter() {
  Serial.println(F("  Type of heat Treatment ?"));
  Serial.println(F("  A for austenitizing (cycle Nassheuer)"));
  Serial.println(F("  T for tempering (cycle Sottri)"));
}

void WaitForParameter() {
  char Input = ('\0');
  Serial.println(Input);
  while (Input != 'a' || Input != 'A' || Input != 't' || Input != 'T') {
    //updateLed(now, 100);
    if (Serial.available()) {
      Input = (Serial.read());
      Serial.println(Input);
      if (Input == 97 || Input == 65) { //  char(97)="a", char(65)="A"
        Serial.println(F("  Launching austenitizing (cycle Nassheuer)....\n"));
        Treatment = 1;
        break;
      } else if (Input == 116 || Input == 84) { //  char(116)="t", char(84)="T"
        Serial.println(F("  Launching tempering (cycle Sottri)....\n"));
        Treatment = 2;
        break;
      } else {
        Serial.println(F("  Please, make a valid choice to start !....\n"));
        AskForParameter();
      }
    }
  }
}

//  ######  For debug purpose only  ##########
void updateLed(int duree_blink) {
  static unsigned long last_blink = 0;
  int timer = now - last_blink;
  if ( timer > duree_blink) {
    etat_pin_debug =  !etat_pin_debug;
    digitalWrite(ledPin, etat_pin_debug);
    last_blink = now;
    Serial.print("depuis last Led blink : ");
    Serial.println(timer);
  }
}
