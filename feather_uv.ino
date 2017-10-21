
//Sensor UV utilizado http://www.crcibernetica.com/electronic-brick-uv-sensor-brick/

#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include "passwords.h" //El archivo passwords.txt debe ser nombrado como passwords.h\
                         esto se hizo de tal forma para que al momento de subir a git\
                         no se compartieran datos sensibles

/*************************** FONA Pins ***********************************/

//Pines por defecto para el Feather Fona 32u4
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/************ Global State (you don't need to change this!) ******************/

// Inicialización de la clase MQTT_FONA que toma como argumentos la clase FONA
//y los parámetros de configuración de MQTT 
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//Líneas de confuguración para el archivo fonahelper.cpp proporcionado por Adafruit

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/
//Declaración y inicialización de los feeds que serán enviados por MQTT a io.adafruit.com

//  Notar que los paths de MQTT siempre son de la forma: <usuario/feeds/nombre_del_feed>

Adafruit_MQTT_Publish uv_index_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/uv_index");

Adafruit_MQTT_Publish feather_battery_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/feather_battery");

//Siempre que se haga una publicación y falle aumentaremos en 1 el contador
//Si logra realizar una publicación reiniciaremos el contador a 0.
uint8_t conteo_de_fallos = 0;

#define MAXFALLOS 5 //Cantidad máxima de fallos que vamos a permitir antes de reiniciar el sistema

//Último porcentaje de la batería tomado
uint16_t ultima_medicion_de_bateria;

uint32_t previousMillis_1 = 0;//Actualizar tiempo de último envío de batería

float lecturas_uv_sensor;

void setup() {
  
  Serial.begin(115200);
  //*********************************************************
  //La siguiente línea es solo para propósitos de desarrollo
  //DEBE SER COMENTADA UNA VEZ EL PROYECTO ESTÉ EN MARCHA
  //DE LO CONTRARIO EL PROGRAMA NO INICIARÁ
  //*********************************************************
  while (!Serial);//Iniciar el programa hasta que se habra el puerto serial
  Serial.println("Inicio de programa");

  //Un watchdog es un circuito aparte dentro del microncontrolador que verifica que nuestro\
  programa se encuentre corriende, si por algún motivo nuestro programa se detiene, este circuito\
  reiniciará nuestro microcontrolador ya que esto indica una posible falla. Por lo que su\
  contador debe reiniciarse cada 8s como máximo

  Watchdog.reset();//Reiniciamos el Watchdog para ya que si sobrepasa los 8s reinicia el programa\
                    y estamos por esperar 5s, debemos asegurarnos de poder esperar ese tiempo
  delay(5000);  //Se debe esperar unos momentos a que el SIM800 se estabilize
  Watchdog.reset();//Reiniciamos nuevamente el tiempo para poder continuar con el flujo del programa
  
  //Inicar la conección a la red celular
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  
  Watchdog.reset();  
  
}//end setup

void loop() {
  Watchdog.reset();//Siempre reinicar al inicio o al final de cada loop el watchdog

  //TODO Crear método para enviar las mediciones con estos datos a MQTT
//  float lecturas_Volts = lecturas_V_sensor_UV();
//  unsigned int indice = indice_UV(lecturas_Volts);
//  Serial.print(F("Indice de radiación UV = "));
//  Serial.println(indice);
//  delay(1000);//Esperar un segundo hasta la siguiente medición

  MQTT_connect();
  //Envío de prueba para verificar que se puede realizar la conexión a io.adadfruit.com
  //log_battery_percent(80, feather_battery_feed);//Tratar de enviar el mensaje por MQTT

  //La conexión debe temporizarse para no saturar el sitio de mensajes de forma inncesaria
  alarma1_enviar_bateria(millis(), 60000);//Actualizar Adafruit feeds cada 60s

  if(conteo_de_fallos >= MAXFALLOS){
    //Se llegó a la máxima cantidad de fallos por lo que se bloqueará el programa
    //de esta forma el Watchdog entrará en acción y reiniciará nuestro programa
    //para solucionar cualquier problema de conexión
    while(1);
  }//end if
  
}//end loop

float lecturas_V_sensor_UV(){//Las lecturas deben retornarse en Volts
  /*Este médoto convierte el muestreo realizado por el pin analógico del sensor
    a valores de tensión[V] para así luego poder determinar el indice UV
    Según las especificaciones del sensor la tensión de salida del sensor
    es lineal al indice UV
  */
  //Se realizará un muestreo de la señal entrante para tener una mejor aproximación
  long sensor_uv = 0;
  for(int i = 0; i < 1024; i++){  
    sensor_uv += analogRead(A0);//Tomar nueva medición del sensor y sumarla para crear un promedio
    delay(2);
  }//end for
  sensor_uv /= 1024;//Tomar las mediciones y dividirlas entre el total de mediciones
  float sensor_en_V = sensor_uv*(3/1024.0);//Voltage del sensor en V
  
  return sensor_en_V;//Retorna el valor en V
}//end lecturas_mV_sensor_uv

unsigned int get_indice_UV(float &lectura_V){//El valor se envia por referencia para no utilizar más\
                                         memoria de la necesaria
  /*Conversión aproximada para determinar el indice UV que incide sobre el sensor.
    El indice UV se puede aproximar mediante la expresión UV = (307*lectura_V)/200
  */

  //El indice UV es un número entero, pero la siguiente operación retornará un "float"
  //por lo que luego se tendrá que redondear el valor

  float indice_uv_crudo = (307*lectura_V)/200;

  unsigned int indice_uv = round(indice_uv_crudo);//Valor entero del indice UV
  
  return indice_uv;
}//end indice

void log_indice_uv(){}

int get_fona_battery(void){//Leer el porcentaje de la batería restante
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end get_fona_battery

void alarma1_enviar_bateria(uint32_t timer, uint32_t interval){
    uint16_t fona_battery = get_fona_battery();//Tomar una medición reciente de la batería

    //Se verificará si el porcentaje es igual al último enviado, de ser así no se enviará nuevamente\
    para ahorar espacio, ya que este es limitado al igual que la cantidad de envíos a adafruit
    if((fona_battery != ultima_medicion_de_bateria) && (fona_battery != 0)){
      ultima_medicion_de_bateria = fona_battery;//Actualizar al último valor de batería 
      Serial.print(F("Publicando porcentaje de batería: "));
      Serial.println(fona_battery);
      log_to_mqtt(fona_battery, feather_battery_feed);//Tratar de enviar el mensaje por MQTT
    }//end if
    previousMillis_1 = timer;
}//end temporizer

void log_to_mqtt(uint32_t indicador, Adafruit_MQTT_Publish& publishFeed){// Log battery
  if(!publishFeed.publish(indicador)){
    Serial.println(F("Publicación fallida!"));
    conteo_de_fallos++;
  }else{
    Serial.println(F("Publicación OK!"));
    conteo_de_fallos = 0;
  }//end if
}//end log_battery_percent


//Método creado por Adafruit
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
