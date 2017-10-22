
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

Adafruit_MQTT_Publish feather_asu_signal_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/feather_asu_signal");

//Siempre que se haga una publicación y falle aumentaremos en 1 el contador
//Si logra realizar una publicación reiniciaremos el contador a 0.
uint8_t conteo_de_fallos = 0;

#define MAXFALLOS 5 //Cantidad máxima de fallos que vamos a permitir antes de reiniciar el sistema

//Últimos valores de la batería y el sensor UV
uint16_t ultima_medicion_de_bateria;

uint16_t ultima_medicion_uv = 0;//Inicia en negativo para actualizar la medición a cero en caso\
                                 que se haya reiniciado y tenga una medición vieja en io.adafruit.com

uint32_t previousMillis_1 = 0;//Actualizar tiempo de último envío de batería
uint32_t previousMillis_2 = 0;//Actualizar tiempo de último envío de indice UV
uint32_t previousMillis_3 = 0;//Actualizar tiempo de último envío de indice UV

uint8_t led_conexion_mqtt = 13;
uint8_t estado_conexion_mqtt = 0;

void setup() {
  
  Serial.begin(115200);
  //*********************************************************
  //La siguiente línea es solo para propósitos de desarrollo
  //DEBE SER COMENTADA UNA VEZ EL PROYECTO ESTÉ EN MARCHA
  //DE LO CONTRARIO EL PROGRAMA NO INICIARÁ
  //*********************************************************
//  while (!Serial);//Iniciar el programa hasta que se habra el puerto serial
  Serial.println("Inicio de programa");

  pinMode(OUTPUT, led_conexion_mqtt);

  //Un watchdog es un circuito aparte dentro del microncontrolador que verifica que nuestro\
  programa se encuentre corriende, si por algún motivo nuestro programa se detiene, este circuito\
  reiniciará nuestro microcontrolador ya que esto indica una posible falla. Por lo que su\
  contador debe reiniciarse cada 8s como máximo

  Watchdog.enable(8000);
  Watchdog.reset();//Reiniciamos el Watchdog para ya que si sobrepasa los 8s reinicia el programa\
                    y estamos por esperar 5s, debemos asegurarnos de poder esperar ese tiempo
  delay(5000);  //Se debe esperar unos momentos a que el SIM800 se estabilize
  Watchdog.reset();//Reiniciamos nuevamente el tiempo para poder continuar con el flujo del programa
  
  //Inicar la conección a la red celular
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));
  
}//end setup

void loop() {
  Watchdog.reset();//Siempre reinicar al inicio o al final de cada loop el watchdog

  MQTT_connect();
  //Envío de prueba para verificar que se puede realizar la conexión a io.adadfruit.com
  //log_battery_percent(80, feather_battery_feed);//Tratar de enviar el mensaje por MQTT
  
  if(conteo_de_fallos >= MAXFALLOS){
    //Se llegó a la máxima cantidad de fallos por lo que se bloqueará el programa
    //de esta forma el Watchdog entrará en acción y reiniciará nuestro programa
    //para solucionar cualquier problema de conexión
    Watchdog.enable(100);
    Watchdog.reset();
    while(1);//Este loop infinito correrá durante 100ms y el programa se reiniciará
  }//end if

  if(estado_conexion_mqtt){
    digitalWrite(led_conexion_mqtt, HIGH);
  }

  //La conexión debe temporizarse para no saturar el sitio de mensajes de forma inncesaria
  alarma1_enviar_bateria(millis(), 60000);//Actualizar Adafruit feeds cada 60s
  alarma2_enviar_indice_uv(millis(), 60000);//Actualizar Adafruit feeds cada 60s
  alarma3_enviar_asu_signal(millis(), 30000);//Actualizar Adafruit feeds cada 60s
  
}//end loop

//===============================================================

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

int get_fona_battery(void){//Leer el porcentaje de la batería restante
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end get_fona_battery

int8_t get_network_asu(void){
  //Arbitrary Strength Unit (ASU) is an integer value proportional to the received signal strength measured by the mobile phone.
  uint8_t asu = fona.getRSSI();

  #if defined(DEBUG)
    int8_t r;
    if(asu == 0){
      r = -115;
    }else if(asu == 1){
      r = -111;
    }else if(asu == 31){
      r = -52;
    }else if ((asu >= 2) && (asu <= 30)){
      r = map(asu, 2, 30, -110, -54);
    }//end if
    char str_rssi[25];
    sprintf(str_rssi, "RSSI = %i: %i dBm", asu, r);
//    Serial.println(str_rssi);
//     serial.print(F("RSSI = ")); serial.print(asu); serial.print(": ");
//     serial.print(r); serial.println(F(" dBm"));
  #endif
  return asu;
}//end get_network_asu

//===============================================================

void alarma1_enviar_bateria(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_1 > interval) {
    uint16_t fona_battery = get_fona_battery();//Tomar una medición reciente de la batería

    //Se verificará si el porcentaje es igual al último enviado, de ser así no se enviará nuevamente\
    para ahorar espacio, ya que este es limitado al igual que la cantidad de envíos a adafruit
    if((fona_battery != ultima_medicion_de_bateria) && (fona_battery != 0)){
      ultima_medicion_de_bateria = fona_battery;//Actualizar al último valor de batería 
      Serial.print(F("Publicando porcentaje de batería: "));
      Serial.println(fona_battery);
      int verificacion_envio = log_to_mqtt(fona_battery, feather_battery_feed);//Tratar de enviar el mensaje por MQTT
      //Si el envío no se realiza siempre voy a querer que cuando se vuelva a intentar realizar el envío
      //el valor anterior y el nuevo valor no sean iguales para así enviarlo
      if(verificacion_envio == EXIT_FAILURE){
        ultima_medicion_de_bateria = 0;
      }//end if      
    }else{
      //Esta línea se utilizará únicamente para propósitos de información
      char buffer[60];
      sprintf(buffer, "Medición duplicada de bateria: %i, no hay cambios", fona_battery);
      Serial.println(buffer);
    }//end if
    previousMillis_1 = timer;
  }//end if
}//end alarma1_enviar_bateria

void alarma2_enviar_indice_uv(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_2 > interval) {
    float lecturas_Volts = lecturas_V_sensor_UV();
    unsigned int uv_index = get_indice_UV(lecturas_Volts);
//    if((uv_index != ultima_medicion_uv) && (uv_index != 0)){//El indice inicia en 1 por lo que no se\
                                                              enviaran valores menores a 1
      Serial.print(F("Publicando Indice de radiación UV: "));
      Serial.println(uv_index);
      int verificacion_envio = log_to_mqtt(uv_index, uv_index_feed);
      //Si el envío no se realiza siempre voy a querer que cuando se vuelva a intentar realizar el envío
      //el valor anterior y el nuevo valor no sean iguales para así enviarlo
      if(verificacion_envio == EXIT_FAILURE){
        ultima_medicion_uv = 0;
      }//end if
//    }else{
//      //Esta línea se utilizará únicamente para propósitos de información
//      char buffer[60];
//      sprintf(buffer, "Medición duplicada UV: %i, no hay cambios", uv_index);
//      Serial.println(buffer);
//    }//end if
    previousMillis_2 = timer;
  }//end if
}//end alarma2_enviar_indice_uv

void alarma3_enviar_asu_signal(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_3 > interval) {
    int32_t feather_asu_signal = get_network_asu();
    Serial.print(F("Publicando señal ASU: "));
    Serial.println(feather_asu_signal);
    log_to_mqtt(feather_asu_signal, feather_asu_signal_feed);
    previousMillis_3 = timer;
  }//end if
}//end alarma1_enviar_bateria

//===============================================================

unsigned int log_to_mqtt(int32_t indicador, Adafruit_MQTT_Publish& publishFeed){
  //Método generalizado para enviar datos a mqtt, de esta forma no hay réplicas del mismo método\
  simplifica el mantenimiento del programa
  if(!publishFeed.publish(indicador)){
    Serial.println(F("Publicación fallida!"));
    conteo_de_fallos++;
    return EXIT_FAILURE;
  }else{
    Serial.println(F("Publicación OK!"));
    conteo_de_fallos = 0;
  }//end if
  return EXIT_SUCCESS;
}//end log_battery_percent

void MQTT_connect(){
  /* Function to connect and reconnect as necessary to the MQTT server.
 Should be called in the loop function and it will take care if connecting.*/
  int8_t connection_status;
  // Stop if already connected.
  if (mqtt.connected()){
    estado_conexion_mqtt = 1;
    return;
  }//end if

  if(!estado_conexion_mqtt){
    digitalWrite(led_conexion_mqtt, LOW);
  }

  uint8_t network_status = fona.getNetworkStatus();
  if(network_status == 0){
    // debug_print(F("WARNING"), F("Not registered"), true);
    Serial.println(F("Not registered"));
//    return EXIT_FAILURE;
  }else if(network_status == 1){
    // debug_print(F("STATUS"), F("Registered (home)"), true);
    Serial.println(F("Registered (home)"));
//    return EXIT_SUCCESS;
  }else if(network_status == 2){
    // debug_print(F("WARNING"), F("Not registered (searching)"), true);
    Serial.println(F("Not registered (searching)"));
//    return EXIT_FAILURE;
  }else if(network_status == 3){
    // debug_print(F("ERROR"), F("Denied"), true);
    Serial.println(F("Denied"));
//    return EXIT_FAILURE;
  }else if(network_status == 4){
    // debug_print(F("WARNING"), F("Unknown"), true);
    Serial.println(F("Unknown"));
//    return EXIT_FAILURE;
  }else if(network_status == 5){
    // debug_print(F("WARNING"), F("Registered roaming"), true);
    Serial.println(F("Registered roaming"));
//    return EXIT_FAILURE;
  }//end if
  
  Serial.print("Connecting to MQTT... ");

  for(uint8_t i = 0; i < 10; i++){//Arbitrary elected 10 times
    if((connection_status = mqtt.connect()) == 0){//connect will return 0 for connected
        Serial.println("MQTT Connected!");
        estado_conexion_mqtt = 1;
        conteo_de_fallos = 0;
      break;
    }else{
      Serial.println(mqtt.connectErrorString(connection_status));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      little_blink();//Indicador de que tendrá que reintentar conectarse a MQTT
      estado_conexion_mqtt = 0;
      conteo_de_fallos++;
      Watchdog.reset();
      delay(5000); // wait 5 seconds
    }//end if
  }//end for
}//end MQTT_connect

void little_blink(){
  //Este método nos permitirá conocer si el Feather está intentando conectarse a Adafruit
  //En caso de que el LED se encuentre fijo indicará una conexión permanente
  digitalWrite(led_conexion_mqtt, LOW);
  delay(200);
  digitalWrite(led_conexion_mqtt, HIGH);
  delay(200);
  digitalWrite(led_conexion_mqtt, LOW);
}//end little_blink











