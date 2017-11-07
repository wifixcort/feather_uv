
//Sensor UV utilizado http://www.crcibernetica.com/electronic-brick-uv-sensor-brick/


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

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

/************************Neopixel Pins***********************************/
//Pin en el que se encuentran conectados los neopixeles
#define LED_PIN 12
#define BUTTON_PIN 11 //Pin en el que se encuentra el botón
#define SIGNAL_PIN  10

//Se agregó una resistencia entre GND y el pin 11 para evitar ruidos

#define NUMPIXELS 5 //Cantidad de Neipixeles


SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);


// Inicialización de la clase MQTT_FONA que toma como argumentos la clase FONA
//y los parámetros de configuración de MQTT 
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

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

#define NETOWRK_CONCT_TRY 40 //No intentar más de 40 veces una reconexion de GPRS

//Últimos valores de la batería y el sensor UV
uint16_t ultima_medicion_de_bateria;

uint16_t ultima_medicion_uv = 0;//Inicia en negativo para actualizar la medición a cero en caso\
                                 que se haya reiniciado y tenga una medición vieja en io.adafruit.com

uint32_t previousMillis_1 = 0;//Actualizar tiempo de último envío de batería
uint32_t previousMillis_2 = 0;//Actualizar tiempo de último envío de indice UV
uint32_t previousMillis_3 = 0;//Actualizar tiempo de último envío de indice UV

uint8_t led_conexion_mqtt = 13;

int buttonState = 0;  //Variable para almacenar el estado del botón

//Inicialización del objeto NeoPixel
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

int delayval = 250;//Intervalo para prender cada led

//Array de colores en formato uint32_t que indican el indice de radiación
int32_t ledIndex[15] = {0, 3302400, 65280, 16777010, 13158400, 16776960, 6566400, 6566400, 16724530, 13107200, 16711680, 13107400};


void little_blink(int16_t t_delay, uint8_t num_blink = 4);

void setup() {  
  Serial.begin(115200);
  //*********************************************************
  //La siguiente línea es solo para propósitos de desarrollo
  //DEBE SER COMENTADA UNA VEZ EL PROYECTO ESTÉ EN MARCHA
  //DE LO CONTRARIO EL PROGRAMA NO INICIARÁ
  //*********************************************************
//  while (!Serial);//Iniciar el programa hasta que se habra el puerto serial
  Serial.println("Inicio de programa");
  
  //Inicialización de pines
  pinMode(led_conexion_mqtt, OUTPUT);
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(SIGNAL_PIN, HIGH);//Este pin es simplemente una señal en alto para el pin 11
  
  pixels.begin(); //Configuración inicial de los Neopixeles
  pixels.setBrightness(80);//Disminución del brillo

  /*Un watchdog es un circuito aparte dentro del microncontrolador que verifica que nuestro
  programa se encuentre corriende, si por algún motivo nuestro programa se detiene, este circuito
  reiniciará nuestro microcontrolador ya que esto indica una posible falla. Por lo que su
  contador debe reiniciarse cada 8s como máximo*/

  delay(5000);  //Se debe esperar unos momentos a que el SIM800 se estabilize

  little_blink(150);//Indicador de inicio
  
  iniciar_FONA();
  
}//end setup

void loop() {
  
  //La conexión debe temporizarse para no saturar el sitio de mensajes de forma inncesaria
  alarma1_enviar_bateria(millis(), 60000);//Actualizar Adafruit feeds cada 60s
  alarma2_enviar_indice_uv(millis(), 60000);//Actualizar Adafruit feeds cada 60s
  alarma3_enviar_asu_signal(millis(), 30000);//Actualizar Adafruit feeds cada 60s

  buttonState = digitalRead(BUTTON_PIN);//Leer el estado del botón
  if (buttonState == HIGH){//Verificar si el botón se encuentra presionado
    float lecturas_Volts = lecturas_V_sensor_UV();
    unsigned int uv_index = get_indice_UV(lecturas_Volts);    
    setLedIndex(uv_index);//Establecimiento del indice de Radiación(Prueba de diseño)
  }else{//Si el botón no está presionado, apagar los leds
    setLedIndex(0);//Apagar todos los neopixeles
  }//end if

  
}//end loop

//===============================================================
//===============================================================
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

int get_fona_battery(){//Leer el porcentaje de la batería restante
  uint16_t vbat;
  fona.getBattPercent(&vbat);
  return vbat;
}//end get_fona_battery

int8_t get_network_asu(){
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
  //En cuanto a las alarmas cada tiempo definido por la variable interval estos método ejecutará su código \
  de otra forma no ejecutará nada evitando perder tiempo de forma innecesaria

void alarma1_enviar_bateria(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_1 > interval) {
    Serial.println(F("Alarma #1"));
    verificador_de_conexion();//Antes de iniciar verificar todas las conexiones se encuentran en orden    
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
    Serial.println(F("Alarma #2"));    
    verificador_de_conexion();//Antes de iniciar verificar todas las conexiones se encuentran en orden
    float lecturas_Volts = lecturas_V_sensor_UV();
    unsigned int uv_index = get_indice_UV(lecturas_Volts);
    Serial.print(F("Publicando Indice de radiación UV: "));
    Serial.println(uv_index);
    int verificacion_envio = log_to_mqtt(uv_index, uv_index_feed);
    //Si el envío no se realiza siempre voy a querer que cuando se vuelva a intentar realizar el envío
    //el valor anterior y el nuevo valor no sean iguales para así enviarlo
    if(verificacion_envio == EXIT_FAILURE){
      ultima_medicion_uv = 0;
    }//end if
    previousMillis_2 = timer;
  }//end if
}//end alarma2_enviar_indice_uv

void alarma3_enviar_asu_signal(uint32_t timer, uint32_t interval){
  if(timer - previousMillis_3 > interval) {
    Serial.println(F("Alarma #3"));    
    verificador_de_conexion();//Antes de iniciar verificar todas las conexiones se encuentran en orden
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
  //Esta funcion permite la conexion y reconexión a MQTT en caso de problemas de RED
  int8_t connection_status;

  if (mqtt.connected()){//Salir si ya se está conectado a MQTT
    digitalWrite(led_conexion_mqtt, HIGH);//
    return;
  }//end if

  //Si la ejecuacion llega hasta acá tenemos problemas de conexión
  digitalWrite(led_conexion_mqtt, LOW);//Apagar el LED
  
  Serial.print("Connecting to MQTT... ");

  for(uint8_t i = 0; i < 10; i++){//Se elegió intentar intentar un máximo de 10, de forma arbitraria\
                                    puede aumentarse o disminuirse a gusto    
    if((connection_status = mqtt.connect()) == 0){//Si se está conextado a la red
      Serial.println("MQTT Connected!");
      digitalWrite(led_conexion_mqtt, HIGH);
      conteo_de_fallos = 0;//Reiniciar la cantidad de fallos
      break;//Ya no es necesario continuar en el "for" porque ya estamos conectados
    }else{
      Serial.println(mqtt.connectErrorString(connection_status));
      Serial.println("Retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      little_blink(250, 20);//Indicador de que tendrá que reintentar conectarse a MQTT
      conteo_de_fallos++;
//      delay(5000); // wait 5 seconds
    }//end if
  }//end for
}//end MQTT_connect

void verificador_de_conexion(){
  if(estado_de_red() == EXIT_FAILURE){//Verificar si estamos conectados a la RED Kolbi
    Serial.println(F("Problemas de conexion de red, reconectando..."));
    digitalWrite(led_conexion_mqtt, LOW);
    conexion_fona_kolbi();
  }//end if  
  if(!fona.GPRSstate()){//Verificar el estado de GPRS
    Serial.println(F("Problemas de señal de GPRS"));
    digitalWrite(led_conexion_mqtt, LOW);//Estamos claramente desconectados de MQTT apagamos el LED
    Serial.println(F("Desabilitando GPRS"));
    fona.enableGPRS(false);
    delay(5000);//Esperar unos segundos para que la conexión se estabilice
    Serial.println(F("Habilitando GPRS"));
    if (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to turn GPRS on"));  
      reboot(F("ERROR GPRS. es necesario reiniciar"));
    }//end if
  }//end if
  if(!fona.TCPconnected()){//Si no se tiene conexión por TCP no estaremos conectados a MQTT
    Serial.println(F("Fallo de conexión a MQTT, reconectando..."));
    MQTT_connect();//Reconectar a MQTT
  }else{
    digitalWrite(led_conexion_mqtt, HIGH);
  }//end if
  if(conteo_de_fallos >= MAXFALLOS){
    //Se llegó a la máxima cantidad de fallos por lo que se bloqueará el programa
    //de esta forma el Watchdog entrará en acción y reiniciará nuestro programa
    //para solucionar cualquier problema de conexión    
    reboot(F("Máxima cantidad de fallos al enviar a MQTT"));
  }//end if  
}//end verificador_de_conexion

uint8_t estado_de_red(){
  uint8_t network_status = fona.getNetworkStatus();
  if(network_status == 0){
    Serial.println(F("Not registered"));
    return EXIT_FAILURE;
  }else if(network_status == 1){
    Serial.println(F("Registered (home)"));
    return EXIT_SUCCESS;
  }else if(network_status == 2){
    Serial.println(F("Not registered (searching)"));
    return EXIT_FAILURE;
  }else if(network_status == 3){
    Serial.println(F("Denied"));
    return EXIT_FAILURE;
  }else if(network_status == 4){
    Serial.println(F("Unknown"));
    return EXIT_FAILURE;
  }else if(network_status == 5){
    Serial.println(F("Registered roaming"));
    return EXIT_FAILURE;
  }//end if
}//end estado_de_red

void conexion_fona_kolbi(){
  //Inicar la conección a la red celular GPRS
  fona.setGPRSNetworkSettings(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD));
  for(uint8_t i = 0; i < NETOWRK_CONCT_TRY; i++){
    delay(5000);
    if(estado_de_red() == EXIT_SUCCESS){
      Serial.print(F("Conectado. Fortaleza de señal: "));
      Serial.println(get_network_asu());
      return;
    }//end if
  }//end for
  Serial.println(F("No fue posible establecer la conexión"));
  reboot(F("ERROR grave, reiniciando..."));
}//end conexion_fona_kolbi()

void iniciar_FONA(){
  Serial.println(F("Inicializando FONA....(Puede tomar algunos segundos)"));
  
  fonaSS.begin(4800); // if you're using software serial
  
  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1) 
    reboot(F("La FONA presentó problemas de comunicación. Reiniciando"));
  }
  
  fonaSS.println("AT+CMEE=2");
  Serial.println(F("FONA OK"));

  Serial.println(F("Connectando a la red celular..."));
  while (estado_de_red() == EXIT_FAILURE){
   delay(500);//Reintentar en 500ms
  }//end while

  delay(5000);  //Esperar 5s para estabilizar FONA
  
  conexion_fona_kolbi();//Conectarse a red GPRS

  Serial.println(F("Desabilitando GPRS"));
  fona.enableGPRS(false);
  
  delay(5000);  //Esperar 5s para estabilizar FONA

  Serial.println(F("Habilitando GPRS"));
  if (!fona.enableGPRS(true)){
    reboot(F("No es posible continuar, error al habilitar GPRS, reiniciando..."));
  }
  Serial.println(F("Conectado a Red Celular!"));
  
  MQTT_connect();//Conectarse a MQTT
  if(conteo_de_fallos >= MAXFALLOS){
    //Se llegó a la máxima cantidad de fallos por lo que se bloqueará el programa
    //de esta forma el Watchdog entrará en acción y reiniciará nuestro programa
    //para solucionar cualquier problema de conexión    
    reboot(F("Máxima cantidad de fallos al conectar a MQTT"));
  }//end if  
  
  
}


void reboot(const __FlashStringHelper *error){
  Serial.println(error);
  delay(1000);
  Watchdog.enable(100);
  Watchdog.reset();
  while (1);//Este loop infinito correrá durante 100ms y el programa se reiniciará
}//end

void little_blink(int16_t t_delay, uint8_t num_blink){
  //Este método nos permitirá conocer si el Feather está intentando conectarse a Adafruit
  //En caso de que el LED se encuentre fijo indicará una conexión permanente
  for(uint8_t i = 0; i < num_blink; i++){
    digitalWrite(led_conexion_mqtt, LOW);
    delay(t_delay);
    digitalWrite(led_conexion_mqtt, HIGH);
    delay(t_delay);    
  }///end for
  digitalWrite(led_conexion_mqtt, LOW);
}//end little_blink

void setLedIndex(uint8_t _ledIndex){
  if(_ledIndex == 0){
    for(int i= 0;i < NUMPIXELS;i++){
      // pixels.Color toma valores RGB en formato uint32_t
      //adafruit originalmente diseño un método para convertir de RGB a uint32_t
      //pero no es necesario utilizarlo y en este caso es mejor utilizar colores ya convertidos
      pixels.setPixelColor(i, ledIndex[0]);//Configurar el color a mostrar
      pixels.show();//Actualizar el estado de los LEDs
      delay(delayval);//Esperar algunos milisegundos para prender otro led
    }//end for    
  }else if(_ledIndex == 11){
    for(int i= 0;i < NUMPIXELS;i++){
      pixels.setPixelColor(i, ledIndex[11]);//Configurar el color a mostrar
      pixels.show();//Actualizar el estado de los LEDs
      delay(10);//Esperar algunos milisegundos para prender otro led
    }//end for
  }else if(_ledIndex <= 5){
    for(int i=1;i < _ledIndex+1;i++){
      pixels.setPixelColor(i-1, ledIndex[i]);//Configurar el color a mostrar
      pixels.show();//Actualizar el estado de los LEDs
      delay(delayval);//Esperar algunos milisegundos para prender otro led
    }//end for
  }else if(_ledIndex > 5){
    for(int i=1;i <= _ledIndex-5 ;i++){
      pixels.setPixelColor(i-1, ledIndex[5+i]);//Configurar el color a mostrar
      pixels.show();//Actualizar el estado de los LEDs
      delay(delayval);//Esperar algunos milisegundos para prender otro led
    }//end for
  }//end if
}//end setLedIndex

