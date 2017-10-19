
//Sensor UV utilizado http://www.crcibernetica.com/electronic-brick-uv-sensor-brick/

float lecturas_uv_sensor;

void setup() {
  Serial.begin(115200);
  Serial.println("Inicio de programa");
}//end setup

void loop() {
  Serial.println(lecturas_V_sensor_UV());
  delay(1000);//Esperar un segundo hasta la siguiente medición
}//end loop

float lecturas_V_sensor_UV(){//Las lecturas deben retornarse en Volts
  /*Este médoto convierte el muestreo realizado por el pin analógico del sensor
    a valores de tensión[V] para así luego poder determinar el indice UV
    Según las especificaciones del sensor la tensión de salida del sensor
    es lineal al indice UV
  */
  //Se realizará un muestreo de la señal entrante para tener una mejor aproximación
  long sensor_uv = 0;
  long  sum = 0;
  for(int i = 0; i < 1024; i++){  
    sensor_uv += analogRead(A0);//Tomar nueva medición del sensor y sumarla para crear un promedio
    delay(2);
  }//end for
  sensor_uv /= 1024;//Tomar las mediciones y dividirlas entre el total de mediciones
  float sensor_en_V = sensor_uv*(3/1024.0);//Voltage del sensor en V
  
  return sensor_en_V;//Retorna el valor en V
}//end lecturas_mV_sensor_uv

unsigned int indice_UV(float lectura_V){
  /*Conversión aproximada para determinar el indice UV que incide sobre el sensor.
    El indice UV se puede aproximar mediante la expresión UV = (307*lectura_V)/200
  */
}//end indice
