
//Sensor UV utilizado http://www.crcibernetica.com/electronic-brick-uv-sensor-brick/

float lecturas_uv_sensor;

void setup() {
  Serial.begin(115200);
  Serial.println("Inicio de programa");
}//end setup

void loop() {

}//end loop

float lecturas_V_sensor_UV(){
  /*Este médoto convierte el muestreo realizado por el pin analógico del sensor
    a valores de tensión[V] para así luego poder determinar el indice UV
    Según las especificaciones del sensor la tensión de salida del sensor
    es lineal al indice UV
  */
}//end lecturas_mV_sensor_uv

unsigned int indice_UV(float lectura_V){
  /*Conversión aproximada para determinar el indice UV que incide sobre el sensor.
    El indice UV se puede aproximar mediante la expresión UV = (307*lectura_V)/200
  */
}//end indice
