// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

//Pin en el que se encuentran conectados los neopixeles
#define LED_PIN            12

//Cantidad de Neipixeles
#define NUMPIXELS      5

#define BUTTON_PIN 11     // the number of the pushbutton pin
#define SIGNAL_PIN  10

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

//Inicialización del objeto NeoPixel
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500;//Intervalo para prender cada led

//Array de colores en formato uint32_t que indican el indice de radiación
int32_t ledIndex[15] = {0, 3302400, 65280, 16777010, 13158400, 16776960, 6566400, 6566400, 16724530, 13107200, 16711680, 13107400};

void setup() {
  //Inicialización de pines
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(SIGNAL_PIN, HIGH);//Este pin es simplemente una señal en alto para el pin 11
  
  pixels.begin(); //Configuración inicial de los Neopixeles
  pixels.setBrightness(80);//Disminución del brillo
}

void loop() {
  // read the state of the pushbutton value:
    buttonState = digitalRead(BUTTON_PIN);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    setLedIndex(7);//Establecimiento del indice de Radiación(Prueba de diseño)
  } else {
    setLedIndex(0);//Apagar todos los neopixeles
  }//end if
    
}

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
