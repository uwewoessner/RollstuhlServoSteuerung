#include "FilterDerivative.h"
#include <stdio.h>

float FilterDerivative::input( float inVal ) {
  long thisUS = micros();
  float dt = 1e-6*float(thisUS - LastUS);   // cast to float here, for math
  LastUS = thisUS;                          // update this now
  
  Derivative = (inVal-LastInput) / dt;
    
  LastInput = inVal;
  return output();
}
  
float FilterDerivative::output() { return Derivative; }
  
void testFilterDerivative() {
  FilterDerivative der;
  
  while(true) {
    float t = 1e-6 * float( micros() );
    float value = 100*sin(TWO_PI*t);
    
    der.input(value);
    
    printf( "\n %f \t",der.output() );
    
    //delay(10);
  }
}
