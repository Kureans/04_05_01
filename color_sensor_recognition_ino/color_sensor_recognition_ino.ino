/* This code works with GY-31 TCS3200 TCS230 color sensor module
 * It select a photodiode set and read its value (Red Set/Blue set/Green set) and displays it on the Serial monitor
 * and identify if possible the color
 * Refer to www.surtrtech.com for more details
 */
 
#define s0 8        //Module pins wiring
#define s1 9
#define s2 12
#define s3 13
#define out 7

#define echoPin A0 // attach pin A0 Arduino to pin Echo of HC-SR04
#define trigPin A1 //attach pin A1 Arduino to pin Trig of HC-SR04


unsigned long Red=0, Blue=0, Green=0;  //RGB values 
// defines variables
double duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement


void setup() 
{
   pinMode(s0,OUTPUT);    //pin modes
   pinMode(s1,OUTPUT);
   pinMode(s2,OUTPUT);
   pinMode(s3,OUTPUT);
   pinMode(out,INPUT);
   pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
   pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

   Serial.begin(9600);   //intialize the serial monitor baud rate
   
   digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   digitalWrite(s1,HIGH); //LOW/LOW is off HIGH/LOW is 20% and LOW/HIGH is  2%
   
}

void loop(){

  GetDistance();
  delay(500);
  GetColors();                                     //Execute the GetColors function to get the value of each RGB color
  
                                                   //Depending of the RGB values given by the sensor we can define the color and displays it on the monitor

    if (Green == 0)
      Serial.println("Error");
    else if (Green >= 800)
      Serial.println("Red");
    else
      Serial.println("Green");                                               

  delay(3000);                                   //2s delay you can modify if you want
  
  
  
}


void GetDistance()
{
 // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH); 
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void noColour()
{
  digitalWrite(s2, HIGH);                                           //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green 
  digitalWrite(s3, LOW);     
}

void GetColors()  
{    
  digitalWrite(s2, LOW);                                           //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green 
  digitalWrite(s3, LOW);                                           
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);       //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again, if you have trouble with this expression check the bottom of the code
  Serial.print("Red value= "); 
  Serial.print(Red);          //it's a time duration measured, which is related to frequency as the sensor gives a frequency depending on the color
  Serial.print("\t");          //The higher the frequency the lower the duration
  delay(50);  
  
  digitalWrite(s2,LOW);
  digitalWrite(s3, HIGH);                                         //Here we select the other color (set of photodiodes) and measure the other colors value using the same techinque
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  Serial.print("B value= "); 
  Serial.print(Blue);          //it's a time duration measured, which is related to frequency as the sensor gives a frequency depending on the color
  Serial.print("\t");          //The higher the frequency the lower the duration
  delay(50);  
  
  
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);  
  Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  Serial.print("G value= "); 
  Serial.print(Green);          //it's a time duration measured, which is related to frequency as the sensor gives a frequency depending on the color
  Serial.print("\t");          //The higher the frequency the lower the duration
  delay(50);  
}

