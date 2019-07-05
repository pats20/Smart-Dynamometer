double Vs = 9.0;                    // applied voltage
double Vf_p = 0;                    //
double Vf_n = 0;

int valPWM_EV;
int valPWM_dyno;
unsigned long t0 = 0;               //Initialize t0 as 0 Sec
unsigned long t1 = 0;               //Initialize t1 as 0 Sec
unsigned long t2 = 0;               //Initialize t2 as 0 Sec
unsigned long T = 500;              // Sampling time,ms
int sig = 0;                        //Initialize current signal as low
int sig_prev = 0;                   //Initialize previous signal as low
int count = 0;
double RPM = 0;                     //Initialize RPM as 0 RPM
double ref_RPM = 0;               // example value

unsigned long time_int = 0; // initial time during drive cycle
unsigned long time_array[9] = {0,10,20,30,40,50,60,70,80}; //time array 
//double ref_RPM_array[9] = {50,100,150,200,250,200,150,100,50}; //rpm array
double ref_RPM_array[9] = {150,150,150,150,150,150,150,150,150}; //rpm array
double theta[9] = {0,0,0,0,0,0,0,0,0}; //road grade array
//double theta[9] = {0,0,-5,-5,0,0,5,5,0}; //road grade array

double theta_road;
int i = 0;

//double alpha = 0.000172/5;
double alpha = 0;
//double beta = 0.00785;
double beta = 0;
double gamma = 25.807;  

double inte = 0;                    //initial error
double err = 0;                     // error calculated
double V_ev = 0;                    // voltage of electric vehicle
double V_ev_max = 4.5;                // maximum voltage of EV
double V_ev_min = -4.5;               //minimum voltage of ev
double V_dyno = 0;                  // voltage of electric vehicle
double V_dyno_max = 9;              // maximum voltage of EV
double V_dyno_min = -9;             //minimum voltage of ev
double Kp = 0.005;                   // Proportional Gain
double Ki = 0.01;                   // Integral Gain


void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);               // voltage is set at corresponding value
  pinMode(8, OUTPUT);               // voltage is set at corresponding value
  Serial.begin(9600);               // Set the Serial Speed to 9600 Mb/s
  delay(1000);                      // Delay 1 sec   
  t0 = millis();                    //Assigning current time to t0
  time_int = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((millis()-t0) > T){                                //
    t0 = millis();
    RPM = 0;
    if (t2 > t1 && count > 1){
      RPM = 1000.0*60.0/12.0*(count-1)/(t2-t1);
    }

    // conditions for drive cycle
    if ((millis()-time_int) > time_array[i]*1000 && i<sizeof(time_array)-1)
     { 
      ref_RPM = ref_RPM_array[i]; // to access the array
      theta_road = theta[i]*3.14/180;
      i = (i+1); // to loop the drive cycle
     }
    
     err = ref_RPM - RPM;
     inte = inte + err*T/1000.0;      // integration anti - windup
     V_ev = Kp*err + Ki*inte;
     
     if (V_ev > V_ev_max){
        V_ev = V_ev_max;
        inte = inte - err*T/1000.0;   // integration anti - windup
     }
     if (V_ev < V_ev_min){
        V_ev = V_ev_min;
        inte = inte - err*T/1000.0;  
     }

    //V_ev = 0;
    
    if (V_ev > 0)
       {valPWM_EV = 255*(V_ev+Vf_p)/Vs;
        digitalWrite(7,LOW);}
    else if (V_ev < 0)
       {valPWM_EV = 255*(V_ev+Vf_n)/Vs;
        digitalWrite(7,HIGH);}
    else
       {valPWM_EV = 0;}
    
    analogWrite(5,valPWM_EV);

   // Dyno
    V_dyno = alpha * RPM * RPM + beta * RPM + gamma * sin(theta_road); // Air drag equation

    // V_dyno = 9;
    
    if (V_dyno > 0)
       {valPWM_dyno = 255*(V_dyno)/Vs;
        digitalWrite(8,HIGH);}
    else if (V_dyno < 0)
       {valPWM_dyno = 255*(V_dyno)/Vs;
        digitalWrite(8,LOW);}
    else
       {valPWM_dyno = 0;}

    analogWrite(6,valPWM_dyno);

    Serial.print(t0/1000.0);
    Serial.print("\t");
    //Serial.print(valPWM);
    Serial.print(V_ev);
    Serial.print("\t");
    Serial.print(V_dyno);
    Serial.print("\t");
    Serial.print(ref_RPM);
    Serial.print("\t");
    // Serial.print(ref_RPM_array[i]);
    //Serial.print("\t");
    Serial.println(RPM);  
        
    count = 0;
  }

  
  else{
    sig = digitalRead(4);
    if ((sig > sig_prev) && count == 0){
      t1 = millis();
      t2 = millis();
      count++;
    }
    if ((sig > sig_prev) && count > 0){
      t2 = millis();
      count++;
    }
  }
       
  sig_prev = sig;
  }
