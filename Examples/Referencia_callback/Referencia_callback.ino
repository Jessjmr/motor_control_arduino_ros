//Programa V2.0 Incluye Control PI de velocidad y programa Publisher and Subscriber ROS

//Librerias ROS
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
//Libreria Encoder
#include <Encoder.h>
//Libreria Motor
#include <Adafruit_MotorShield.h>

float t1=0,u,t=0,int_e=0,kp,ki,ref,int_bound,ref_ros;
float a,ref_f=0,Hz;
long g_ant,grados;
unsigned long timeold = 0;

//Nodo de ROS nh
ros::NodeHandle  nh;

//Publisher ROS
std_msgs::Int32 info_sub1;
ros::Publisher vref("vref", &info_sub1);

//Callback Subcriber and publisher
void valor_ref( const std_msgs::Int32& v_ref){
  ref=v_ref.data;
  info_sub1.data=ref;
  vref.publish(&info_sub1);
}

//Subscriber ROS
ros::Subscriber<std_msgs::Int32> sub1("referencia_ros", valor_ref);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

void setup() {
  //Iniciando serial
  Serial.begin(57600);
  //Set-up motor shield
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin(1600,&Wire1)) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found."); 
  
  // Set-up encoder
  Serial.println("Basic Encoder Test:");
  // Medición de tiempo
  t1=float(millis())/1000; 
  // Cambio de pasos de encoder a grados= 2220 pulsos por revolución entre 360 grados
  g_ant=0.162162162*myEnc.read();  

  //Set-up salida LED
  pinMode(13, OUTPUT);  

  //Set-up ROS and subscriber ROS
  nh.initNode();
  nh.advertise(vref);
  nh.subscribe(sub1);
  
  
  //Variables de control
  kp=2;
  ki=10; 
  
  // saturación de la integral 
  int_bound=1.8;

  //Referencia, configurable por serial
  ref=0;
  a = 600;

  //Hertz
  Hz=20;  
}

void loop() {

if (millis() - timeold >= (1000/Hz)) {

  float t2,d_finitas,rpm,dt,e;
  long grados_actuales;

  // Lectura actual de posición
  grados_actuales = 0.162162162*myEnc.read();
  // Lectura de tiempo
  t2=float(millis())/1000;
  dt=t2-t1;
  // Calculo de la velocidad usando derivada sucia (diferencias finitas)
  d_finitas=(grados_actuales-g_ant)/(dt);
  // Grados a RPM  
  rpm=d_finitas/6;
  timeold = millis();
  //Control P  
  //u=(255/120)*(k*ref+(1-k)*rpm);
  if (abs(ref-ref_f)<abs(a/Hz)){
    ref_f=ref;        
  }
  else if(ref_f>ref){
    if(abs(ref-ref_f)>90){
      ref_f = ref_f - 0.3*(a/Hz);      
    }
    else{         
      ref_f = ref_f - (a/Hz);
    }    
  } else if (ref_f<ref) {
    if(abs(ref-ref_f)>90){
      ref_f = ref_f + 0.3*(a/Hz);      
    }
    else{         
      ref_f = ref_f + (a/Hz);
    }
  }
  
  
  e=ref_f-rpm; //Error
  if (abs(e)<10){
    int_e=int_e+e*dt; // Integral del error
  }
  
  // Saturación del integrador
  if (int_e>int_bound){
    int_e=int_bound;
  }
  else if (int_e<-int_bound){
    int_e=-int_bound;
  }
  //Control PI
  u=(255/120)*(rpm+(kp*e)+(ki*int_e));
  //u=255;
  //Serial.print(u*(120.0/255.0));
  //Serial.print(",");
  //Serial.print(ref_f);
  //Serial.print(",");
  //Serial.println(rpm);
  //Saturación del control
  if (u>255){
      u=255;
  }
  else if (u<-255){
      u=-255;
  }
  if (u<0){
  myMotor->run(BACKWARD);}
  else  {
  myMotor->run(FORWARD);}
  
  myMotor->setSpeed(abs(u));

  //ROS Publisher
  nh.spinOnce();
  

  t1=t2;
  g_ant=grados_actuales; 
}  
}