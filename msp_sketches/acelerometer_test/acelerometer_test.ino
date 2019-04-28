
const int ap1 = A5;
const int ap2 = A4;
const int ap3 = A3;

int svx = 0;
int svy = 0;
int svz = 0;

float xa = 0.0;
float ya = 0.0;
float za = 0.0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  analogReference(DEFAULT);
  svx = analogRead(ap1);
  svy = analogRead(ap2);
  svz = analogRead(ap3);

  xa = (((svx*3.3)/1024)-1.65)/0.330;
  ya = (((svy*3.3)/1024)-1.65)/0.330;
  za = (((svz*3.3)/1024)-1.65)/0.330;
  
  Serial.print("x = " );                       
  Serial.print(xa);
  Serial.print("\ty = " );                       
  Serial.print(ya); 
  Serial.print("\tz = " );                       
  Serial.println(za);    

  Serial.print("theta = " );                       
  Serial.print(atan(xa/(sqrt(pow(ya,2)+pow(za,2)))));
  Serial.print("\tpsi = " );                       
  Serial.print(atan(ya/(sqrt(pow(xa,2)+pow(za,2)))));
  Serial.print("\tphi = " );                       
  Serial.println(atan((sqrt(pow(ya,2)+pow(za,2)))/xa));

  Serial.print("roll = " );                       
  Serial.print(atan2(ya,za)*57.29577951+180);
  Serial.print("\tpitch = " );                       
  Serial.print(atan2(za,xa)*57.29577951+180);
  Serial.print("\tyaw = " );                       
  Serial.println(atan2(xa,ya)*57.29577951+180);
  Serial.println();
  delay(2000);
}
