String data;
int l_rpm, r_rpm;
const int left_dir = 2;
const int right_dir = 4;
const int left_pwm_pin = 3;
const int right_pwm_pin = 5;




void setup() {
  Serial.begin(9600);
  pinMode(left_dir, OUTPUT);
  pinMode(right_dir, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    String part01 = getValue(data,',',0);
    String part02 = getValue(data,',',1);
    l_rpm = part01.toInt();
    r_rpm = part02.toInt();
    motor_control(l_rpm, r_rpm);
    
 }
 Serial.print(l_rpm);
 Serial.print(" ");
 Serial.print(r_rpm);
 Serial.println(" ");
 
}


void motor_control(int l_rpm, int r_rpm){
  digitalWrite(left_dir,1);
  analogWrite(left_pwm_pin, l_rpm);
  digitalWrite(right_dir,1);
  analogWrite(right_pwm_pin, r_rpm);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
