#include <PID_v1.h>

// IR Sensors
int sensor1 = 11;      // Left most sensor
int sensor2 = 10;
int sensor3 = 9;
int sensor4 = 8;      // Right most sensor
     
// Initial Values of Sensors
int sensor[4] = {0, 0, 0, 0};

// Motor Variables
int ENA = 6;              //Motor phải
int motorInput1 = 2;
int motorInput2 = 3;
int motorInput3 = 4;
int motorInput4 = 7;
int ENB = 5;             //Motor trái

//Initial Speed of Motor
double initial_motor_speed = 150 ;
double motor_speed = initial_motor_speed ;

//Tốc độ rẽ
double banh_chinh = 95;
double banh_phu = 95;


// PID Constants
double Kp = 15;//4-5-6-7.5-8.5
double Ki = 0.2;//2-0.25-0.15-0.02-0.05
double Kd = 10;//-1.7-1.5-1.7-1.6-1.4-0.2-1.9

double error = 0, PID_value = 0, Setpoint = 0;
double max_PID_value = 255 - motor_speed;


int flag = 0;
int memory = 0;

//Khai báo PID
  PID myPID(&error, &PID_value, &Setpoint, Kp, Ki, Kd, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                                //P_ON_E (Proportional on Error) is the default behavior
  
void setup()
{
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  
  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!");
  delay(1000);
  Serial.println(max_PID_value);
  delay(1000);
  //Serial.println(motor_speed);
  //delay(1000);
  myPID.SetOutputLimits(-max_PID_value, max_PID_value);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);//40-30-20
}

void loop()
{
  read_sensor_values();
  //Serial.print(error);

  if ((error >= -1)&&(error <= 1)) memory = error; //tạo memory
  
  if (error == 100) {               // Rẽ Trái 90*    Make left turn untill it detects straight path
    do {                            // Quay sang trái cho tới khi phát hiện ngay giữa line
      sharpLeftTurn();
      analogWrite(ENA, banh_chinh);        //Right Motor Speed
      analogWrite(ENB, banh_phu);         //Left Motor Speed
      read_sensor_values();
    } while (error != 0);
    //motor_speed = initial_motor_speed;  //phục hồi tốc độ
  
  } else if (error == 101){           // Rẽ Phải 90* Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
      do {                          // Quay sang trái cho tới khi phát hiện ngay giữa line
        sharpRightTurn();
        analogWrite(ENA, banh_phu);        //Right Motor Speed
        analogWrite(ENB, banh_chinh);        //Left Motor Speed
        read_sensor_values();
      } while (error != 0);
      //motor_speed = initial_motor_speed;  //phục hồi tốc độ
      
  } else if (error == 102)         // Mất tín hiệu thì chạy theo memory
       error = memory ;
      
  else {
    //calculate_pid();                  // Tính giá trị PID
    myPID.Compute();
    motor_control();                  // Điều chỉnh motor theo giá trị PID mới tính, cho xe chạy thẳng
    Serial.print(PID_value);
      }
}




void read_sensor_values()
{
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  
/*if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0))
  error=3;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1))
  error=1;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1))
  error=0;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1))
  error=-1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1))
  error=-2;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1))
  error=-3;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Turn robot right side
    error = 101;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Memory
    error = 102;
  //else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)&&(sensor[4]==0)) // Turn left side or stop
   // error = 103;
  else 
    error = 102; */   //Memory

if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1))
  error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1))
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0))
  error=1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-1;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0))
  error=-3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1)) // Turn robot right side
    error = 101;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)) // Memory
    error = 102;
  //else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)&&(sensor[4]==0)) // Turn left side or stop
   // error = 103;
  else 
    error = 102;    //Memory
}


void motor_control()
{
  // Calculating the effective motor speed:
  
  int left_motor_speed = motor_speed  - PID_value;   //
  int right_motor_speed = motor_speed + PID_value;  //

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);   //
  right_motor_speed = constrain(right_motor_speed, 0, 255); //


  analogWrite(ENA, left_motor_speed); //Left Motor Speed    //  
  analogWrite(ENB, right_motor_speed); //Right Motor Speed  //

  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
