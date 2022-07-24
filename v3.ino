#include <PID_v1.h>

// IR Sensors
int sensor1 = 11;      // Left most sensor
int sensor2 = 10;
int sensor3 = 9;
int sensor4 = 8;      // Right most sensor
     
// Initial Values of Sensors
int sensor[4] = {0, 0, 0, 0};

// Motor Variables              
int motorInput1 = 5;      //Motor Phải
int motorInput2 = 4;
int motorInput3 = 6;      //Motor Trái
int motorInput4 = 7;                 

//Initial Speed of Motor
double initial_motor_speed = 205; // 195-200-205-210-212.5-200-205.0000-200
double motor_speed = initial_motor_speed ;

//Tốc độ rẽ
int banh_chinh = 110;//-135-100-110.0000-90-100
int banh_phu = 255-100; //Đảo ngược-75-95-100.0000-75-80-100


// PID Constants
double Kp = 18;//4-5-6-7.5-8.5-13-14-15-16-16.5-17-17.5-18.0000-15-16
double Ki = 0.04;//2-0.25-0.15-0.02-0.05-0.25-0.1-0.08-0.06-0.040000-0.02
double Kd = 8;//-1.7-1.5-1.7-1.6-1.4-0.2-1.9-10-8.5-8-8-7.5-6.500-8-7-8.00000-1.5-2.5-3-1.9-7.5


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


  
  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(500);
  Serial.println("Started !!");
  delay(1000); 
  Serial.println(max_PID_value);
  delay(1000);
  
  myPID.SetOutputLimits(-max_PID_value, max_PID_value);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(13);//40-30-20-10-12-12.5-13.50000-12.5-14-13-12.5-14-16-13.5-12-13
}

void loop()
{
  read_sensor_values();
  //Serial.print(error);

  if ((error >= -1)&&(error <= 1)) memory = error; //tạo memory
  
  if (error == 100) {               // Rẽ Trái 90*    Make left turn untill it detects straight path
    do {                            // Quay sang trái cho tới khi phát hiện ngay giữa line
      sharpLeftTurn();
      read_sensor_values();
    } while (error != 0);
    //motor_speed = initial_motor_speed;  //phục hồi tốc độ
  
  } else if (error == 101){           // Rẽ Phải 90* Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
      do {                          // Quay sang trái cho tới khi phát hiện ngay giữa line
        sharpRightTurn();
        read_sensor_values();
      } while (error != 0);
      //motor_speed = initial_motor_speed;  //phục hồi tốc độ
      
  } else if (error == 102)         // Mất tín hiệu thì chạy theo memory
       error = memory ;
      
  else {
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
 
if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0))
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
    error = 102;    //Memory
}
/*if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1))
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
}*/


void motor_control()
{
  // Calculating the effective motor speed:
  
  int left_motor_speed = motor_speed  + PID_value;   //
  int right_motor_speed = motor_speed - PID_value;  //

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);   //
  right_motor_speed = constrain(right_motor_speed, 0, 255); //


  analogWrite(motorInput1, left_motor_speed); //Left Motor Speed    //  
  analogWrite(motorInput3, right_motor_speed); //Right Motor Speed  //

  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput2, LOW); //LOW = TIẾN
  digitalWrite(motorInput4, LOW);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(motorInput2, HIGH); //HIGH = LÙI
  digitalWrite(motorInput4, HIGH);
}

void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(motorInput1, banh_chinh);        //R Motor Speed
  digitalWrite(motorInput2, LOW);
  analogWrite(motorInput3, banh_phu);         //L Motor Speed
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(motorInput1, banh_phu);        //Left Motor Speed
  digitalWrite(motorInput2, HIGH);
  analogWrite(motorInput3, banh_chinh);         //Right Motor Speed
  digitalWrite(motorInput4, LOW);
}
