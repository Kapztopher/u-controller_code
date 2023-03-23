#include <Arduino.h>
#include <Wire.h>

// Interupt pin to be used for the leg position sensors
#define leg_pos_intpin    2

// MCPU23008A pin to leg sensor as well as leg_state[8] array position for high low value
#define front_left_high   2
#define front_left_low    0  
#define front_right_high  3
#define front_right_low   1
#define rear_left_high    6
#define rear_left_low     7
#define rear_right_high   5
#define rear_right_low    4

// Leg motor control pin connections
#define front_left_motor  4
#define front_right_motor 5
#define rear_left_motor   6
#define rear_right_motor  7

void posi_sens_intp(void);
void read_pos(void);
void map_leg_states(void);
void legs_high(void);
void legs_low(void);
void run_left_leg_high(void);
void run_left_leg_low(void);
void run_rear_high(void);
void run_rear_low(void);
void run_front_high(void);
void run_front_low(void);
void run_right_leg_high(void);
void run_right_leg_low(void);
void run_fl_rr_high(void);
void run_fr_rl_high (void);
void run_fl_rr_low(void);
void run_fr_rl_low (void);
void walk_step(void);


volatile bool posi_sens = false;
int mcp_reading = 0;
bool leg_states[8] = {0};

// 0 = front left, 1 = front right, 2 = rear left, 3 = rear right
bool motor_states[4] = {0};

void setup()
{

  pinMode(leg_pos_intpin, INPUT);

  pinMode(front_left_motor, OUTPUT);
  pinMode(front_right_motor, OUTPUT);
  pinMode(rear_left_motor, OUTPUT);
  pinMode(rear_right_motor, OUTPUT);

  digitalWrite(front_left_motor, LOW);
  digitalWrite(front_right_motor, LOW);
  digitalWrite(rear_left_motor, LOW);
  digitalWrite(rear_right_motor, LOW);


  int error1 = 0;

  Wire.begin();
  Serial.begin(9600);

  // I2C Port Expander
  Wire.beginTransmission(0x27); // begins talking to the slave device
  error1 = Wire.endTransmission();
  if (error1 == 0)
  {
    Serial.println("I2C I/O expander found at port 0x27");
  }
  else
  {
    Serial.println("Unknown error at address 0x27 - I/O expander");
    delay(500);
  }

  // while (error != 0)  {
  Wire.beginTransmission(0x27); // begins talking to the slave device
  error1 = Wire.endTransmission();
  //}

  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x00);             // selects the IODIRA register
  Wire.write(0xFF);             // this sets all port A pins to inputs
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x01);             // selects the IPOL register
  Wire.write(0x00);             // this sets the reflected polarity of the pin (0) Same (1) Opposite
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x02);             // Interupt on change
  Wire.write(0xFF);             // (I) enables GPIO interupt (O) disables GPIO interupt
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x03);             // selects the DEFVAL register
  Wire.write(0x00);             // Sets the defualt value of input pins
  Wire.endTransmission();       // stops talking to device
  delay(25);



  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x04);             // selects the INTCON register
  Wire.write(0x00);         // (I) Sets how pin is compared using interupt on change (O) Compares to previous value
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x05);             // selects the IOCON register
  Wire.write(0x0C);             // this sets the I/O controls
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27); // begins talking to the slave device
  Wire.write(0x06);             // selects the Pull up resitors (0) none (1) 100k pull up
  Wire.write(0x00);             // Sets the defualt value of input pins
  Wire.endTransmission();       // stops talking to device
  delay(25);

  Wire.beginTransmission(0x27);
  Wire.write(0x09);
  Wire.requestFrom(0x27, 1, true);
  if (Wire.available())
    {mcp_reading = Wire.read();
    }
  Wire.endTransmission();

  attachInterrupt(digitalPinToInterrupt(leg_pos_intpin), posi_sens_intp, FALLING);
  
}

void loop()
{

  

  if (posi_sens == true)
    { posi_sens = false;
    read_pos();
    delay(500);
    }
legs_low();
for (byte i = 0; i < 8; i++)
{
walk_step();
}







}

void posi_sens_intp(void)
{
  posi_sens = true;
}

void read_pos(void)
{
  delay(50);
  Wire.beginTransmission(0x27);
  Wire.write(0x09);
  Wire.requestFrom(0x27, 1, true);
  if (Wire.available())
  {
    mcp_reading = Wire.read();
  }
  Wire.endTransmission();
  /*
  Serial.println();
    for (byte i = 0; i < 8; i++)
  {
    Serial.print("\tLeg ");
    Serial.print(i+1);
  }
    Serial.println();
   for (byte i = 0; i < 8; i++)
  {
    Serial.print("\t");
    Serial.print(leg_states[i]);
  }
   Serial.println();
   */
}

void map_leg_states (void){

for (byte i = 0; i < 8; i++){
if (mcp_reading % 2){
  leg_states[i] = true;
  }
else {
  leg_states[i] = false;
  }
  mcp_reading = mcp_reading / 2;
  }}

void legs_high (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_high]) {
    motor_states[0] = true;
  }
  if (!leg_states[front_right_high])  {
    motor_states[1] = true;
  }
  if (!leg_states[rear_left_high])  {
    motor_states[2] = true;
  }
  if (!leg_states[rear_right_high])  {
    motor_states[3] = true;
  }

  if (motor_states[0])
  {
    digitalWrite(front_left_motor, HIGH);
  }
  if (motor_states[1])
  {
    digitalWrite(front_right_motor, HIGH);
  }
  if (motor_states[2])
  {
    digitalWrite(rear_left_motor, HIGH);
  }
  if (motor_states[3])
  {
    digitalWrite(rear_right_motor, HIGH);
  }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    
    if (leg_states[front_left_high])
    {
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
    }
    if (leg_states[front_right_high])
    {
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
    }
    if (leg_states[rear_left_high])
    {
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
    }
    if (leg_states[rear_right_high])
    {
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
    }

    if (!motor_states[0] && !motor_states[1] && !motor_states[2] && !motor_states[3])
    {
      break;
    }}}


void legs_low (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_low]) {
    motor_states[0] = true;
  }
  if (!leg_states[front_right_low])  {
    motor_states[1] = true;
  }
  if (!leg_states[rear_left_low])  {
    motor_states[2] = true;
  }
  if (!leg_states[rear_right_low])  {
    motor_states[3] = true;
  }

  if (motor_states[0])
  {
    digitalWrite(front_left_motor, HIGH);
  }
  if (motor_states[1])
  {
    digitalWrite(front_right_motor, HIGH);
  }
  if (motor_states[2])
  {
    digitalWrite(rear_left_motor, HIGH);
  }if (motor_states[3])
  {
    digitalWrite(rear_right_motor, HIGH);
  }
  while (true)
  {
      read_pos();
      delay(25);
   
      map_leg_states();
      delay(25);
    
    if (leg_states[front_left_low])
    {
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
    }
    if (leg_states[front_right_low])
    {
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
    }
    if (leg_states[rear_left_low])
    {
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
    }
    if (leg_states[rear_right_low])
    {
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
    }

    if (!motor_states[0] && !motor_states[1] && !motor_states[2] && !motor_states[3])
    {
      break;
    }}}


void run_left_leg_high (void){
  read_pos();
  if (!leg_states[front_left_high])
  {
  digitalWrite(front_left_motor, HIGH);
   while(!leg_states[front_left_high]){
    read_pos();
    map_leg_states();
    delay(25);
    }
  digitalWrite(front_left_motor, LOW);
  }
  
}

void run_left_leg_low (void){
  read_pos();
  if (!leg_states[front_left_low])
  {
  digitalWrite(front_left_motor, HIGH);
   while(!leg_states[front_left_low]){
    read_pos();
    map_leg_states();
    delay(25);
    }
  digitalWrite(front_left_motor, LOW);
    }}
  
void run_right_leg_high (void){
  read_pos();
  if (!leg_states[front_right_high])
  {
  digitalWrite(front_right_motor, HIGH);
   while(!leg_states[front_right_high]){
    read_pos();
    map_leg_states();
    delay(25);
    }
  digitalWrite(front_right_motor, LOW);
  }}

void run_right_leg_low (void){
  read_pos();
  if (!leg_states[front_right_low])
  {
  digitalWrite(front_right_motor, HIGH);
    while(!leg_states[front_right_low]){
      read_pos();
      posi_sens = false;
      map_leg_states();
      }}
  digitalWrite(front_right_motor, LOW);
  }
  




void run_rear_high (void){
  read_pos();
  map_leg_states();

  if (!leg_states[rear_left_high])  {
    motor_states[2] = true;
    }
  if (!leg_states[rear_right_high])  {
    motor_states[3] = true;
    }

  if (motor_states[2]){
    digitalWrite(rear_left_motor, HIGH);
    }
  if (motor_states[3]){
    digitalWrite(rear_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[rear_left_high]){
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
      }
    if (leg_states[rear_right_high]){
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
      }
    if (!motor_states[2] && !motor_states[3]){
    break;
    }}}


void run_rear_low (void){
  read_pos();
  map_leg_states();

  if (!leg_states[rear_left_low])  {
    motor_states[2] = true;
    }
  if (!leg_states[rear_right_low])  {
    motor_states[3] = true;
    }

  if (motor_states[2]){
    digitalWrite(rear_left_motor, HIGH);
    }
  if (motor_states[3]){
    digitalWrite(rear_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
    if (leg_states[rear_left_low]){
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
      }
    if (leg_states[rear_right_low]){
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
      }
    if (!motor_states[2] && !motor_states[3]){
    break;
    }}}



void run_front_high (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_high])  {
    motor_states[0] = true;
    }
  if (!leg_states[front_right_high])  {
    motor_states[1] = true;
    }

  if (motor_states[0]){
    digitalWrite(front_left_motor, HIGH);
    }
  if (motor_states[1]){
    digitalWrite(front_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[front_left_high]){
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
      }
    if (leg_states[front_right_high]){
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
      }
    if (!motor_states[0] && !motor_states[1]){
    break;
    }}}


void run_front_low (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_low])  {
    motor_states[0] = true;
    }
  if (!leg_states[front_right_low])  {
    motor_states[1] = true;
    }

  if (motor_states[0]){
    digitalWrite(front_left_motor, HIGH);
    }
  if (motor_states[1]){
    digitalWrite(front_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
    if (leg_states[front_left_low]){
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
      }
    if (leg_states[front_right_low]){
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
      }
    if (!motor_states[0] && !motor_states[1]){
    break;
    }}}

void run_fl_rr_high (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_high])  {
    motor_states[0] = true;
    }
  if (!leg_states[rear_right_high])  {
    motor_states[3] = true;
    }

  if (motor_states[0]){
    digitalWrite(front_left_motor, HIGH);
    }
  if (motor_states[3]){
    digitalWrite(rear_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[front_left_high]){
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
      }
    if (leg_states[rear_right_high]){
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
      }
    if (!motor_states[0] && !motor_states[3]){
    break;
    }}}


void run_fl_rr_low (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_left_low])  {
    motor_states[0] = true;
    }
  if (!leg_states[rear_right_low])  {
    motor_states[3] = true;
    }

  if (motor_states[0]){
    digitalWrite(front_left_motor, HIGH);
    }
  if (motor_states[3]){
    digitalWrite(rear_right_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[front_left_low]){
      digitalWrite(front_left_motor, LOW);
      motor_states[0] = false;
      }
    if (leg_states[rear_right_low]){
      digitalWrite(rear_right_motor, LOW);
      motor_states[3] = false;
      }
    if (!motor_states[0] && !motor_states[3]){
    break;
    }}}


void run_fr_rl_high (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_right_high])  {
    motor_states[1] = true;
    }
  if (!leg_states[rear_left_high])  {
    motor_states[2] = true;
    }

  if (motor_states[1]){
    digitalWrite(front_right_motor, HIGH);
    }
  if (motor_states[2]){
    digitalWrite(rear_left_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[front_right_high]){
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
      }
    if (leg_states[rear_left_high]){
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
      }
    if (!motor_states[1] && !motor_states[2]){
    break;
    }}}


void run_fr_rl_low (void){
  read_pos();
  map_leg_states();

  if (!leg_states[front_right_low])  {
    motor_states[1] = true;
    }
  if (!leg_states[rear_left_low])  {
    motor_states[2] = true;
    }

  if (motor_states[1]){
    digitalWrite(front_right_motor, HIGH);
    }
  if (motor_states[2]){
    digitalWrite(rear_left_motor, HIGH);
    }
  while (true)
  {
      read_pos();
      map_leg_states();
      delay(25);
    if (leg_states[front_right_low]){
      digitalWrite(front_right_motor, LOW);
      motor_states[1] = false;
      }
    if (leg_states[rear_left_low]){
      digitalWrite(rear_left_motor, LOW);
      motor_states[2] = false;
      }
    if (!motor_states[1] && !motor_states[2]){
    break;
    }}}



  void walk_step(void){
    for (byte i = 0; i < 3; i++)
    {

    legs_high();
    
    run_fr_rl_low();
    run_fl_rr_low();
    legs_high();
    run_fl_rr_low();
    run_fr_rl_low();
    legs_high();
    
    }}