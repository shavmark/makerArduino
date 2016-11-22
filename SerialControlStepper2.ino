/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    SerialControlStepper.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/11/19
 * @brief   Description: this file is sample code for Stepper Driver device.
 *
 * Function List:
 *
 *    1. void MeStepper::moveTo(long absolute); 
 *    2. void MeStepper::move(long relative);
 *    3. boolean MeStepper::run();
 *    4. void MeStepper::setMaxSpeed(float speed);
 *    5. void MeStepper::setAcceleration(float acceleration);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2015/09/07    1.0.0          rebuild the old lib.
 * forfish      2015/11/19    1.0.0          add some descriptions.
 * </pre>
 */

#include "MeOrion.h"
#include <SoftwareSerial.h>

/* commands
      1. void MeStepper::setpin(uint8_t dir_data, uint8_t step_data);
 *    2. void MeStepper::moveTo(long absolute); 
 *    3. void MeStepper::move(long relative);
 *    4. boolean MeStepper::run();
 *    5. boolean MeStepper::runSpeed();
 *    6. void MeStepper::setMaxSpeed(float speed);
 *    7. void MeStepper::setAcceleration(float acceleration);
 *    8. void MeStepper::setSpeed(float speed);
 *    9. float MeStepper::speed();
 *    10. long MeStepper::distanceToGo();
 *    11. long MeStepper::targetPosition();
 *    12. long MeStepper::currentPosition();  
 *    13. void MeStepper::setCurrentPosition(long position);  
 *    14. void MeStepper::runToPosition();
 *    15. boolean MeStepper::runSpeedToPosition();
 *    16. void MeStepper::runToNewPosition(long position);
 *    17. void MeStepper::disableOutputs();
 *    18. void MeStepper::enableOutputs();
 */


class xyRobot
{    
  public:
    enum Command {NoCommand, SetPin, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetDistanceToGo, GetTargetPositon, GetCurrentPosition, } ;
    
    xyRobot() {allocRobots();}

    ~xyRobot() {end();}
    
    void begin(int baud);
    void end();
    
    int read();         // must be called regularly to clean out Serial buffer
    void run();      
    void IDPacket();
  private:

  /* data - 1 or 2 steppers defined in the data
 *  byte 0 : 0xee - start of data packet
 *  byte 1 : cmd for stepper1 (see enum Command)
 *  byte 2 : data for stepper1 (high byte) 
 *  byte 3 : data for stepper1 (low byte)
 *  byte 4 : cmd  for stepper2, NoCommand for none
 *  byte 5 : data for stepper2 (high byte)
 *  byte 6 : data for stepper2 (low byte)
 */

    enum StepperID {IDstepper1=1, IDstepper2=4};
    long getLongData(StepperID id){return (long)((packet[id+1]<<8) + packet[id+2]);}
    float getFloatData(StepperID id){return (float)((packet[id+1]<<8) + packet[id+2]);}
    Command getCommand(StepperID id) {return packet[id];}
    MeStepper*  getStepper(StepperID id);
   
    void exec(StepperID id);
    
    // internal variables used for reading messages
    const int packetsize = 7;
    uint8_t packet[7];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
    int readpacket();
    MeStepper* steppers[2]; // enable reference by id
    void allocRobots();
};

xyRobot robot;

void setup()
{  
  robot.begin(9600);
}

void loop()
{
  robot.read();
  robot.run();
}

// same data type as trossen for consistancy
// key data is there are 5 bytes, the ARM ID is known, byte 4 is 0, chcksum is known
void xyRobot::IDPacket()  {
  Serial.write(0xFF);
  Serial.write((uint8_t) 4);// ARM ID
  Serial.write((uint8_t) 6); // mode
  Serial.write((uint8_t) 0);
  Serial.write((uint8_t)(255);
  
}

MeStepper*  xyRobot::getStepper(StepperID id) {
  if (id == IDstepper1){
    return steppers[0];
  }
  if (id == IDstepper2){
    return steppers[1];
  }
}
void xyRobot::end(){
  if (steppers[0]){
    delete steppers[0];
    steppers[0] = nullptr;
  }
  if (steppers[1]){
    delete steppers[1];
    steppers[1] = nullptr;
  }
}
void xyRobot::allocRobots(){
  steppers[0] = new MeStepper(PORT_1);
  steppers[1] = new MeStepper(PORT_2);
  if (steppers[0] == nullptr || steppers[1] == nullptr){
   Serial.println("crash!");//bugbug go to json
  }
  memset(packet, 0, sizeof packet);
}

void xyRobot::run(){
   if (getStepper(IDstepper1) == nullptr || getStepper(IDstepper2) == nullptr){
   Serial.println("crash!");//bugbug go to json
   return;
  }
  if (!getStepper(IDstepper1)->run()){
    Serial.println("IDstepper1 bad!");
  }
  if (!getStepper(IDstepper2)->run()){
    Serial.println("IDstepper2 bad!");
  }
}
 void xyRobot::begin(int baud){
  Serial.begin(baud);

  if (getStepper(IDstepper1) == nullptr && getStepper(IDstepper2) == nullptr){
    Serial.println("not setup!"); //bugbug return json in all micro code
    return;
  }

  index = -1;
  // Change these to suit your stepper if you want, but set some reasonable defaults now
  if (getStepper(IDstepper1)){
    getStepper(IDstepper1)->setMaxSpeed(1000);
    getStepper(IDstepper1)->setAcceleration(20000);
  }
  if (getStepper(IDstepper2)){
    getStepper(IDstepper2)->setMaxSpeed(1000);
    getStepper(IDstepper2)->setAcceleration(20000);
  }
  Serial.println("xy robot signed on");
  Serial.println("my name is Lena");
}

void xyRobot::exec(StepperID id){
  switch(getCommand(id)){
   case MoveTo:
   Serial.print("move to ");
   Serial.println(getLongData(id));
   //getStepper(id)->moveTo(getLongData(id));
   break; 
  }
  IDPacket(); // always sent to be comptable with trossen
  //bugbug only send when data is validated if (!getStepper(id)->run()){
    //Serial.println("bad!");
  //}
}


/* data - 1 or 2 steppers defined in the data
 *  byte 0 : 0xee - start of data packet
 *  byte 1 : cmd for stepper1 (see enum Command)
 *  byte 2 : data for stepper1 (high byte) 
 *  byte 3 : data for stepper1 (low byte)
 *  byte 4 : cmd  for stepper2, NoCommand for none
 *  byte 5 : data for stepper2 (high byte)
 *  byte 6 : data for stepper2 (low byte)
 */
int xyRobot::read(){

    if (readpacket()){
        // packet complete, execute it
        if (getCommand(IDstepper1) != NoCommand){
          exec(IDstepper1);
        }
        if (getCommand(IDstepper2) != NoCommand){
          exec(IDstepper2);
        }
        // reset data
         memset(packet, 0, sizeof packet);
        return 1;
    }
    return 0;
}

int xyRobot::readpacket(){
  
   while(Serial.available() > 0){

        if(index == -1){         // looking for new packet
            if(Serial.read() == 0xee){
              // new packet found
              index = 0;
            }
            else {
              Serial.println("unknown");
            }
        }
        else {
            if(index == packetsize){ // packet complete
                index = -1;
                //flush serial
                while(Serial.available()) {
                  Serial.read();
                }
                 Serial.print("packet read:");
                 Serial.print(" cmd1:");
                 Serial.print(getCommand(IDstepper1));
                 Serial.print(" long data: ");
                 Serial.print(getLongData(IDstepper1));
                 Serial.print(" float data: ");
                 Serial.print(getFloatData(IDstepper1));
                 Serial.print(" cmd2:");
                 Serial.print(getCommand(IDstepper2));
                 Serial.print(" long data: ");
                 Serial.print(getLongData(IDstepper2));
                 Serial.print(" float data: ");
                 Serial.println(getFloatData(IDstepper2));
                return 1;
            }
            packet[index] = (uint8_t) Serial.read();
            index++;
        }
    }
    return 0;
}
