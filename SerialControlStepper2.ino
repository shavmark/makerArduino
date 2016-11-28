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
#define MyIDis 1

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

  enum Command {NoCommand,  Crash, Echo, SetPin=1, MoveTo, Move, Run, RunSpeed, SetMaxSpeed, SetAcceleration, SetSpeed, SetCurrentPosition, RunToPosition, RunSpeedToPosition, DisableOutputs, EnableOutputs, GetDistanceToGo, GetTargetPositon, GetCurrentPosition, } ;
  enum DataType {IKM_MAKERBOTXY=5, MAKERBOT_ID=199};

class xyRobot
{    
  public:
    ~xyRobot();
    
    void begin(int baud);
    
    int read();         // must be called regularly to clean out Serial buffer
    void run();      
    void IDPacket(Command, uint8_t data1, uint8_t data2);
    bool connection;
    
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
    int index=-1;              // -1 = waiting for new packet
    int readpacket();
    MeStepper* steppers[2]; // enable reference by id
    void allocRobots();
};

xyRobot robot;

void echo(uint8_t val){
  Serial.write(val);
}

void setup(){   
  
  robot.begin(19200);
  robot.connection = false;
  
}

void loop(){

  // sign on does not work in setup for some reason, spent 2 days on it.  time to move on
   if (!robot.connection){
    delay(50);
    robot.IDPacket(NoCommand, MAKERBOT_ID, IKM_MAKERBOTXY);
    Serial.println("Makeblock flatbed");
    Serial.println("bob");
   }

  robot.read();

  //robot.run();
}

// same data type as trossen for consistancy
// key data is there are 5 bytes, the ARM ID is known, byte 4 is 0, chcksum is known
void xyRobot::IDPacket(Command cmd, uint8_t data1, uint8_t data2)  {
  echo(0xee);
  echo(data1);// ARM ID for example
  echo(data2); 
  echo(cmd); // cmd?
  echo((255 - (data1+data2+cmd)%256));  
}

MeStepper*  xyRobot::getStepper(StepperID id) {
  if (id == IDstepper2){
    return steppers[1];
  }
  return steppers[0]; // always return something
}
xyRobot::~xyRobot(){
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
    IDPacket(Crash, 1, 0);
  }
}

void xyRobot::run(){

  if (getStepper(IDstepper1) == nullptr || getStepper(IDstepper2) == nullptr){
   IDPacket(Crash, 2, 0);
   return;
  }
  
  if (!getStepper(IDstepper1)->run()){
   IDPacket(Crash, 3, 0);
  }
  
  if (!getStepper(IDstepper2)->run()){
   IDPacket(Crash, 4, 0);
  }
}
 void xyRobot::begin(int baud){
  
  Serial.begin(baud);

  allocRobots();

  if (getStepper(IDstepper1) == nullptr && getStepper(IDstepper2) == nullptr){
   IDPacket(Crash, 5, 0);
   return;
  }

  // Change these to suit your stepper if you want, but set some reasonable defaults now
  if (getStepper(IDstepper1)){
    getStepper(IDstepper1)->setMaxSpeed(1000);
    getStepper(IDstepper1)->setAcceleration(20000);
  }
  if (getStepper(IDstepper2)){
    getStepper(IDstepper2)->setMaxSpeed(1000);
    getStepper(IDstepper2)->setAcceleration(20000);
  }

  index = -1;
}

void xyRobot::exec(StepperID id){
   
   IDPacket(Echo, id, getCommand(IDstepper1));
   
  switch(getCommand(id)){
   case MoveTo:
   //getStepper(id)->moveTo(getLongData(id));
   break;
  }
 
  //bugbug only send when data is validated if (!getStepper(id)->run()){
    //IDPacket(Crash, 99, 0);
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
      connection = true;
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

// does not send serial data
int xyRobot::readpacket(){

   while(Serial.available() > 0){

        if(index == -1){         // looking for new packet
            if(Serial.read() == 0xee){
              // new packet found
              index = 0;
              packet[index] = 0xee;
              index++;
            }
        }
        else {
            if(index == packetsize){ // packet complete
                index = -1;
                //flush serial
                while(Serial.available()) {
                  Serial.read();
                }
                return 1;
            }
            else if (index < packetsize){
              packet[index] = (uint8_t)Serial.read();
              index++;
            }
        }
    }
    return 0;
}
