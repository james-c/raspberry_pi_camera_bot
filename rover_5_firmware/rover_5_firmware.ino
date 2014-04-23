/* Copyright (c) 2014, Dawn Robotics Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <stdint.h>
#include <Servo.h>
#include "ultrasonic_sensor.h"
#include "rover_motor.h"
#include "rover_ir_sensors.h"

//------------------------------------------------------------------------------
const uint8_t VERSION_MAJOR = 0;
const uint8_t VERSION_MINOR = 5;
const uint16_t FIRMWARE_ID = 0xACED;

const uint16_t MAX_MSG_SIZE = 16;
const uint16_t MSG_START_BYTES = 0xFFFF;
const uint16_t MSG_ID_POS = 2;
const uint16_t MSG_SIZE_POS = 3;
const uint16_t MSG_HEADER_SIZE = 2 + 1 + 1; // Start bytes + Id + size

const uint8_t COMMAND_ID_GET_FIRMWARE_INFO = 1;
const uint8_t COMMAND_ID_SET_OUTPUTS = 2;
const uint8_t COMMAND_ID_SET_PAN_SERVO_LIMITS = 3;
const uint8_t COMMAND_ID_SET_TILT_SERVO_LIMITS = 4;

const uint8_t RESPONSE_ID_FIRMWARE_INFO = 1;
const uint8_t RESPONSE_ID_INVALID_COMMAND = 2;
const uint8_t RESPONSE_ID_INVALID_CHECK_SUM = 3;

enum eRobotState
{
    eRS_Invalid = -1,
    eRS_DrivingForwardsLookingForWall,
    eRS_BackingUp,
    eRS_EdgingForward,
    eRS_TurningLeft,
    eRS_FollowingWallOnRight
};

const int NUM_IR_SENSORS = 4;
const int IR_LED_PINS[ NUM_IR_SENSORS ] = { A0, A0, A1, A1 };   
const int IR_SENSOR_PINS[ NUM_IR_SENSORS ] = { A3, A2, A4, A5 };

const int LEFT_DIR_PIN = 12;
const int LEFT_PWM_PIN = 11;
const int LEFT_ENCODER_FIRST_PIN = 3;
const int LEFT_ENCODER_SECOND_PIN = 5;
const int LEFT_CURRENT_PIN = A6;

const int RIGHT_DIR_PIN = 7;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_ENCODER_FIRST_PIN = 2;
const int RIGHT_ENCODER_SECOND_PIN = 4;
const int RIGHT_CURRENT_PIN = A7;

const int PAN_SERVO_PIN = 9;
const int TILT_SERVO_PIN = 10;
const int ULTRASONIC_SENSOR_PIN = 8;

const int ABSOLUTE_MIN_PWM = 400;
const int ABSOLUTE_MAX_PWM = 2600;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 2000;

const float CLOSE_ULTRASONIC_RANGE = 0.1;
const int CLOSE_RANGE_IR_VALUE = 120;

const float WALL_FAR_DISTANCE = 0.25f;
const float WALL_CLOSE_DISTANCE = 0.24f;

RoverMotor gLeftMotor( LEFT_DIR_PIN, LEFT_PWM_PIN,
    LEFT_ENCODER_FIRST_PIN, LEFT_ENCODER_SECOND_PIN, LEFT_CURRENT_PIN );
RoverMotor gRightMotor( RIGHT_DIR_PIN, RIGHT_PWM_PIN,
    RIGHT_ENCODER_FIRST_PIN, RIGHT_ENCODER_SECOND_PIN, RIGHT_CURRENT_PIN );

UltrasonicSensor gUltrasonicSensor( ULTRASONIC_SENSOR_PIN );
RoverIRSensors gRoverIRSensors( 
    IR_LED_PINS[ 0 ], IR_LED_PINS[ 1 ],
    IR_LED_PINS[ 2 ], IR_LED_PINS[ 3 ],
    IR_SENSOR_PINS[ 0 ], IR_SENSOR_PINS[ 1 ],
    IR_SENSOR_PINS[ 2 ], IR_SENSOR_PINS[ 3 ] );

//------------------------------------------------------------------------------
// This class is provided because the Arduino Servo library maps minimum and 
// maximum bounds to a single byte for some reason.
class ServoLimits
{
    // MIN_PULSE_WIDTH and MAX_PULSE_WIDTH come from Servo.h
    public: ServoLimits( int minPWM=MIN_PULSE_WIDTH, int maxPWM=MAX_PULSE_WIDTH )
    {
        setLimits( minPWM, maxPWM );
    }
    
    public: void setLimits( int minPWM, int maxPWM )
    {
        mMinPWM = constrain( minPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
        mMaxPWM = constrain( maxPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
    }
    
    public: int convertAngleToPWM( int angle )
    {
        angle = constrain( angle, 0, 180 );
        return map( angle, 0, 180, mMinPWM, mMaxPWM );
    }
    
    public: int getMinPWM() const { return mMinPWM; }
    public: int getMaxPWM() const { return mMaxPWM; }
    
    private: int mMinPWM;
    private: int mMaxPWM;
};

//------------------------------------------------------------------------------
enum eMotorDirection
{
    eMD_Forwards,
    eMD_Backwards
};

enum eMessageState
{
    eMS_WaitingForMessage,
    eMS_ReceivingMessage
};

eMessageState gMessageState = eMS_WaitingForMessage;
uint8_t gMsgBuffer[ MAX_MSG_SIZE ];
uint16_t gNumMsgBytesReceived = 0;

Servo gPanServo;
ServoLimits gPanServoLimits;
Servo gTiltServo;
ServoLimits gTiltServoLimits;

uint8_t gLeftMotorDutyCycle = 0;
uint8_t gRightMotorDutyCycle = 0;
eMotorDirection gLeftMotorDirection = eMD_Forwards;
eMotorDirection gRightMotorDirection = eMD_Forwards;
uint8_t gPanServoAngle = 90;
uint8_t gTiltServoAngle = 90;
unsigned long gLastCommandTime = 0;


//------------------------------------------------------------------------------
uint8_t getMessageId() { return gMsgBuffer[ MSG_ID_POS ]; }
uint8_t getMessageSize() { return gMsgBuffer[ MSG_SIZE_POS ]; }
void receiveMessages();
void processMessage();
void sendFirmwareInfoResponse();
void sendInvalidCommandResponse();
void sendInvalidChecksumResponse();
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize );

//------------------------------------------------------------------------------
void setup()
{
    gPanServo.attach( PAN_SERVO_PIN );
    gTiltServo.attach( TILT_SERVO_PIN );
    
    Serial.begin( 57600 );
}

//------------------------------------------------------------------------------
void loop()
{
    // Read any commands from the serial connection
    receiveMessages();
    
    // Turn off the motors if we haven't received a command for a while
    unsigned long curTime = millis();
    
    if ( curTime - gLastCommandTime > MOTOR_COMMAND_TIMEOUT_MS )
    {
        gLeftMotorDutyCycle = 0;
        gRightMotorDutyCycle = 0;
        gLeftMotorDirection = eMD_Forwards;
        gRightMotorDirection = eMD_Forwards;
    }
    
    // Read from the robot's sensors
    float ultrasonicRange = gUltrasonicSensor.measureRange();
    gRoverIRSensors.takeReadings();
    int frontLeftIR = gRoverIRSensors.lastFrontLeftReading();
    int frontRightIR = gRoverIRSensors.lastFrontRightReading();
    int rearLeftIR = gRoverIRSensors.lastRearLeftReading();
    int rearRightIR = gRoverIRSensors.lastRearRightReading();

    // Update motors and servos
    if ( gLeftMotor.isStalled() )
    {
        gLeftMotor.clearStall();
    }
    
    if ( gRightMotor.isStalled() )
    {
        gRightMotor.clearStall();
    }
    
    gLeftMotor.setTargetRPM( 
        (float)gLeftMotorDutyCycle * ( gLeftMotorDirection == eMD_Forwards ? 1.0 : -1.0 ) );
    gRightMotor.setTargetRPM( 
        (float)gRightMotorDutyCycle * ( gRightMotorDirection == eMD_Forwards ? 1.0 : -1.0 ) );
    
    gLeftMotor.update();
    gRightMotor.update();
    
    gPanServo.writeMicroseconds( gPanServoLimits.convertAngleToPWM( gPanServoAngle ) );
    gTiltServo.writeMicroseconds( gTiltServoLimits.convertAngleToPWM( gTiltServoAngle ) );
    
    // Output debug info here
//     Serial.print( gLeftMotor.getTargetRPM() );
//     Serial.print( " " );
//     Serial.print( gLeftMotor.getLastMeasuredRPM() );
//     Serial.print( " " );
//     Serial.print( gRightMotor.getTargetRPM() );
//     Serial.print( " " );
//     Serial.println( gRightMotor.getLastMeasuredRPM() );
//     Serial.print( gLeftMotor.getLastMeasuredRPM() );
//     Serial.print( " " );
//     Serial.print( gRightMotor.getLastMeasuredRPM() );
//     Serial.print( " " );
//     Serial.print( gLeftMotor.getCurDutyCycle() );
//     Serial.print( " " );
//     Serial.println( gRightMotor.getCurDutyCycle() );
}

//------------------------------------------------------------------------------
void receiveMessages()
{
    bool bMessageReceived = false;
    int numBytesAvailable = Serial.available();
    
    while ( numBytesAvailable > 0 && !bMessageReceived )
    {
        switch ( gMessageState )
        {
            case eMS_WaitingForMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, MSG_HEADER_SIZE - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                                
                if ( MSG_HEADER_SIZE == gNumMsgBytesReceived )
                {
                    if ( MSG_START_BYTES == *((uint16_t*)gMsgBuffer) )
                    {
                        // We have a message header
                        gMessageState = eMS_ReceivingMessage;
                    }
                    else
                    {
                        // Discard the first byte as it is not part of a message
                        gMsgBuffer[ 0 ] = gMsgBuffer[ 1 ];
                        gMsgBuffer[ 1 ] = gMsgBuffer[ 2 ];
                        gMsgBuffer[ 2 ] = gMsgBuffer[ 3 ];
                        gNumMsgBytesReceived = 3;
                    }
                }
                
                break;
            }
            case eMS_ReceivingMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, getMessageSize() - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                
                if ( getMessageSize() == gNumMsgBytesReceived )
                {
                    processMessage();
                    bMessageReceived = true;
                    
                    // Prepare for next message
                    gNumMsgBytesReceived = 0;
                    gMessageState = eMS_WaitingForMessage;
                }
                
                break;
            }
            default:
            {
                // We should never get here, but just in case, return to eMS_WaitingForMessage
                gNumMsgBytesReceived = 0;
                gMessageState = eMS_WaitingForMessage;
            }
        }
        
        numBytesAvailable = Serial.available();
    }
}

//------------------------------------------------------------------------------
void processMessage()
{
    // Check the checksum of the message
    uint8_t calculatedCheckSum = calculateCheckSum( gMsgBuffer, getMessageSize() );
    
    if ( calculatedCheckSum != gMsgBuffer[ getMessageSize() - 1 ] )
    {
        sendInvalidCheckSumResponse();
    }
    
    // Handle the command Id
    bool bCommandHandled = false;
    switch ( getMessageId() )
    {
        case COMMAND_ID_GET_FIRMWARE_INFO:
        {
            sendFirmwareInfoResponse();
            bCommandHandled = true;
            
            break;
        }
        case COMMAND_ID_SET_OUTPUTS:
        {
            if ( getMessageSize() == 9 )
            {
                int leftMotorSpeed = (int)gMsgBuffer[ 4 ] - 128;
                int rightMotorSpeed = (int)gMsgBuffer[ 5 ] - 128;
                gPanServoAngle = constrain( gMsgBuffer[ 6 ], 0, 180 );
                gTiltServoAngle = constrain( gMsgBuffer[ 7 ], 0, 180 );
            
                gLeftMotorDirection = ( leftMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gRightMotorDirection = ( rightMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gLeftMotorDutyCycle = constrain( abs( leftMotorSpeed ), 0, 100 );
                gRightMotorDutyCycle = constrain( abs( rightMotorSpeed ), 0, 100 );
                gLastCommandTime = millis();
            
                bCommandHandled = true;
            }
        
            break;
        }
        case COMMAND_ID_SET_PAN_SERVO_LIMITS:
        case COMMAND_ID_SET_TILT_SERVO_LIMITS:
        {
            if ( getMessageSize() == 9 )
            {
                uint16_t servoMin = gMsgBuffer[ 4 ] << 8 | gMsgBuffer[ 5 ];
                uint16_t servoMax = gMsgBuffer[ 6 ] << 8 | gMsgBuffer[ 7 ];
                
                if ( getMessageId() == COMMAND_ID_SET_PAN_SERVO_LIMITS )
                {                   
                    gPanServoLimits.setLimits( servoMin, servoMax );
                }
                else
                {
                    gTiltServoLimits.setLimits( servoMin, servoMax );
                }
                
                bCommandHandled = true;
            }
            break;
        }
    }
    
    if ( !bCommandHandled )
    {
        sendInvalidCommandResponse();
    }
}

//------------------------------------------------------------------------------
void sendFirmwareInfoResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_FIRMWARE_INFO, 0,   // Header
        FIRMWARE_ID >> 8,  FIRMWARE_ID & 0xFF, 
        VERSION_MAJOR, VERSION_MINOR, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCommandResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_COMMAND, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCheckSumResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_CHECK_SUM, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize )
{
    uint32_t sum = 0;
    
    // Use all of the data apart from the message start bytes and the byte
    // that will store the checksum
    for ( uint8_t i = 2; i < msgSize - 1; i++ )
    {
        sum += pData[ i ];
    }
    
    return (uint8_t)(~sum);
}
