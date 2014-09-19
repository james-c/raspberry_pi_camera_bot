#! /usr/bin/env python

# Copyright (c) 2014, Dawn Robotics Ltd
# All rights reserved.

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.

# 3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import logging
import math
import time
import Queue
import mini_driver
import threading
import differential_drive_controller
import csv

#--------------------------------------------------------------------------------------------------- 
class RobotController:
    
    MIN_ANGLE = 0.0
    MAX_ANGLE = 180.0
    CENTRE_ANGLE = (MIN_ANGLE + MAX_ANGLE)/2.0
    
    MAX_UPDATE_TIME_DIFF = 0.25
    TIME_BETWEEN_SERVO_SETTING_UPDATES = 1.0
    TIME_BETWEEN_SENSOR_CONFIGURATION_UPDATES = 0.5
    
    JOYSTICK_DEAD_ZONE = 0.1
    MAX_ABS_NECK_SPEED = 30.0   # Degrees per second
    
    MOTION_COMMAND_TIMEOUT = 2.0 # If no commands for the motors are recieved in this time then
                                 # the motors (drive and servo) are set to zero speed
    
    MOVEMENT_STATE_NORMAL = "normal"
    MOVEMENT_STATE_TURNING = "turning"
    MOVEMENT_STATE_TRAVELLING_AT_SET_SPEED = "travelling_at_set_speed"
    MOVEMENT_STATE_DRIVING_STRAIGHT = "driving_straight"
    
    TICKS_PER_REVOLUTION = 780.0
    WHEEL_DIAMETER = 64.0/1000.0        # Diameter in metres
    WHEEL_CIRCUMFERENCE = math.pi*WHEEL_DIAMETER
    ENCODER_TICKS_PER_METRE = TICKS_PER_REVOLUTION/WHEEL_CIRCUMFERENCE
    
    DEFAULT_SPEED = 0.2     # Metres per second
    DEFAULT_SPEED_TICKS_PER_SECOND = DEFAULT_SPEED*ENCODER_TICKS_PER_METRE
    
    WHEEL_CENTRE_TO_WHEEL_CENTRE_DISTANCE = 0.1     # In metres
    TURN_CIRCUMFERENCE = math.pi*WHEEL_CENTRE_TO_WHEEL_CENTRE_DISTANCE
    TURN_METRES_PER_DEGREE = TURN_CIRCUMFERENCE/360.0
    TURN_ENCODER_TICKS_PER_DEGREE = TURN_METRES_PER_DEGREE*ENCODER_TICKS_PER_METRE
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, robotConfig ):
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.differentialDriveController = differential_drive_controller.DifferentialDriveController()
        
        self.robotMovementState = self.MOVEMENT_STATE_NORMAL
        self.robotConfig = robotConfig
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.panAngle = self.CENTRE_ANGLE
        self.tiltAngle = self.CENTRE_ANGLE
        
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
        
        self.lastServoSettingsSendTime = 0.0
        self.lastSensorConfigurationSendTime = 0.0
        self.lastUpdateTime = 0.0
        self.lastMotionCommandTime = time.time()
        
        self.piSensorModuleName = ""
        self.piSensorModule = None
        self.piSensorReader = None
        self.piSensorDict = {}
        
        self.errorValuesList = []
        self.encoderMoveStartTime = 0.0
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        self.miniDriver.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def getStatusDict( self ):
        
        presetMaxAbsMotorSpeed, presetMaxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
        
        statusDict = {
            "robotMovementState" : self.robotMovementState,
            "batteryVoltage" : self.miniDriver.getBatteryVoltageReading().data,
            "presetMaxAbsMotorSpeed" : presetMaxAbsMotorSpeed,
            "presetMaxAbsTurnSpeed" : presetMaxAbsTurnSpeed,
            "sensors" : self.getSensorDict()
        }
        
        return statusDict
        
    #-----------------------------------------------------------------------------------------------
    def getSensorDict( self ):
                
        sensorDict = {
            "batteryVoltage" : self.miniDriver.getBatteryVoltageReading(),
            "digital" : self.miniDriver.getDigitalReadings(),
            "analog" : self.miniDriver.getAnalogReadings(),
            "ultrasonic" : self.miniDriver.getUltrasonicReading(),
            "encoders" : self.miniDriver.getEncodersReading(),
        }
        
        sensorDict.update( self.piSensorDict )
        
        return sensorDict
    
    #-----------------------------------------------------------------------------------------------
    def normaliseJoystickData( self, joystickX, joystickY ):
        
        stickVectorLength = math.sqrt( joystickX**2 + joystickY**2 )
        if stickVectorLength > 1.0:
            joystickX /= stickVectorLength
            joystickY /= stickVectorLength
        
        if stickVectorLength < self.JOYSTICK_DEAD_ZONE:
            joystickX = 0.0
            joystickY = 0.0
            
        return ( joystickX, joystickY )
    
    #-----------------------------------------------------------------------------------------------
    def centreNeck( self ):
        
        self.panAngle = self.CENTRE_ANGLE
        self.tiltAngle = self.CENTRE_ANGLE
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
    
    #-----------------------------------------------------------------------------------------------
    def setMotorJoystickPos( self, joystickX, joystickY ):
        
        # Switch back to 'normal' movement
        self._setRobotMovementState( self.MOVEMENT_STATE_NORMAL )
        
        # Convert the joystick inputs into motor speeds
        joystickX, joystickY = self.normaliseJoystickData( joystickX, joystickY )
        
        if self.robotConfig.usePresetMotorSpeeds:
            
            maxAbsMotorSpeed, maxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
            
        else:
            
            maxAbsMotorSpeed = self.robotConfig.customMaxAbsMotorSpeed
            maxAbsTurnSpeed = self.robotConfig.customMaxAbsTurnSpeed
        
        # Set forward speed from joystickY
        leftMotorSpeed = maxAbsMotorSpeed*joystickY
        rightMotorSpeed = maxAbsMotorSpeed*joystickY
        
        # Set turn speed from joystickX
        leftMotorSpeed += maxAbsTurnSpeed*joystickX
        rightMotorSpeed -= maxAbsTurnSpeed*joystickX
        
        leftMotorSpeed = max( -maxAbsMotorSpeed, min( leftMotorSpeed, maxAbsMotorSpeed ) )
        rightMotorSpeed = max( -maxAbsMotorSpeed, min( rightMotorSpeed, maxAbsMotorSpeed ) )
        
        self.leftMotorSpeed = leftMotorSpeed*self.robotConfig.leftMotorScale
        self.rightMotorSpeed = rightMotorSpeed
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setMotorSpeeds( self, leftMotorSpeed, rightMotorSpeed ):
        
        """Set motor speeds from -100% to 100%"""
        
        # Switch back to 'normal' movement
        self._setRobotMovementState( self.MOVEMENT_STATE_NORMAL )
        
        if self.robotConfig.usePresetMotorSpeeds:
            
            maxAbsMotorSpeed, maxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
            
        else:
            
            maxAbsMotorSpeed = self.robotConfig.customMaxAbsMotorSpeed
            maxAbsTurnSpeed = self.robotConfig.customMaxAbsTurnSpeed
        
        self.leftMotorSpeed = max( -maxAbsMotorSpeed, min( leftMotorSpeed, maxAbsMotorSpeed ) )
        self.rightMotorSpeed = max( -maxAbsMotorSpeed, min( rightMotorSpeed, maxAbsMotorSpeed ) )
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setNeckJoystickPos( self, joystickX, joystickY ):
        
        joystickX, joystickY = self.normaliseJoystickData( joystickX, joystickY )
        
        # Set pan and tilt angle speeds
        self.panSpeed = -self.MAX_ABS_NECK_SPEED*joystickX
        self.tiltSpeed = -self.MAX_ABS_NECK_SPEED*joystickY
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setNeckAngles( self, panAngle, tiltAngle ):
        
        self.panAngle = max( self.MIN_ANGLE, min( panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( tiltAngle, self.MAX_ANGLE ) )
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def startTurn( self, turnAngle ):
        
        """Start the robot turning a given number of degrees. Positive angles turn to the left (anti-clockwise)"""
        
        if self.robotMovementState == self.MOVEMENT_STATE_NORMAL:
            
            # Get the current encoder readings
            encodersReading = self.miniDriver.getEncodersReading()
            leftEncoderReading, rightEncoderReading = encodersReading.data
            
            # Calculate distance to move wheels
            targetLeftEncoderReading = leftEncoderReading - turnAngle*self.TURN_ENCODER_TICKS_PER_DEGREE
            targetRightEncoderReading = rightEncoderReading + turnAngle*self.TURN_ENCODER_TICKS_PER_DEGREE
            
            # Setup the differential drive controller
            self.differentialDriveController.reset()
            self.differentialDriveController.setTargets( 
                targetLeftEncoderReading, targetRightEncoderReading,
                self.DEFAULT_SPEED_TICKS_PER_SECOND, self.DEFAULT_SPEED_TICKS_PER_SECOND )
            
            self.errorValuesList = []
            self.encoderMoveStartTime = time.time()
            
            self.robotMovementState = self.MOVEMENT_STATE_TURNING
    
    #-----------------------------------------------------------------------------------------------
    def setMotorSpeedsInMetresPerSecond( self, leftMotorSpeed, rightMotorSpeed ):
        
        """This set motors speeds routine allows you to control the motor speeds in terms of metres
        per second"""
        self.robotMovementState = self.MOVEMENT_STATE_NORMAL
            
        # Calculcate our target encoder speeds
        self.leftMotorTargetTicksPerSecond = 1.0
        self.rightMotorTargetTicksPerSecond = 1.0
        
        self.settingMotorSpeedsWithEncoders = True
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def driveStraight( self, distance ):
        
        """Start the robot driving straight for a given distance"""
        if self.robotMovementState == self.MOVEMENT_STATE_NORMAL:
            
            # Get the current encoder readings
            encodersReading = self.miniDriver.getEncodersReading()
            leftEncoderReading, rightEncoderReading = encodersReading.data
            
            # Calculate distance to move wheels
            targetLeftEncoderReading = leftEncoderReading + distance*self.ENCODER_TICKS_PER_METRE
            targetRightEncoderReading = rightEncoderReading + distance*self.ENCODER_TICKS_PER_METRE
            
            # Setup the differential drive controller
            self.differentialDriveController.reset()
            self.differentialDriveController.setTargets( 
                targetLeftEncoderReading, targetRightEncoderReading,
                self.DEFAULT_SPEED_TICKS_PER_SECOND, self.DEFAULT_SPEED_TICKS_PER_SECOND )
            
            self.robotMovementState = self.MOVEMENT_STATE_DRIVING_STRAIGHT
    
    #-----------------------------------------------------------------------------------------------
    def _setRobotMovementState( self, movementState ):
        
        if self.robotMovementState != self.MOVEMENT_STATE_NORMAL:
            
            # Save out error values to a CSV file
            outputFilename = "errors_{0}.csv".format( int( time.time() ) )
            
            with open( outputFilename, "w" ) as csvFile:
                dictWriter = csv.DictWriter( csvFile, 
                    [ "Time", "TargetLeftSpeed", "ActualLeftSpeed", 
                        "TargetRightSpeed", "ActualRightSpeed",
                        "TargetLeftPos", "ActualLeftPos",
                        "TargetRightPos", "ActualRightPos",
                        "LeftMotorSignal", "RightMotorSignal" ] )
                dictWriter.writeheader()
                dictWriter.writerows( self.errorValuesList )
                
            self.errorValuesList = []
        
        self.robotMovementState = movementState
    
    #-----------------------------------------------------------------------------------------------
    def _loadPiSensorModule( self ):
        
        if self.robotConfig.piSensorModuleName != "":
            
            # Try to import the new sensor module
            newSensorModule = None
            try:
                
                newSensorModule = __import__( self.robotConfig.piSensorModuleName, fromlist=[''] )
                
            except Exception as e:
                logging.error( "Caught exception when trying to import Pi sensor module" )
                logging.error( str( e ) )
                
            if newSensorModule != None:
                
                # We have a new sensor module. Shutdown any existing sensor reader
                if self.piSensorReader != None:
                    self.piSensorReader.shutdown()
                    self.piSensorReader = None
                    
                # Remove reference to existing sensor module
                self.piSensorModule = None
                self.piSensorModuleName = ""
                
                # Try to create the new Pi sensor reader
                newSensorReader = None
                
                try:
                    
                    newSensorReader = newSensorModule.PiSensorReader()
                
                except Exception as e:
                    logging.error( "Caught exception when trying to create Pi sensor reader" )
                    logging.error( str( e ) )
                    
                if newSensorReader != None:
                    self.piSensorModule = newSensorModule
                    self.piSensorModuleName = self.robotConfig.piSensorModuleName
                    self.piSensorReader = newSensorReader
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if not self.miniDriver.isConnected():
            return
        
        curTime = time.time()
        timeDiff = min( curTime - self.lastUpdateTime, self.MAX_UPDATE_TIME_DIFF )
        
        # Update the robot's motor speeds based on its movement state
        if self.robotMovementState == self.MOVEMENT_STATE_NORMAL:     
                
            # Turn off the motors if we haven't received a motion command for a while
            if curTime - self.lastMotionCommandTime > self.MOTION_COMMAND_TIMEOUT:

                self.leftMotorSpeed = 0.0
                self.rightMotorSpeed = 0.0
                self.panSpeed = 0.0
                self.tiltSpeed = 0.0
            
        elif self.robotMovementState == self.MOVEMENT_STATE_TURNING \
            or self.robotMovementState == self.MOVEMENT_STATE_DRIVING_STRAIGHT:
            
            # Get encoder reading
            encodersReading = self.miniDriver.getEncodersReading()
            encodersTime = encodersReading.timestamp
            leftEncoderReading, rightEncoderReading = encodersReading.data
            
            # Calculate motor speeds
            self.leftMotorSpeed, self.rightMotorSpeed = self.differentialDriveController.update( 
                leftEncoderReading, rightEncoderReading, encodersTime )
            
            errorValues = {}
            errorValues[ "Time" ] = encodersTime - self.encoderMoveStartTime
            errorValues[ "TargetLeftSpeed" ] = self.differentialDriveController.debug_TargetLeftSpeed
            errorValues[ "ActualLeftSpeed" ] = self.differentialDriveController.debug_ActualLeftSpeed
            errorValues[ "TargetRightSpeed" ] = self.differentialDriveController.debug_TargetRightSpeed
            errorValues[ "ActualRightSpeed" ] = self.differentialDriveController.debug_ActualRightSpeed
            errorValues[ "TargetLeftPos" ] = self.differentialDriveController.debug_TargetLeftPos
            errorValues[ "ActualLeftPos" ] = self.differentialDriveController.debug_ActualLeftPos
            errorValues[ "TargetRightPos" ] = self.differentialDriveController.debug_TargetRightPos
            errorValues[ "ActualRightPos" ] = self.differentialDriveController.debug_ActualRightPos
            errorValues[ "LeftMotorSignal" ] = self.leftMotorSpeed
            errorValues[ "RightMotorSignal" ] = self.rightMotorSpeed

            self.errorValuesList.append( errorValues )
            
            # Revert to the normal movement state if the encoder based movement is complete
            if self.leftMotorSpeed == 0.0 and self.rightMotorSpeed == 0.0:
                self._setRobotMovementState( self.MOVEMENT_STATE_NORMAL )

        else:
            # Program flow should never get here, but reset to safe state if it does
            self._setRobotMovementState( self.MOVEMENT_STATE_NORMAL )
        
        # Update the pan and tilt angles
        self.panAngle += self.panSpeed*timeDiff
        self.tiltAngle += self.tiltSpeed*timeDiff
        
        self.panAngle = max( self.MIN_ANGLE, min( self.panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( self.tiltAngle, self.MAX_ANGLE ) )
        
        # Update the mini driver
        self.miniDriver.setOutputs(
            self.leftMotorSpeed, self.rightMotorSpeed, self.panAngle, self.tiltAngle )
        self.miniDriver.update()
        
        # Send servo settings if needed
        if curTime - self.lastServoSettingsSendTime >= self.TIME_BETWEEN_SERVO_SETTING_UPDATES:
            
            self.miniDriver.setPanServoLimits( 
                self.robotConfig.panPulseWidthMin, 
                self.robotConfig.panPulseWidthMax )
            self.miniDriver.setTiltServoLimits( 
                self.robotConfig.tiltPulseWidthMin, 
                self.robotConfig.tiltPulseWidthMax )
 
            self.lastServoSettingsSendTime = curTime
        
        # Send sensor configuration if needed
        if curTime - self.lastSensorConfigurationSendTime >= self.TIME_BETWEEN_SENSOR_CONFIGURATION_UPDATES:
            
            self.miniDriver.setSensorConfiguration( self.robotConfig.miniDriverSensorConfiguration )
 
            self.lastSensorConfigurationSendTime = curTime
        
        # Change the Pi sensor module if needed
        if self.robotConfig.piSensorModuleName != self.piSensorModuleName:
            self._loadPiSensorModule()
        
        # Read from any sensors attached to the Pi
        if self.piSensorReader != None:
            
            self.piSensorDict = {}
            try:
                self.piSensorDict = self.piSensorReader.readSensors()
            except Exception as e:
                logging.error( "Caught exception when trying to read from Pi sensor reader" )
                logging.error( str( e ) )
        
        self.lastUpdateTime = curTime