
import pid

#---------------------------------------------------------------------------------------------------
class DifferentialDriveController:

    SMOOTHING_THRESHOLD = 1000.0
    MIN_SPEED = 250     # Minimum motor speed in ticks per second
    BUFFER_SIZE = 20
    
    ENCODER_TICKS_ERROR_TOLERANCE = 1

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):

        self.reset()

        self.controlLoopRight = pid.PID()
        self.controlLoopRight.setKp( 0.02 )
        self.controlLoopRight.setKi( 0.0006 )
        self.controlLoopRight.setKd( 0.001 )

        self.controlLoopLeft = pid.PID()
        self.controlLoopLeft.setKp( 0.02 )
        self.controlLoopLeft.setKi( 0.0006 )
        self.controlLoopLeft.setKd( 0.001 )
        
        self.setTargets( 
            targetEncoderTicksLeft=0, targetEncoderTicksRight=0,
            maxSpeedLeft=0, maxSpeedRight=0 )

    #-----------------------------------------------------------------------------------------------
    def reset( self ):
        
        self.lastTime = 0
        self.lastEncoderTicksLeft = 0
        self.lastEncoderTicksRight = 0 

        self.encoderTicksDeltaBufferLeft = [ 0 ] * self.BUFFER_SIZE
        self.encoderTicksDeltaBufferRight = [ 0 ] * self.BUFFER_SIZE
        self.timeDeltaBuffer = [ 0 ] * self.BUFFER_SIZE
        
    #-----------------------------------------------------------------------------------------------
    def setTargets( self, targetEncoderTicksLeft, targetEncoderTicksRight, maxSpeedLeft, maxSpeedRight ):
    
        """All speeds are in ticks per second"""
    
        self.targetEncoderTicksLeft = targetEncoderTicksLeft
        self.targetEncoderTicksRight = targetEncoderTicksRight
        self.maxSpeedLeft = abs( maxSpeedLeft )
        self.maxSpeedRight = abs( maxSpeedRight )
        
    #-----------------------------------------------------------------------------------------------
    def update( self, leftEncoderReading, rightEncoderReading, currentTime ):
    
        # Time delta Buffer
        timeDelta = currentTime - self.lastTime
        self.lastTime = currentTime
            
        self.timeDeltaBuffer.pop( 0 )
        self.timeDeltaBuffer.append( timeDelta )

        # Left motor encoder ticks delta buffer
        currentEncoderTicksLeft = leftEncoderReading
        encoderTicksDelta = currentEncoderTicksLeft - self.lastEncoderTicksLeft
        self.lastEncoderTicksLeft = currentEncoderTicksLeft
        
        self.encoderTicksDeltaBufferLeft.pop( 0 ) 
        self.encoderTicksDeltaBufferLeft.append( encoderTicksDelta )
            
        # Right motor distance delta Buffer
        currentEncoderTicksRight = rightEncoderReading
        encoderTicksDelta = currentEncoderTicksRight - self.lastEncoderTicksRight
        self.lastEncoderTicksRight = currentEncoderTicksRight
        
        self.encoderTicksDeltaBufferRight.pop( 0 ) 
        self.encoderTicksDeltaBufferRight.append( encoderTicksDelta )
            
        # Calculate Current Motor Speed     
        timeDeltaSum = float( sum( self.timeDeltaBuffer ) )
        if timeDeltaSum == 0.0:
            # Protect against divide by zero
            currentMotorSpeedLeft = 0.0
            currentMotorSpeedRight = 0.0
        else:
            currentMotorSpeedLeft = float( sum( self.encoderTicksDeltaBufferLeft ) ) / timeDeltaSum
            currentMotorSpeedRight = float( sum( self.encoderTicksDeltaBufferRight ) ) / timeDeltaSum
        
        # By default try to go at maximum speed
        targetSpeedLeft = self.maxSpeedLeft
        targetSpeedRight = self.maxSpeedRight
        
        # Reduce Left speed if aproaching target distance
        encoderErrorLeft = self.targetEncoderTicksLeft - leftEncoderReading
        
        if abs( encoderErrorLeft ) < self.SMOOTHING_THRESHOLD:
            targetSpeedLeft = ( abs( encoderErrorLeft ) * targetSpeedLeft ) / self.SMOOTHING_THRESHOLD
            targetSpeedLeft = max( self.MIN_SPEED, targetSpeedLeft )  # Make sure than the speed doesn't drop below MIN_SPEED
        
        if encoderErrorLeft < 0:
            targetSpeedLeft = -targetSpeedLeft
            
        if abs( encoderErrorLeft ) < self.ENCODER_TICKS_ERROR_TOLERANCE:
            targetSpeedLeft = 0
         
        # Reduce Right speed if aproaching target distance
        encoderErrorRight = self.targetEncoderTicksRight - rightEncoderReading
        
        if abs( encoderErrorRight ) < self.SMOOTHING_THRESHOLD:
            targetSpeedRight = ( abs( encoderErrorRight ) * targetSpeedRight ) / self.SMOOTHING_THRESHOLD
            targetSpeedRight = max( self.MIN_SPEED, targetSpeedRight )  # Make sure than the speed doesn't drop below MIN_SPEED
        
        if encoderErrorRight < 0:
            targetSpeedRight = -targetSpeedRight
            
        if abs( encoderErrorRight ) < self.ENCODER_TICKS_ERROR_TOLERANCE:
            targetSpeedRight = 0
        
        # Control the motor speed
        motorSignalLeft = self.controlLoopLeft.adjustSignal( currentMotorSpeedLeft, targetSpeedLeft )
        motorSignalRight = self.controlLoopRight.adjustSignal( currentMotorSpeedRight, targetSpeedRight )
        
        # Constrain the motor signal 
        motorSignalLeft = max( -100, min( motorSignalLeft, 100 ) )
        motorSignalRight = max( -100, min( motorSignalRight, 100 ) )
        
        print "Max motor speed", self.maxSpeedLeft
        print 'Motor speed left: ', currentMotorSpeedLeft
        print 'Encoder read left: ', leftEncoderReading
        
        print 'Motor speed right: ', currentMotorSpeedRight
        print 'Encoder read right: ', rightEncoderReading 
        
        print "Target left speed", targetSpeedLeft
        print "Target right speed", targetSpeedRight
        
        print "motorSignalLeft", motorSignalLeft
        print "motorSignalRight", motorSignalRight
        
        print '\n'
        
        #Correct Heading #Works only forwards !!!!!!!!!!!!!!!!!!!!!!!
        #headingError = ( targetSpeedRight - motorSpeedRight ) - ( targetSpeedLeft - motorSpeedLeft )
        #headingConstant = 0.01
        
        #if headingError > 0:

            #MotorSignalRight -= headingError * headingConstant
            #MotorSignalLeft += headingError * headingConstant
            
        #if headingError < 0:
            
            #MotorSignalRight += headingError * headingConstant
            #MotorSignalLeft -= headingError * headingConstant
        
        self.debug_TargetLeftSpeed = targetSpeedLeft
        self.debug_ActualLeftSpeed = currentMotorSpeedLeft
        self.debug_TargetRightSpeed = targetSpeedRight
        self.debug_ActualRightSpeed = currentMotorSpeedRight
        self.debug_TargetLeftPos = self.targetEncoderTicksLeft
        self.debug_ActualLeftPos = leftEncoderReading
        self.debug_TargetRightPos = self.targetEncoderTicksRight
        self.debug_ActualRightPos = rightEncoderReading
        
        return motorSignalLeft, motorSignalRight