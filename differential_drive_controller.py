
import pid
import time
import math

#---------------------------------------------------------------------------------------------------
class FilterIIR:
    
    """A simple IIR filter implementation that can be used to incrementally filter a signal.
       It is assumed that the signal is sampled at a regular rate."""
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, B, A=[] ):
        
        """Filter constructor which takes the IIR numerator coefficients (B) and denominator
           coefficients (A). Note: We assume that the first denominator is 1.0 and not 
           provided."""
        
        assert( len( B ) > 0 )
        
        self.B = B
        self.A = A
        self.reset()
        
    #-----------------------------------------------------------------------------------------------
    def reset( self ):
        
        """Resets the filter, clearing out all previous inputs and outputs"""
        
        self.x = [ 0.0 for i in range( len( self.B ) ) ]
        self.y = [ 0.0 for i in range( len( self.A ) ) ]
        if len( self.A ) == 0:
            self.y = [ 0.0 ]
    
    #-----------------------------------------------------------------------------------------------
    def getLastOutput( self ):
        
        return self.y[ 0 ]
    
    #-----------------------------------------------------------------------------------------------
    def stepFilter( self, newX ):
        
        """Steps the filter with a new input, and returns the latest output"""
        
        # Add the new input to our list of inputs
        self.x = [ newX ] + self.x[ :-1 ]     
        
        # Calculate the new output
        sumB = 0.0
        for i in range( len( self.B ) ):
            sumB += self.x[ i ]*self.B[ i ]
            
        sumA = 0.0    
        for i in range( len( self.A ) ):
            sumA += self.y[ i ]*self.A[ i ]
        
        newY = sumB + sumA
        
        # Add the new output to our list of outputs
        self.y = [ newY ] + self.y[ :-1 ]
        
        return newY

#---------------------------------------------------------------------------------------------------
class TrajectoryGenerator:
        
    #-----------------------------------------------------------------------------------------------
    def __init__( self, startEncoderTicks, targetEncoderTicks, maxSpeed, maxAccDec ):
        
        self.startEncoderTicks = startEncoderTicks
        self.targetEncoderTicks = targetEncoderTicks
        self.maxSpeed = abs( maxSpeed )
        self.maxAccDec = abs( maxAccDec )
        
        timeToMaxSpeed = self.maxSpeed / self.maxAccDec
        fullAccDecDistance = maxAccDec*timeToMaxSpeed*timeToMaxSpeed
        
        distance = targetEncoderTicks - startEncoderTicks
        absDistance = abs( distance )
        self.goingForward = (distance > 0)
        
        if absDistance <= fullAccDecDistance:
            
            # We can't accelerate to full speed
            self.accDecTime = 2.0*math.sqrt( absDistance / self.maxAccDec )
            self.fullSpeedTime = 0.0
            
            accTime = self.accDecTime/2.0            
            self.maxSpeed = self.maxAccDec*accTime
            
        else:
            
            # We accelerate to full speed and then decelerate
            self.accDecTime = 2.0*timeToMaxSpeed
            
            fullSpeedDistance = absDistance - fullAccDecDistance
            self.fullSpeedTime = fullSpeedDistance / self.maxSpeed
    
    #-----------------------------------------------------------------------------------------------
    def getTrajectoryLength( self ):
        
        return self.accDecTime + self.fullSpeedTime
    
    #-----------------------------------------------------------------------------------------------
    def getVelocityAndPos( self, trajectoryTime ):
        
        if trajectoryTime < 0.0:
            trajectoryTime = 0.0
        
        velocity = 0.0
        pos = self.startEncoderTicks
        
        if self.goingForward:
            acc = self.maxAccDec
            dec = -self.maxAccDec
            speed = self.maxSpeed
        else:
            acc = -self.maxAccDec
            dec = self.maxAccDec
            speed = -self.maxSpeed
            
        accTime = self.accDecTime/2.0
        if trajectoryTime < accTime:
            
            # Accelerating
            velocity = acc*trajectoryTime
            pos += 0.5*acc*trajectoryTime*trajectoryTime
            
        elif trajectoryTime < accTime + self.fullSpeedTime:
            
            # Full speed
            remainingTime = trajectoryTime - accTime
            
            velocity = speed
            pos += 0.5*acc*accTime*accTime + speed*remainingTime
            
        elif trajectoryTime < accTime + self.fullSpeedTime + accTime:
            
            # Decelerating
            remainingTime = trajectoryTime - (accTime + self.fullSpeedTime)
            
            velocity = speed + dec*remainingTime
            pos += 0.5*acc*accTime*accTime + speed*self.fullSpeedTime \
                + speed*remainingTime + 0.5*dec*remainingTime*remainingTime
        
        else:
            
            # Finished
            velocity = 0.0
            pos += 0.5*acc*accTime*accTime + speed*self.fullSpeedTime \
                + speed*accTime + 0.5*dec*accTime*accTime
        
        return velocity, pos
        
#---------------------------------------------------------------------------------------------------
class DifferentialDriveController:

    SMOOTHING_THRESHOLD = 400.0
    MIN_SPEED = 0     # Minimum motor speed in ticks per second
    BUFFER_SIZE = 5
    
    ENCODER_TICKS_ERROR_TOLERANCE = 1

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):

        self.controlLoopLeft = pid.PID()
        self.controlLoopLeft.setKp( 0.25 )
        self.controlLoopLeft.setKi( 0.00 ) #0.0005 )
        self.controlLoopLeft.setKd( 0.04 ) #0.005 )
    
        self.controlLoopRight = pid.PID()
        self.controlLoopRight.setKp( 0.25 )
        self.controlLoopRight.setKi( 0.00 ) #0.0005 )
        self.controlLoopRight.setKd( 0.04 ) #0.005 )
        
        self.leftTrajectoryGenerator = None
        self.rightTrajectoryGenerator = None
    
        # We use a 2nd order lowpass Butterworth filter with a cutoff frequency of 20Hz 
        # (100Hz sample rate) to filter calculated encoder velocity
        FILTER_B = [ 0.010313, 0.061877, 0.154693, 0.206257, 0.154693, 0.061877, 0.010313 ]
        FILTER_A = [ -1.1876007, 1.3052133, -0.6743275, 0.2634693, -0.0517530, 0.0050225 ]
        
        self.leftEncoderFilter = FilterIIR( FILTER_B, FILTER_A )
        self.rightEncoderFilter = FilterIIR( FILTER_B, FILTER_A )
    
        self.maxAccDec = 1000.0
    
        self.reset()
        
        self.setTargets( 
            targetEncoderTicksLeft=0, targetEncoderTicksRight=0,
            maxSpeedLeft=0, maxSpeedRight=0 )

    #-----------------------------------------------------------------------------------------------
    def reset( self ):
        
        self.controlLoopLeft.reset()
        self.controlLoopRight.reset()
        self.oldLeftSpeed = 0.0
        self.oldRightSpeed = 0.0
        
        self.lastTime = time.time()
        self.lastEncoderTicksLeft = 0
        self.lastEncoderTicksRight = 0 
        self.startTime = time.time()

        self.encoderTicksDeltaBufferLeft = [ 0 ] * self.BUFFER_SIZE
        self.encoderTicksDeltaBufferRight = [ 0 ] * self.BUFFER_SIZE
        self.timeDeltaBuffer = [ 0 ] * self.BUFFER_SIZE
        
        self.leftEncoderFilter.reset()
        self.rightEncoderFilter.reset()
        
        self.motorSignalLeft = 0.0
        self.motorSignalRight = 0.0
        
        self.updateCalled = False

    #-----------------------------------------------------------------------------------------------
    def setTargets( self, targetEncoderTicksLeft, targetEncoderTicksRight, maxSpeedLeft, maxSpeedRight ):
    
        """All speeds are in ticks per second"""
    
        self.targetEncoderTicksLeft = targetEncoderTicksLeft
        self.targetEncoderTicksRight = targetEncoderTicksRight
        self.maxSpeedLeft = abs( maxSpeedLeft )
        self.maxSpeedRight = abs( maxSpeedRight )
        
    #-----------------------------------------------------------------------------------------------
    def getTrajectoryTargetTicks( self ):
        
        if self.leftTrajectoryGenerator == None:
            return None
            
        else:
            v, p1 = self.leftTrajectoryGenerator.getVelocityAndPos( 
                self.leftTrajectoryGenerator.getTrajectoryLength() + 1.0 )
            v, p2 = self.rightTrajectoryGenerator.getVelocityAndPos( 
                self.rightTrajectoryGenerator.getTrajectoryLength() + 1.0 )
            return p1, p2
        
    #-----------------------------------------------------------------------------------------------
    def startMove( self, startEncoderTicksLeft, startEncoderTicksRight, 
        targetEncoderTicksLeft, targetEncoderTicksRight, 
        maxSpeedLeft, maxSpeedRight, maxAccDec ):
    
        """All speeds are in ticks per second"""
    
        self.reset()
    
        self.startEncoderTicksLeft = startEncoderTicksLeft
        self.startEncoderTicksRight = startEncoderTicksRight
        self.targetEncoderTicksLeft = targetEncoderTicksLeft
        self.targetEncoderTicksRight = targetEncoderTicksRight
        self.maxSpeedLeft = abs( maxSpeedLeft )
        self.maxSpeedRight = abs( maxSpeedRight )
        self.maxAccDec = abs( maxAccDec )            
        
        self.leftTrajectoryGenerator = TrajectoryGenerator(
            startEncoderTicksLeft, targetEncoderTicksLeft, maxSpeedLeft, maxAccDec )
        self.rightTrajectoryGenerator = TrajectoryGenerator(
            startEncoderTicksRight, targetEncoderTicksRight, maxSpeedLeft, maxAccDec )
    
    #-----------------------------------------------------------------------------------------------
    def getMoveTime( self ):
    
        return max( self.leftTrajectoryGenerator.getTrajectoryLength(),
            self.rightTrajectoryGenerator.getTrajectoryLength() ) + 1.0
    
    #-----------------------------------------------------------------------------------------------
    def update( self, leftEncoderReading, rightEncoderReading, currentTime ):
    
        if not self.updateCalled:
            
            self.lastTargetLeftEncoderReading = leftEncoderReading
            self.lastTargetRightEncoderReading = rightEncoderReading
            
            self.updateCalled = True
    
        # Time delta Buffer
        timeDelta = currentTime - self.lastTime
        self.lastTime = currentTime
            
        self.timeDeltaBuffer.pop( 0 )
        self.timeDeltaBuffer.append( timeDelta )

        # Left motor encoder ticks delta buffer
        currentEncoderTicksLeft = leftEncoderReading
        leftEncoderTicksDelta = currentEncoderTicksLeft - self.lastEncoderTicksLeft
        self.lastEncoderTicksLeft = currentEncoderTicksLeft
        
        self.encoderTicksDeltaBufferLeft.pop( 0 ) 
        self.encoderTicksDeltaBufferLeft.append( leftEncoderTicksDelta )
            
        # Right motor distance delta Buffer
        currentEncoderTicksRight = rightEncoderReading
        rightEncoderTicksDelta = currentEncoderTicksRight - self.lastEncoderTicksRight
        self.lastEncoderTicksRight = currentEncoderTicksRight
        
        self.encoderTicksDeltaBufferRight.pop( 0 ) 
        self.encoderTicksDeltaBufferRight.append( rightEncoderTicksDelta )
            
        # Calculate Current Motor Speed     
        timeDeltaSum = float( sum( self.timeDeltaBuffer ) )
        if timeDeltaSum == 0.0:
            # Protect against divide by zero
            currentMotorSpeedLeft = 0.0
            currentMotorSpeedRight = 0.0
        else:
            currentMotorSpeedLeft = float( sum( self.encoderTicksDeltaBufferLeft ) ) / timeDeltaSum
            currentMotorSpeedRight = float( sum( self.encoderTicksDeltaBufferRight ) ) / timeDeltaSum
        
        simpleFilterSpeedLeft = currentMotorSpeedLeft
        simpleFilterSpeedRight = currentMotorSpeedRight
        
        # Instantaneous speed...
        if timeDelta == 0.0:
            # Protect against divide by zero
            instantSpeedLeft = 0.0
            instantSpeedRight = 0.0
        else:
            instantSpeedLeft = float( leftEncoderTicksDelta ) / timeDelta
            instantSpeedRight = float( rightEncoderTicksDelta ) / timeDelta

        # New filtered speed
        #F = 0.1
        #currentMotorSpeedLeft = F*instantSpeedLeft + (1.0-F)*self.oldLeftSpeed
        #currentMotorSpeedRight = F*instantSpeedRight + (1.0-F)*self.oldRightSpeed
        #self.oldLeftSpeed = currentMotorSpeedLeft
        #self.oldRightSpeed = currentMotorSpeedRight
        
        # By default try to go at maximum speed
        moveTime = currentTime - self.startTime
        if moveTime < 0.5:
            targetSpeedLeft = self.maxSpeedLeft * moveTime/0.5
            targetSpeedRight = self.maxSpeedRight * moveTime/0.5
        elif moveTime > 2.5:
            targetSpeedLeft = 0.0
            targetSpeedRight = 0.0
        elif moveTime > 2.0:
            targetSpeedLeft = self.maxSpeedLeft * (1.0 - (moveTime - 2.0)/0.5)
            targetSpeedRight = self.maxSpeedRight * (1.0 -(moveTime - 2.0)/0.5)
        else:
            targetSpeedLeft = self.maxSpeedLeft
            targetSpeedRight = self.maxSpeedRight
        
        self.targetEncoderTicksLeft = self.lastTargetLeftEncoderReading + timeDelta*targetSpeedLeft
        self.targetEncoderTicksRight = self.lastTargetRightEncoderReading + timeDelta*targetSpeedRight
        self.lastTargetLeftEncoderReading = self.targetEncoderTicksLeft
        self.lastTargetRightEncoderReading = self.targetEncoderTicksRight
        
        # Reduce Left speed if aproaching target distance
        encoderErrorLeft = self.targetEncoderTicksLeft - leftEncoderReading
        
        #if abs( encoderErrorLeft ) < self.SMOOTHING_THRESHOLD:
            #targetSpeedLeft = ( abs( encoderErrorLeft ) * targetSpeedLeft ) / self.SMOOTHING_THRESHOLD
            #targetSpeedLeft = max( self.MIN_SPEED, targetSpeedLeft )  # Make sure than the speed doesn't drop below MIN_SPEED
        
        #if encoderErrorLeft < 0:
        #    targetSpeedLeft = -targetSpeedLeft
            
        if abs( encoderErrorLeft ) < self.ENCODER_TICKS_ERROR_TOLERANCE:
            targetSpeedLeft = 0
         
        # Reduce Right speed if aproaching target distance
        encoderErrorRight = self.targetEncoderTicksRight - rightEncoderReading
        
        #if abs( encoderErrorRight ) < self.SMOOTHING_THRESHOLD:
            #targetSpeedRight = ( abs( encoderErrorRight ) * targetSpeedRight ) / self.SMOOTHING_THRESHOLD
            #targetSpeedRight = max( self.MIN_SPEED, targetSpeedRight )  # Make sure than the speed doesn't drop below MIN_SPEED
        
        #if encoderErrorRight < 0:
        #    targetSpeedRight = -targetSpeedRight
            
        if abs( encoderErrorRight ) < self.ENCODER_TICKS_ERROR_TOLERANCE:
            targetSpeedRight = 0
        
        # Control the motor speed
        self.motorSignalLeft = 0.015 * targetSpeedLeft
        self.motorSignalRight = 0.015 * targetSpeedRight
        #self.motorSignalLeft =  self.controlLoopLeft.adjustSignal( 
            #currentMotorSpeedLeft, targetSpeedLeft, timeDelta )
        #self.motorSignalRight = self.controlLoopRight.adjustSignal( 
            #currentMotorSpeedRight, targetSpeedRight, timeDelta )
        
        
        targetSpeedLeft, self.targetEncoderTicksLeft = \
            self.leftTrajectoryGenerator.getVelocityAndPos( moveTime )
        targetSpeedRight, self.targetEncoderTicksRight = \
            self.rightTrajectoryGenerator.getVelocityAndPos( moveTime )
        
        self.motorSignalLeft = self.controlLoopLeft.adjustSignal( 
            leftEncoderReading, self.targetEncoderTicksLeft, timeDelta )
        self.motorSignalRight = self.controlLoopRight.adjustSignal( 
            rightEncoderReading, self.targetEncoderTicksRight, timeDelta )
        
        
        #print currentMotorSpeedLeft, targetSpeedLeft, diff
        
        #if self.motorSignalLeft > -15.0 and self.motorSignalLeft < 15.0:
            #self.motorSignalLeft = 0.0
            
        #if self.motorSignalRight > -15.0 and self.motorSignalRight < 15.0:
            #self.motorSignalRight = 0.0
        
        # Constrain the motor signal 
        self.motorSignalLeft = max( -100, min( self.motorSignalLeft, 100 ) )
        self.motorSignalRight = max( -100, min( self.motorSignalRight, 100 ) )
        
        #print "Max motor speed", self.maxSpeedLeft
        #print 'Motor speed left: ', currentMotorSpeedLeft
        #print 'Encoder read left: ', leftEncoderReading
        
        #print 'Motor speed right: ', currentMotorSpeedRight
        #print 'Encoder read right: ', rightEncoderReading 
        
        #print "Target left speed", targetSpeedLeft
        #print "Target right speed", targetSpeedRight
        
        #print "motorSignalLeft", self.motorSignalLeft
        #print "motorSignalRight", self.motorSignalRight
        
        #print '\n'
        
        #Correct Heading #Works only forwards !!!!!!!!!!!!!!!!!!!!!!!
        #headingError = ( targetSpeedRight - motorSpeedRight ) - ( targetSpeedLeft - motorSpeedLeft )
        #headingConstant = 0.01
        
        #if headingError > 0:

            #MotorSignalRight -= headingError * headingConstant
            #MotorSignalLeft += headingError * headingConstant
            
        #if headingError < 0:
            
            #MotorSignalRight += headingError * headingConstant
            #MotorSignalLeft -= headingError * headingConstant
        
        if moveTime > self.leftTrajectoryGenerator.getTrajectoryLength():
            print "Error:", self.targetEncoderTicksLeft - leftEncoderReading
        
        self.debug_TargetLeftSpeed = targetSpeedLeft
        self.debug_ActualLeftSpeed = currentMotorSpeedLeft
        self.debug_TargetRightSpeed = targetSpeedRight
        self.debug_ActualRightSpeed = currentMotorSpeedRight
        self.debug_TargetLeftPos = self.targetEncoderTicksLeft
        self.debug_ActualLeftPos = leftEncoderReading
        self.debug_TargetRightPos = self.targetEncoderTicksRight
        self.debug_ActualRightPos = rightEncoderReading
        self.debug_InstantLeftSpeed = instantSpeedLeft
        self.debug_InstantRightSpeed = instantSpeedRight
        self.debug_SimpleFilterLeftSpeed = simpleFilterSpeedLeft
        self.debug_SimpleFilterRightSpeed = simpleFilterSpeedLeft
        
        return self.motorSignalLeft, self.motorSignalRight