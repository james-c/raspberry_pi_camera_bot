
import pid
import time
import math

#---------------------------------------------------------------------------------------------------
class TrajectoryGenerator:
        
    #-----------------------------------------------------------------------------------------------
    def __init__( self, startEncoderTicks, targetEncoderTicks, maxSpeed, 
        maxAccDec, accelerateSmoothlyAtStart=True ):
        
        self.startEncoderTicks = startEncoderTicks
        self.targetEncoderTicks = targetEncoderTicks
        self.maxSpeed = abs( maxSpeed )
        self.maxAccDec = abs( maxAccDec )
        self.accelerateSmoothlyAtStart = accelerateSmoothlyAtStart
        
        timeToMaxSpeed = self.maxSpeed / self.maxAccDec
        fullAccDecDistance = maxAccDec*timeToMaxSpeed*timeToMaxSpeed
        
        distance = targetEncoderTicks - startEncoderTicks
        absDistance = abs( distance )
        self.goingForward = (distance > 0)
        
        if self.accelerateSmoothlyAtStart:
            
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
                
        else:
            
            # Assume that we're going at the max speed right from the start
            self.accDecTime = timeToMaxSpeed    # We only have deceleration time
            
            fullSpeedDistance = absDistance - fullAccDecDistance/2.0
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
        
        if self.accelerateSmoothlyAtStart:
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
                    
        else:
            
            # We skip the initial acceleration
            decTime = self.accDecTime
            if trajectoryTime < self.fullSpeedTime:
                
                # Full speed                
                velocity = speed
                pos += speed*trajectoryTime
                
            elif trajectoryTime < self.fullSpeedTime + decTime:
                
                # Decelerating
                remainingTime = trajectoryTime - self.fullSpeedTime
                
                velocity = speed + dec*remainingTime
                pos += speed*self.fullSpeedTime \
                    + speed*remainingTime + 0.5*dec*remainingTime*remainingTime
            
            else:
                
                # Finished
                velocity = 0.0
                pos += speed*self.fullSpeedTime \
                    + speed*decTime + 0.5*dec*decTime*decTime
        
        return velocity, pos
        
#---------------------------------------------------------------------------------------------------
class DifferentialDriveController:

    SMOOTHING_THRESHOLD = 400.0
    MIN_SPEED = 0     # Minimum motor speed in ticks per second
    BUFFER_SIZE = 5
    
    ENCODER_TICKS_ERROR_TOLERANCE = 1

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):

        # Mini Driver 6xAA
        #self.controlLoopLeft = pid.PID()
        #self.controlLoopLeft.setKp( 0.25 )
        #self.controlLoopLeft.setKi( 0.00 ) #0.0005 )
        #self.controlLoopLeft.setKd( 0.04 ) #0.005 )
    
        #self.controlLoopRight = pid.PID()
        #self.controlLoopRight.setKp( 0.25 )
        #self.controlLoopRight.setKi( 0.00 ) #0.0005 )
        #self.controlLoopRight.setKd( 0.04 ) #0.005 )
        
        # TODO: Expose these as configuration options
        # Rover 5
        self.controlLoopLeft = pid.PID()
        self.controlLoopLeft.setKp( 1.8 )
        self.controlLoopLeft.setKi( 0.005 ) #0.0005 )
        self.controlLoopLeft.setKd( 0.01 ) #0.005 )
    
        self.controlLoopRight = pid.PID()
        self.controlLoopRight.setKp( 1.8 )
        self.controlLoopRight.setKi( 0.005 ) #0.0005 )
        self.controlLoopRight.setKd( 0.01 ) #0.005 )
        
        self.leftTrajectoryGenerator = None
        self.rightTrajectoryGenerator = None

        self.maxAccDec = 1000.0
    
        self.reset()

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
        
        self.motorSignalLeft = 0.0
        self.motorSignalRight = 0.0
        
        self.updateCalled = False
        
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
        maxSpeedLeft, maxSpeedRight, maxAccDec, accelerateSmoothlyAtStart=True ):
    
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
            startEncoderTicksLeft, targetEncoderTicksLeft, maxSpeedLeft, 
            maxAccDec, accelerateSmoothlyAtStart )
        self.rightTrajectoryGenerator = TrajectoryGenerator(
            startEncoderTicksRight, targetEncoderTicksRight, maxSpeedLeft, 
            maxAccDec, accelerateSmoothlyAtStart )
    
    #-----------------------------------------------------------------------------------------------
    def getMoveTime( self ):
    
        return max( self.leftTrajectoryGenerator.getTrajectoryLength(),
            self.rightTrajectoryGenerator.getTrajectoryLength() ) + 1.0
    
    #-----------------------------------------------------------------------------------------------
    def update( self, leftEncoderReading, rightEncoderReading, currentTime ):
    
        if not self.updateCalled:
            
            self.lastEncoderTicksLeft = leftEncoderReading
            self.lastEncoderTicksRight = rightEncoderReading
            
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
        
        trajectoryTime = currentTime - self.startTime
        targetSpeedLeft, self.targetEncoderTicksLeft = \
            self.leftTrajectoryGenerator.getVelocityAndPos( trajectoryTime )
        targetSpeedRight, self.targetEncoderTicksRight = \
            self.rightTrajectoryGenerator.getVelocityAndPos( trajectoryTime )
        
        self.motorSignalLeft = self.controlLoopLeft.adjustSignal( 
            leftEncoderReading, self.targetEncoderTicksLeft, timeDelta )
        self.motorSignalRight = self.controlLoopRight.adjustSignal( 
            rightEncoderReading, self.targetEncoderTicksRight, timeDelta )
        
        # Constrain the motor signal 
        self.motorSignalLeft = max( -100, min( self.motorSignalLeft, 100 ) )
        self.motorSignalRight = max( -100, min( self.motorSignalRight, 100 ) )
        
        if trajectoryTime > self.leftTrajectoryGenerator.getTrajectoryLength():
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
        self.debug_SimpleFilterRightSpeed = simpleFilterSpeedRight
        
        return self.motorSignalLeft, self.motorSignalRight