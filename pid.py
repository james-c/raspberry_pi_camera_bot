
#---------------------------------------------------------------------------------------------------
class PID:

    #-----------------------------------------------------------------------------------------------              
    def __init__( self ):
    
        # Signal
        self.signalIn = 0
        self.signalOut = 0
        
        #Signal error
        self.error = 0
        self.errorIntegral = 0
        self.errorDerivative = 0
        
        self.acceptableError = 0

        # Default control loop constants   
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        
        # Signal min and max constrain
        self.minSignal = -60
        self.maxSignal = 60 
        
    #-----------------------------------------------------------------------------------------------              
    def resetErrorValues( self ):
        self.error = 0
        self.errorIntegral = 0
        self.errorDerivative = 0
        
    #-----------------------------------------------------------------------------------------------          
    def setSignalConstrain( self, minVal, maxVal ):
        self.minSignal, self.maxSignal = minVal, maxVal
        
    #-----------------------------------------------------------------------------------------------            
    def setAcceptableError( self, acceptableError ):
        self.acceptableError = acceptableError

    #-----------------------------------------------------------------------------------------------
    def setKp( self, Kp ):
        self.Kp = Kp

    #-----------------------------------------------------------------------------------------------
    def setKi( self, Ki ):
        self.Ki = Ki

    #-----------------------------------------------------------------------------------------------
    def setKd( self, Kd ):
        self.Kd = Kd
    
    #-----------------------------------------------------------------------------------------------
    # Control signal value 

    def adjustSignal( self, signalIn, targetValue ):
        
        self.signalIn = signalIn
        self.targetValue = targetValue
        
        #error
        self.error = self.targetValue - self.signalIn
        
        #Acceptable error band
        if abs(self.error) <= self.acceptableError:
            self.error = 0
            self.errorIntegral = 0
            self.errorDerivative = 0
            
        else:
            #error Integration
            self.errorIntegral = self.errorIntegral + self.error
            #self.errorIntegral = self.constrain(self.errorIntegral, -40, 40)
            
            #error Derivatieve
            self.errorDerivative = self.error - self.errorDerivative
            
        #Sum of errors times their gains
        self.signalOut = (self.Kp*self.error) + (self.Ki*self.errorIntegral) + (self.Kd*self.errorDerivative)
        self.signalOut = self.constrain(self.signalOut, self.minSignal, self.maxSignal)
        
        return self.signalOut

    #-----------------------------------------------------------------------------------------------
    # Constrain function

    def constrain( self, value, minValue, maxValue  ):   
        return max( minValue, min( value, maxValue ) )