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

import copy
import os
import os.path
import subprocess
import time
import logging

#---------------------------------------------------------------------------------------------------
class ScriptHandler:
    
    """This class allows users to run scripts located in a 'scripts directory'. The ScriptHandler
       is responsible for starting the scripts, catching any exceptions they may throw, and for
       shutting them down if needed"""
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, scriptsDir ):
            
        self.scriptProcess = None
        self.activeScriptName = None
        self.scriptsDir = scriptsDir
        self.scriptFilenames = {}
        
        # Scan the scripts directory to look for available scripts
        for dirpath, dirnames, filenames in os.walk( self.scriptsDir ):
        
            for filename in filenames:
                
                scriptFilename = os.path.abspath( os.path.join( dirpath, filename ) )
                scriptName = self._getScriptName( scriptFilename )
                if scriptName != None:
                    
                    if scriptName in self.scriptFilenames:
                        logging.warning( "Found multiple scripts with the name {0}".format( scriptName ) )
                    else:
                        self.scriptFilenames[ scriptName ] = scriptFilename
                        logging.info( "Found script {0} - {1}".format( scriptName, scriptFilename ) )

    #-----------------------------------------------------------------------------------------------
    def __del__( self ):

        self.stopScript()
    
    #-----------------------------------------------------------------------------------------------
    def _getScriptName( self, scriptFilename ):
        
        """Scans through a script to see if it can find a SCRIPT_NAME variable that identifies
           it. We use this technique rather than importing the file to avoid any side effects
           that may be produced by running the script Python code early. This routine returns
           the script name if found, None if no name is found"""
           
        scriptName = None
        
        try:
            with open( scriptFilename ) as scriptFile:
                
                for line in scriptFile:
                    
                    line = line.strip()
                    lineElements = line.split( "=" )
                    
                    if len( lineElements ) >= 2:
                        
                        varName = lineElements[ 0 ].strip()
                        varValue = lineElements[ 1 ].strip()
                        
                        if len( varName ) > 0 and varName == "SCRIPT_NAME" \
                            and len( varValue ) > 0 and varValue[ 0 ] != "#":
                                
                            scriptName = varValue.strip( "'\"" )
                    
                
        except Exception:
           
           # Ignore any errors that may arise
           pass
       
        return scriptName
    
    #-----------------------------------------------------------------------------------------------
    def startScript( self, scriptName, args=[] ):
        
        # Attempt to start script if one isn't already running
        if self.scriptProcess == None \
            or self.scriptProcess.poll() != None:
            
            if scriptName in self.scriptFilenames:

                try:
                    self.scriptProcess = subprocess.Popen( 
                        [ self.scriptFilenames[ scriptName ] ] + args )
                    self.activeScriptName = scriptName
                    logging.error( "Started script {0}".format( scriptName ) )
                    
                except Exception as e:
                    logging.error( "Caught exception when attempting " \
                        + "to start script {0} - {1}".format( scriptName, str( e ) ) )
                    self.scriptProcess = None
                    self.activeScriptName = None
                    
            else:
                
                logging.error( "Tried to start unknown script {0}".format( scriptName ) )
        
        else:
            
            logging.warning( "Attempted to start script {0} when " \
                + "script {1} was already running".format( scriptName, self.activeScriptName ) )
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):

        """This routine monitors the status of the running script if there is one"""
    
        if self.scriptProcess != None:
            
            if self.scriptProcess.poll() != None:
                
                logging.info( "The script {0} has ended".format( self.activeScriptName ) )
                self.scriptProcess = None
                self.activeScriptName = None
                
    #-----------------------------------------------------------------------------------------------
    def stopScript( self ):

        if self.scriptProcess != None:
            self.scriptProcess.terminate()
            self.scriptProcess = None
            self.activeScriptName = None
