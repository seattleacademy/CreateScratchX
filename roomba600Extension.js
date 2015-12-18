// roomba600Extension.js
// Roomba 600 Scratch Extension

// Copyright (c) 2015 iRobot Corporation
// http://www.irobot.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of iRobot Corporation nor the names
//   of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written
//   permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

(function (ext) {
    "use strict";

    var device = null;
    var rawData = null;
    var dockingq = 0;

    var safeModeOpcode = 131;

    var oiModes = {
        'off'     : 0,
        'passive' : 1,
        'safe'    : 2,
        'full'    : 3
        };

    // Sensor states:
    var sensorIndices = {
        // Individual sensors
        'bumps-wheeldrops': 7,
        'wall': 8,
        'cliff-left': 9,
        'cliff-front-left': 10,
        'cliff-front-right': 11,
        'cliff-right': 12,
        'virtual-wall': 13,
        'overcurrents': 14,
        'dirt-detect': 15,
        'ir-opcode': 17,
        'buttons': 18,
        'distance': 19,
        'angle': 20,
        'charging-state': 21,
        'voltage': 22,
        'current': 23,
        'temperature': 24,
        'battery-charge': 25,
        'battery-capacity': 26,
        'wall-signal': 27,
        'cliff-left-signal': 28,
        'cliff-front-left-signal': 29,
        'cliff-front-right-signal': 30,
        'cliff-right-signal': 31,
        'charger-available': 34,
        'open-interface-mode': 35,
        'song-number': 36,
        'song-playing?': 37,
        'oi-stream-num-packets': 38,
        'velocity': 39,
        'radius': 40,
        'velocity-right': 41,
        'velocity-left': 42,
        'encoder-counts-left': 43,
        'encoder-counts-right': 44,
        'light-bumper': 45,
        'light-bump-left': 46,
        'light-bump-front-left': 47,
        'light-bump-center-left': 48,
        'light-bump-center-right': 49,
        'light-bump-front-right': 50,
        'light-bump-right': 51,
        'ir-opcode-left': 52,
        'ir-opcode-right': 53,
        'left-motor-current': 54,
        'right-motor-current': 55,
        'main-brush-current': 56,
        'side-brush-current': 57,
        'stasis': 58,

        // Sensor groups
        'group-0': 0,
        'group-1': 1,
        'group-2': 2,
        'group-3': 3,
        'group-4': 4,
        'group-5': 5,
        'group-6': 6,
        'group-100': 100,
        'group-101': 101,
        'group-106': 106,
        'group-107': 107
    };

    var sensorCache = {
        // Individual sensors
        'bumps-wheeldrops': 0,
        'wall': 0,
        'cliff-left': 0,
        'cliff-front-left': 0,
        'cliff-front-right': 0,
        'cliff-right': 0,
        'virtual-wall': 0,
        'overcurrents': 0,
        'dirt-detect': 0,
        'ir-opcode': 0,
        'buttons': 0,
        'distance': 0,
        'angle': 0,
        'charging-state': 0,
        'voltage': 0,
        'current': 0,
        'temperature': 0,
        'battery-charge': 0,
        'battery-capacity': 0,
        'wall-signal': 0,
        'cliff-left-signal': 0,
        'cliff-front-left-signal': 0,
        'cliff-front-right-signal': 0,
        'cliff-right-signal': 0,
        'charger-available': 0,
        'open-interface-mode': 0,
        'song-number': 0,
        'song-playing?': 0,
        'oi-stream-num-packets': 0,
        'velocity': 0,
        'radius': 0,
        'velocity-right': 0,
        'velocity-left': 0,
        'encoder-counts-left': 0,
        'encoder-counts-right': 0,
        'light-bumper': 0,
        'light-bump-left': 0,
        'light-bump-front-left': 0,
        'light-bump-center-left': 0,
        'light-bump-center-right': 0,
        'light-bump-front-right': 0,
        'light-bump-right': 0,
        'ir-opcode-left': 0,
        'ir-opcode-right': 0,
        'left-motor-current': 0,
        'right-motor-current': 0,
        'main-brush-current': 0,
        'side-brush-current': 0,
        'stasis': 0
    };

    var sensorLengths = [26, 10, 6, 10, 14, 12, 52, // Groups  0  - 6
                         1, 1, 1, 1, 1, 1, 1, 1,    // Sensors 7  - 14
                         1, 1, 1, 1, 2, 2, 1, 2,    // Sensors 15 - 22
                         2, 1, 2, 2, 2, 2, 2, 2,    // Sensors 23 - 30
                         2, 1, 2, 1, 1, 1, 1, 1,    // Sensors 31 - 38
                         2, 2, 2, 2, 2, 2, 1, 2,    // Sensors 39 - 46
                         2, 2, 2, 2, 2, 1, 1, 2,    // Sensors 47 - 54
                         2, 2, 2, 1                 // Sensors 55 - 58
                        ];


    // Wrapper that will also print out the opcodes being sent to the browser console.
    function sendToRobot(cmd) {
        // TODO fix funny printing of single byte arrays...
        console.log("Sending " + Array.apply([], cmd).join(","));
        device.send(cmd.buffer);
    }


    // Private logic

    function getSensor(which) {
        return sensorCache[which];
    }

    function anyCliff() {
        return (   (getSensor('cliff-left') & 0x01)
                || (getSensor('cliff-front-left') & 0x01)
                || (getSensor('cliff-front-right') & 0x01)
                || (getSensor('cliff-right') & 0x01) );
    }

    function getBooleanSensor(which) {
        if (device === null) {
            return false;
        }

        if (which === 'robot control allowed' &&
            (getSensor('open-interface-mode') == oiModes['safe']) ) { return true; }

        // bumps-wheeldrops
        if (which === 'right bumper' && (getSensor('bumps-wheeldrops') & 0x01)) { return true; }
        if (which === 'left bumper'  && (getSensor('bumps-wheeldrops') & 0x02)) { return true; }
        if (which === 'bumper'       && (getSensor('bumps-wheeldrops') & 0x03)) { return true; }
        if (which === 'right wheel'  && (getSensor('bumps-wheeldrops') & 0x04)) { return true; }
        if (which === 'left wheel'   && (getSensor('bumps-wheeldrops') & 0x08)) { return true; }
        if (which === 'any wheel'    && (getSensor('bumps-wheeldrops') & 0x0C)) { return true; }

        if (which === 'left cliff'        && (getSensor('cliff-left') & 0x01))        { return true; }
        if (which === 'front-left cliff'  && (getSensor('cliff-front-left') & 0x01))  { return true; }
        if (which === 'front-right cliff' && (getSensor('cliff-front-right') & 0x01)) { return true; }
        if (which === 'right cliff'       && (getSensor('cliff-right') & 0x01))       { return true; }
        if (which === 'any cliff' && anyCliff())      { return true; }
        if (which === 'cliff' && anyCliff()) { return true; }

        if (which === 'virtual wall' && (getSensor('virtual-wall') & 0x01))  { return true; }

        // overcurrents
        if (which === 'side-brush stall'  && (getSensor('overcurrents') & 0x01)) { return true; }
        if (which === 'main-brush stall'  && (getSensor('overcurrents') & 0x04)) { return true; }
        if (which === 'right wheel stall' && (getSensor('overcurrents') & 0x08)) { return true; }
        if (which === 'left wheel stall'  && (getSensor('overcurrents') & 0x10)) { return true; }
        if (which === 'stall'             && (getSensor('overcurrents') & 0xFF)) { return true; }

        // buttons
        if (which === 'clean button'    && (getSensor('buttons') & 0x01)) { return true; }
        if (which === 'spot button'     && (getSensor('buttons') & 0x02)) { return true; }
        if (which === 'dock button'     && (getSensor('buttons') & 0x04)) { return true; }
        if (which === 'minute button'   && (getSensor('buttons') & 0x08)) { return true; }
        if (which === 'hour button'     && (getSensor('buttons') & 0x10)) { return true; }
        if (which === 'day button'      && (getSensor('buttons') & 0x20)) { return true; }
        if (which === 'schedule button' && (getSensor('buttons') & 0x40)) { return true; }
        if (which === 'clock button'    && (getSensor('buttons') & 0x80)) { return true; }
        if (which === 'any button'      && (getSensor('buttons') & 0xFF)) { return true; }
            
        // charging state
        if (which === 'charging error'    && (getSensor('charging-state') === 5)) { return true; }
        if (which === 'recovery charging' && (getSensor('charging-state') === 1)) { return true; }
        if (which === 'full charging'     && (getSensor('charging-state') === 2)) { return true; }
        if (which === 'trickle charging'  && (getSensor('charging-state') === 3)) { return true; }
        if (which === 'not charging'      && (getSensor('charging-state') === 0)) { return true; }
        if (which === 'charging'          && (getSensor('charging-state') > 0) && (getSensor('charging-state') < 5)) { return true; }

        // charger available
        if (which === 'charger plugged in'   &&  (getSensor('charger-available') & 0x01)) { return true; }
        if (which === 'robot on dock'        &&  (getSensor('charger-available') & 0x02)) { return true; }
        if (which === 'charger not detected' && !(getSensor('charger-available') & 0x03)) { return true; }

        if (which === 'song playing' && (getSensor('song-playing?') & 0x01)) { return true; }
        
        // Light bumper sections
        if ((which === 'wall')       && (getSensor('light-bumper') & 0xFF)) { return true; }
        if ((which === 'wall left')  && (getSensor('light-bumper') & 0x03)) { return true; }
        if ((which === 'wall front') && (getSensor('light-bumper') & 0x1E)) { return true; }
        if ((which === 'wall right') && (getSensor('light-bumper') & 0x30)) { return true; }
        // Individual light bumpers
        if (which === 'left light bumper activated'         && (getSensor('light-bumper') & 0x01)) { return true; }
        if (which === 'front-left light bumper activated'   && (getSensor('light-bumper') & 0x02)) { return true; }
        if (which === 'center-left light bumper activated'  && (getSensor('light-bumper') & 0x04)) { return true; }
        if (which === 'center-right light bumper activated' && (getSensor('light-bumper') & 0x08)) { return true; }
        if (which === 'front-right light bumper activated'  && (getSensor('light-bumper') & 0x10)) { return true; }
        if (which === 'right light bumper activated'        && (getSensor('light-bumper') & 0x20)) { return true; }
        if (which === 'light bumper activated' && (getSensor('light-bumper') & 0xFF)) { return true; }

        return false;
    }

    ext.resetAll = function () {};


    // Hats
    ext.whenSensorConnected = function (which) {
        return getBooleanSensor(which);
    };

    ext.whenSensorPass = function (which, sign, level) {
        if (sign === '<')
        {
            return getSensor(which) < level;
        }
        else if (sign === '>')
        {
            return getSensor(which) > level;
        }
        else
        {
            return getSensor(which) === level;
        }
    };


    // Reporters
    ext.booleanSensor = function (which) {
        return getBooleanSensor(which);
    };

    ext.sensor = function (which) {
        return getSensor(which);
    };


    // Commands
    
    var tempo = 60; // In beats/minute
    ext.playNote = function(note, beats, callback) {
        var duration = (beats/tempo)*60*64;
        if (duration < 1) {
            alert("Roomba cannot play a note for that short.\\n"
                  + "Increase the beats and/or reduce the tempo");
        } else if (duration > 255) {
            alert("Roomba cannot play a note for that long.\\n"
                  + "Decrease the beats and/or increase the tempo");
        }
        if ((note > 107) || ((note < 31) && (note !== 0))) // 0 = rest note.
        {
            alert("Roomba only has a note range of 31 to 107.\\n"
                  + "Please select within this range.\\n"
                  + "To rest, use 0 as the note value.");
            callback();
        }
        var cmd = new Uint8Array([140, // Load 'song' (note in this case)
                                  0,   // Song number (irrelevant)
                                  1,
                                  note,
                                  duration,
                                  141, // Play 'song'
                                  0
                                 ]);
        sendToRobot(cmd);

        window.setTimeout(function() {
            callback();
        }, (1000*duration/64)); // Duration is 1/64th seconds. Convert to ms.
    }

    ext.setTempo = function(desiredTempo) {
        // Assuming 32nd note at fastest roomba can (60sec/min * 64 * 4/32)
        if (desiredTempo > 500) {
            alert("This tempo is too fast for Roomba. Please set to under 500 bpm");
        }
        // Assuming quarter note at slowest roomba can (255/64 = 3.98 seconds)
        else if (desiredTempo < 20) {
            alert("This tempo is too slow for Roomba. Please set to over 20 bpm");
        }
        else
        {
            tempo = desiredTempo;
        }
    }

    var topDriveSpeed = 300;
    var driveSpeed = topDriveSpeed;
    var driveRadius = 32768;
    var robotDriving = 0;


    ext.setDriveSpeed = function(speed) {
        // Speed passed in as a percentage of the normal drive speed of 300 mm/s
        // Silently cap speed percentage between 0 and 100%.
        if (speed > 100)
        {
            speed = 100;
        }
        else if (speed < 0)
        {
            speed = 0;
        }
        driveSpeed = (topDriveSpeed*speed)/100;

        console.info("Set speed to " + speed);
    };

    ext.setDriveRadius = function(radius) {
        /* Radius must be between -2000 mm and 2000 mm or
         * signify straight (32767/32768).
         */
        if (((radius <= 2000) && (radius >= -2000)) ||
            (radius === 32767) || (radius === 32768) )
        {
            driveRadius = radius;
        }
        else
        {
            alert("Radius must be between -2000 and 2000 mm or be 32768 (straight)");
        }
        console.info("Set drive radius to " + radius);
    };

    ext.drive = function() {
        if (driveSpeed === 0)
        {
            alert("Drive speed is set to 0%.");
        }
        go(driveRadius, driveSpeed);
    }

    function go(radius, speed) {
        var radius_twos = radius;
        var speed_twos = speed;

        dockingq = 0; // No longer docking.

        if(radius < 0) {
            radius_twos = 65535 + radius;
        }

        if(speed < 0) {
            speed_twos = 65535 + speed;
        }
        
        robotDriving = (speed != 0); // Used for constant bump safety check

        var cmd = new Uint8Array([137,  // Drive
                                  (speed_twos >> 8) & 0xFF,
                                  speed_twos & 0xFF,
                                  (radius_twos >> 8) & 0xFF,
                                  radius_twos & 0xFF
                                 ]);
        sendToRobot(cmd);
        cmd = null;  // Is this necessary?
    }

    function calculateEncoderDiffs(startEnc, endEnc) {
        var diffs = [endEnc[0] - startEnc[0],
                     endEnc[1] - startEnc[1]];

        if (diffs[0] >= 65535)  { diffs[0] -= 65535; }
        else if (diffs[0] <= -65535) { diffs[0] += 65535; }

        if (diffs[1] >= 65535)  { diffs[1] -= 65535; }
        else if (diffs[1] <= -65535) { diffs[1] += 65535; }

        return diffs;
    }

    function calculateDistanceCM(startEnc, endEnc) {
        // From OI Spec (see sensor packets 43 and 44).
        var distanceClicksPerMM = 0.28117374;
        var diffs = calculateEncoderDiffs(startEnc, endEnc);
        var distanceClicks = (diffs[0] + diffs[1])/16;

        return (distanceClicks / distanceClicksPerMM)/10; // Convert to cm.
    }

    function calculateAngleDeg(startEnc, endEnc) {
        // From OI Spec (see sensor packets 43 and 44).
        var angleClicksPerDeg = 0.5692593;
        var diffs = calculateEncoderDiffs(startEnc, endEnc);
        var angleClicks = (diffs[1] - diffs[0])/16;

        return (angleClicks / angleClicksPerDeg);
    }


    var startEncoders;

    var driveUntilDistance   = 0;
    var distanceTraveled     = 0;
    var estimatedDistanceTraveled = 0;
    var distanceCountOfIntervals  = 0;
    var distanceCountOfLastUpdate = 0;

    var turnUntilAngle    = 0;
    var angleTraveled     = 0;
    var estimatedAngleTraveled = 0;
    var angleCountOfIntervals  = 0;
    var angleCountOfLastUpdate = 0;


    function checkEncoderDistance(interval, callback) {
        var endEncoders = [getSensor('encoder-counts-left'),
                           getSensor('encoder-counts-right')];
        var distanceUpdate = calculateDistanceCM(startEncoders, endEncoders);
        if (distanceUpdate) {
            distanceTraveled += distanceUpdate;
            estimatedDistanceTraveled = distanceTraveled;
            distanceCountOfLastUpdate = distanceCountOfIntervals - 1;
        }
        // If no updates since drive was requested, assume we are moving at
        // least half as fast as the input drive speed (converted to CM).
        else if (distanceTraveled == 0) {
            estimatedDistanceTraveled += ((driveSpeed/10) * interval/1000)/2;
        }

        // Estimate how much we moved (and will move by the next update).
        // Require at least 2 updates before we are accurate (update takes ~3 intervals)
        else if (distanceCountOfLastUpdate > 3) {
            estimatedDistanceTraveled += distanceTraveled/distanceCountOfLastUpdate;
        }

        console.log("Drove " + estimatedDistanceTraveled
                    + " of " + driveUntilDistance + " cm --" + distanceTraveled);

        // Do not update until we know the robot is moving.
        if (distanceTraveled !== 0) {
            distanceCountOfIntervals++;
        }

        if (   (Math.ceil(Math.abs(estimatedDistanceTraveled)) < Math.abs(driveUntilDistance))
            && getBooleanSensor('robot control allowed')
               // In case another block stops the robot from driving.
            && robotDriving )
        {
            startEncoders = endEncoders;

            setTimeout(function() {
                checkEncoderDistance(interval, callback);
            }, interval);
        }
        else
        {
            go(32768,0);
            callback();
        }
    }

    function checkEncoderAngle(interval, callback) {
        var endEncoders = [getSensor('encoder-counts-left'),
                           getSensor('encoder-counts-right')];
        var angleUpdate = calculateAngleDeg(startEncoders, endEncoders);
        if (angleUpdate) {
            angleTraveled += angleUpdate;
            estimatedAngleTraveled = angleTraveled;
            angleCountOfLastUpdate = angleCountOfIntervals - 1;
        }
        // If no updates since turn was requested, assume we are moving at
        // least a little bit.
        else if (angleTraveled == 0) {
            estimatedAngleTraveled += 0.5 * driveSpeed/topDriveSpeed;
        }

        // Estimate how much we moved (and will move by the next update).
        // Require at least 2 updates before we are accurate (update takes ~3 intervals)
        if (angleCountOfLastUpdate > 3 ) {
            estimatedAngleTraveled += angleTraveled/angleCountOfLastUpdate;
        }
        
        console.log("Turned " + estimatedAngleTraveled
                    + " of " + turnUntilAngle + " deg --" + angleTraveled);

        // Do not update until we know the robot is moving.
        if (angleTraveled !== 0) {
            angleCountOfIntervals++;
        }

        if (   (Math.ceil(Math.abs(estimatedAngleTraveled)) < Math.abs(turnUntilAngle))
            && getBooleanSensor('robot control allowed')
               // In case another block stops the robot from driving.
            && robotDriving )
        {
            startEncoders = endEncoders;

            setTimeout(function() {
                checkEncoderAngle(interval, callback);
            }, interval);
        }
        else
        {
            go(32768,0);
            callback();
        }
    }

    function turnUntil(direction, degrees, callback) {
        // Allow only a single full rotation per call. Avoids
        // encoder roll over complications.
        if ((degrees > 360) || (degrees < 0))
        {
            alert("Angle must be between 0 and 360 degrees.");
            return callback();
        }
        if (driveSpeed === 0)
        {
            alert("Drive speed is set to 0%.");
            return callback();
        }
        if(direction === 'clockwise')
        {
            go(-1, driveSpeed);
        }
        else
        {
            go(1, driveSpeed);
        }

        // Wait until traveled angle degrees
        startEncoders = [getSensor('encoder-counts-left'),
                         getSensor('encoder-counts-right')];

        turnUntilAngle = degrees;
        angleTraveled  = 0;
        estimatedAngleTraveled = 0;
        angleCountOfIntervals  = 0;
        angleCountOfLastUpdate = 0;
        
        console.info("Turning until " + degrees + " degrees");
        checkEncoderAngle(5, callback);
    };

    ext.turnUntilLeft = function(degrees, callback) {
        turnUntil('counter-clockwise', degrees, callback);
    }

    ext.turnUntilRight = function(degrees, callback) {
        turnUntil('clockwise', degrees, callback);
    }

    function driveDistance(distance, callback) {
        // Check for the maximum distance that can be measured
        //var distanceClicksPerMM = 0.28117374; // From the Roomba 600 code
        // Max distance is 65535 * distanceClicksPerMM ~ 18.4 meters.
        // Rounding to 15 meters
        if (Math.abs(distance) > 1500 )
        {
            alert("Distance must be between -1500 and 1500 centimeters.");
            return callback();
        }
        if (driveSpeed === 0)
        {
            alert("Drive speed is set to 0%.");
            return callback();
        }

        if (distance > 0)
        {
            go(driveRadius, driveSpeed);
        }
        else
        {
            go(driveRadius, -driveSpeed);
        }

        // Wait until traveled distance mm
        startEncoders = [getSensor('encoder-counts-left'),
                         getSensor('encoder-counts-right')];

        driveUntilDistance = distance;
        distanceTraveled   = 0;
        estimatedDistanceTraveled = 0;
        distanceCountOfIntervals  = 0;
        distanceCountOfLastUpdate = 0;

        console.info("Driving until " + distance + " mm");
        checkEncoderDistance(5, callback);
    };

    ext.driveUntil = function(distance, callback) {
        driveDistance(distance, callback);
    }

    ext.stop = function() {
        go(32768,0);
    };

    var motorStatus = {
        'side-brush-on': 0,
        'vacuum-on'    : 0,
        'main-brush-on': 0,
        'side-brush-cw': 0,
        'main-brush-cw': 0,
    };

    function getMotorEnableByte(mStatusObj) {
        return ((mStatusObj['main-brush-cw'] << 4)
                + (mStatusObj['side-brush-cw'] << 3)
                + (mStatusObj['main-brush-on'] << 2)
                + (mStatusObj['vacuum-on'] << 1)
                + (mStatusObj['side-brush-on']) );
    }

    function sendNonWheelMotors() {
        var cmd = new Uint8Array([138, getMotorEnableByte(motorStatus)]);
        sendToRobot(cmd);
    }

    ext.vacuumEnable = function(enabled) {
        if (enabled === 'off')
        {
            motorStatus['vacuum-on'] = 0;
        }
        else
        {
            motorStatus['vacuum-on'] = 1;
        }
        sendNonWheelMotors();
    };

    ext.motorEnable = function(motor, direction) {
        switch(motor)
        {
        case 'side brush':
            if (direction === 'off')
            {
                motorStatus['side-brush-on'] = 0;
            }
            else
            {
                motorStatus['side-brush-on'] = 1;
                if (direction === 'clockwise')
                {
                    motorStatus['side-brush-cw'] = 1;
                }
                else
                {
                    motorStatus['side-brush-cw'] = 0;
                }
            }
            break;
        case 'main brush':
            if (direction === 'off')
            {
                motorStatus['main-brush-on'] = 0;
            }
            else
            {
                motorStatus['main-brush-on'] = 1;
                if (direction === 'clockwise')
                {
                    motorStatus['main-brush-cw'] = 0;
                }
                else
                {
                    motorStatus['main-brush-cw'] = 1;
                }
            }
            break;
        }

        sendNonWheelMotors();
    };


    var ledStates = {
        'check-robot': 0,
        'dock': 0,
        'spot': 0,
        /* Debris is displayed on startup to avoid confusion
           of all LEDs being off (could be misinterpreted as sleeping */
        'debris': 1,
        'Sat': 0,
        'Fri': 0,
        'Thu': 0,
        'Wed': 0,
        'Tue': 0,
        'Mon': 0,
        'Sun': 0,
        'Schedule': 0,
        'Clock': 0,
        'AM': 0,
        'PM': 0,
        ':' : 0,
        'powerColor' : 0,
        'powerIntensity' : 0
    };

    function setLEDs() {
        var ledBits = (ledStates['check-robot'] << 3)
            + (ledStates['dock'] << 2)
            + (ledStates['spot'] << 1)
            + ledStates['debris'];

        var cmd = new Uint8Array([139,
                                  ledBits,
                                  ledStates['powerColor'],
                                  ledStates['powerIntensity']
                                 ]);
        sendToRobot(cmd);
        cmd = null;
    }

    function setSchedulingLEDs() {
        var weekdayBits = (ledStates['Sat'] << 6)
            + (ledStates['Fri'] << 5)
            + (ledStates['Thu'] << 4)
            + (ledStates['Wed'] << 3)
            + (ledStates['Tue'] << 2)
            + (ledStates['Mon'] << 1)
            + ledStates['Sun'];

        var schedulingBits = (ledStates['Schedule'] << 4)
            + (ledStates['Clock'] << 3)
            + (ledStates['AM'] << 2)
            + (ledStates['PM'] << 1)
            + ledStates[':'];

        var cmd = new Uint8Array([162,
                                  weekdayBits,
                                  schedulingBits
                                 ]);
        sendToRobot(cmd);
        cmd = null;
    }

    ext.setLED = function(led, state) {
        if (state === 'on')
        {
            ledStates[led] = 1;
        }
        else if(state === 'off')
        {
            ledStates[led] = 0;
        }
        else
        {
            ledStates[led] = state;
        }

        setLEDs();
        setSchedulingLEDs();
    };

    ext.setCleanLEDIntensity = function(intensity) {
        // Silently cap intensity between 0 and 100%.
        if (intensity > 100)
        {
            intensity = 100;
        }
        else if (intensity < 0)
        {
            intensity = 0;
        }

        ledStates['powerIntensity'] = (intensity*255)/100;

        setLEDs();
    };

    ext.setCleanLEDColor = function(color) {
        switch(color)
        {
        case 'green':
            ledStates['powerColor'] = 0;
            break;
        case 'red':
            ledStates['powerColor'] = 255;
            break;
        case 'amber':
            ledStates['powerColor'] = 128;
            break;
        case 'yellow':
            ledStates['powerColor'] = 47;
            break;
        default:
            if (color < 0)
            {
                color = 0;
            }
            else if (color > 255)
            {
                color = 255;
            }
            ledStates['powerColor'] = color;
            break;
        }

        setLEDs();
    };


    var digits = [0,0,0,0];

    function setDigits() {
        var cmd = new Uint8Array([164,
                                  digits[3],
                                  digits[2],
                                  digits[1],
                                  digits[0]
                                 ]);
        sendToRobot(cmd);
    }

    ext.setDigits = function(dig3, dig2, dig1, dig0) {
        digits[0] = dig0.charCodeAt(0);
        digits[1] = dig1.charCodeAt(0);
        digits[2] = dig2.charCodeAt(0);
        digits[3] = dig3.charCodeAt(0);

        setDigits();
    };


    /* Safety Code */
    var overcurrentCounts = {
        'side-brush'  : 0,
        'main-brush'  : 0,
        'right wheel' : 0,
        'left wheel'  : 0
    };
    var overcurrentOffTrigger = 32;  // ~0.5 second sounds good.
    var bumperCounts = 0;

    /*  Call this after an error is detected to avoid warning for multiple
     *  safety checks at the same time.
     */
    function resetSafetyHandlers() {
        // To avoid worrying about browser compatability...
        overcurrentCounts = {
            'side-brush'  : 0,
            'main-brush'  : 0,
            'right wheel' : 0,
            'left wheel'  : 0
        };
        bumperCounts = 0;
    }

    function handleOvercurrent(motor) {
        overcurrentCounts[motor] += getBooleanSensor(motor.concat(' stall'))*2  - 1;
        // Don't allow it to go below 0
        overcurrentCounts[motor] = Math.max(parseInt(overcurrentCounts[motor]), 0);
        // Turn off motor if overcurrenting for too long
        if (overcurrentCounts[motor] > overcurrentOffTrigger)
        {
            console.warn("Motor overcurrented: " + motor);
            if ((motor === 'side-brush') || (motor === 'main-brush'))
            {
                console.log(motor.concat('-on'));
                motorStatus[motor.concat('-on')] = 0;
                sendNonWheelMotors();
            }
            // Stop wheels no matter what caused the error. The alert will stop
            // the blocks so we want to avoid running without control.
            go(32768,0);
            resetSafetyHandlers(); // Reset counts to prevent multiple alerts
            // Alert after stopping because the alert is blocking...
            alert("Safety Error: Overcurrent detected!\\n"
                  + "Please check " + motor + " motor.");
        }
    }

    function badBumper() {
        return getBooleanSensor('bumper') && robotDriving;
    }

    function handleConstantBumper() {
        bumperCounts += badBumper()*2 - 1;
        // Don't allow it to go below 0
        bumperCounts = Math.max(parseInt(bumperCounts), 0);
        // Turn off wheels if bumper is triggered for ~10 seconds
        if (bumperCounts > (10/0.015))  // 15 ms updates
        {
            console.warn("Bumper pressed for too long");
            go(32768,0)
            resetSafetyHandlers(); // Reset counts to prevent multiple alerts
            alert("Safety Error: Bumper pressed for too long!\\n"
                  + "Please check bumper and avoid running Roomba "
                  + "into objects for long periods of time.");
        }
    }

    function safetyChecks() {
        // Make sure it's in safe mode
        if (   (getSensor('open-interface-mode') != oiModes['safe'])
            && (dockingq == 0) )
        {
            // If no safety issues, re-enter safe mode
            if (!(   getBooleanSensor('any wheel')
                  /* Not checking if cliffs are active because this
                     only matters if driving forward.
                  || getBooleanSensor('any cliff')
                  */
                  || getBooleanSensor('charger plugged in') ))
            {
                console.warn("Robot fell out of safe mode.");
                sendToRobot(new Uint8Array([safeModeOpcode]));
                // Re-enable LEDs since they were disabled in passive mode
                setLEDs();
                setSchedulingLEDs();
                setDigits();
            }
        }
        else // Robot is in safe mode and can drive. Check safety conditions.
        {
            handleOvercurrent('side-brush');
            handleOvercurrent('main-brush');
            handleOvercurrent('right wheel');
            handleOvercurrent('left wheel');
            handleConstantBumper();
        }
    }


    var inputArray = [];
    // Process complete packets (framing done previously)
    function processData(data) {
        var bytes = new Uint8Array(data);

        // Check the checksum
        var sum = 0;
        for(var i=0; i<bytes.byteLength; i++) {
            sum += bytes[i];
            if(sum > 255) {
                sum -= 256;
            }
        }

        if((sum & 0xFF) === 0) {
            // Checksum is valid
            
            if(watchdog) {
                console.log("Robot verified. Clearing watchdog");
                clearTimeout(watchdog);
                watchdog = null;
                /* Set debris led on now that robot is verified.
                   (Confusing if no LEDs on at startup) */
                var streamCmd = new Uint8Array([
                    139, 1, 0, 0
                ]);

                sendToRobot(streamCmd);
            }

            // Make sure this device is still in potentialDevices so
            // that when the script is reloaded, we don't lose contact
            // with Roomba
            if(device) {
                if(potentialDevices.indexOf(device) === -1) {
                    potentialDevices.unshift(device);
                }
            }

            var index = 2;

            while(index < (bytes.byteLength - 1)) {
                var opcode = bytes[index];

                //console.log('Opcode ' + opcode);

                if(opcode < sensorLengths.length) {
                    if(sensorLengths[opcode] === 1) {
                        inputArray[opcode] = bytes[index + 1];
                    } else if(sensorLengths[opcode] === 2) {
                        // Two's complement
                        inputArray[opcode] = bytes[index + 2] + ((bytes[index + 1] & 0xFF) << 8);
                    } else {
                        // Unsupported opcode
                        console.warn("Read unsupported opcode " + opcode);
                        return;
                    }

                    // Advance to the next opcode
                    index = index + sensorLengths[opcode] + 1;
                } else {
                    console.warn("Opcode is >= " + sensorLengths.length);
                    index = index + 1;
                }
            }
            for(var name in sensorCache) {
                if(sensorCache.hasOwnProperty(name)) {
                    var v = inputArray[sensorIndices[name]];          
                    sensorCache[name] = v;
                }
            }

            safetyChecks();
        }

        //console.log(sensorCache);
        rawData = null;
        bytes   = null;
    }

    function appendBuffer( buffer1, buffer2 ) {
        var tmp = new Uint8Array( buffer1.byteLength + buffer2.byteLength );
        tmp.set( new Uint8Array( buffer1 ), 0 );
        tmp.set( new Uint8Array( buffer2 ), buffer1.byteLength );
        return tmp.buffer;
    }

    ext.whenDeviceConnected = function() {
        if (device) { return true; }
        return false;
    };
    

    // Extension API interactions
    var potentialDevices = [];
    ext._deviceConnected = function(dev) {
        potentialDevices.push(dev);

        console.info("Device plugged in " + dev.id);
        if (!device)
        {
            tryNextDevice();
        }
    }

    function tryNextDevice() {
        device = potentialDevices.shift();

        if (device)
        {
            console.info("Trying to open device " + device.id);
            device.open({ stopBits: 0, bitRate: 115200, ctsFlowControl: 0 }, deviceOpened);
        }
    }

    var watchdog = null;
    function deviceOpened(dev) {
        if (!dev) {
            // Opening the port failed
            tryNextDevice();
            return;
        }

        // Receive streamed data. This script is not set up to handle
        // non-streamed data!
        device.set_receive_handler(function(data) {
            while(data.byteLength > 0) {
                var dataBytes = new Uint8Array(data);
                var pktStart = dataBytes.indexOf(19);
                var curPktLength, curPktRead;

                if(rawData && (pktStart !== 0)) {
                    // At least some bytes may belong to the previous packet
                    if(rawData.byteLength === 1) {
                        // Only the 19 was picked up earlier
                        curPktLength = dataBytes[0];
                        curPktRead = -1; // We also need to read the length byte
                    } else {
                        curPktLength = rawData[1];
                        curPktRead = rawData.byteLength - 2; // data - header size
                    }
                    // Copy rest of packet
                    // Add 1 for the checksum
                    var nToRead = curPktLength - curPktRead + 1;

                    // Copy data from this buffer to the rawData
                    // accumulator to add on to the packet in progress
                    if(nToRead < rawData.byteLength) {
                        rawData = appendBuffer(rawData, data.slice(0,nToRead));
                        // Consider the rest of the string in the next iteration
                        data = data.slice(nToRead);
                    } else {
                        rawData = appendBuffer(rawData, data.slice(0));
                        // We've used all the data
                        data = "";
                    }

                    // Check for a complete packet
                    if(rawData.byteLength === (curPktLength + 3)) {
                        // Process this packet
                        processData(rawData);
                    }
                } else if(pktStart !== -1) {
                    // There's a packet here, and rawData is either
                    // empty or the packet begins at 0 of data, in
                    // which case whatever is currently in rawData is
                    // moot.

                    if(data.byteLength >= (2 + pktStart)) {
                        // We have a length. Check for a complete packet.
                        curPktLength = dataBytes[pktStart + 1];
                        if((data.byteLength - pktStart) >= curPktLength + 3) {
                            // Process this packet
                            processData(data.slice(pktStart,pktStart + curPktLength + 3));
                            // Consider the rest of the string in the next iteration
                            data = data.slice(pktStart + curPktLength + 3);
                        } else {
                            // The packet isn't complete. Copy the rest and wait.
                            if(!rawData) {
                                rawData = new Uint8Array(data.slice(pktStart));
                            } else {
                                rawData = data.slice(pktStart);
                            }

                            // We've copied the entire data 
                            data = "";
                        }
                    } else {
                        // There's no length in the data; copy
                        // everything and wait for the next
                        // transmission.

                        if(!rawData) {
                            rawData = new Uint8Array(data.slice(pktStart));
                        } else {
                            rawData = data.slice(pktStart);
                        }

                        // We've copied the entire data 
                        data = "";
                    }
                } else {
                    // There's no packet in this string and we weren't
                    // building a packet in rawData previously. Clear
                    // out the data and ignore it.
                    data = "";
                }

                dataBytes = null;
            }
        });

        // Tell the Roomba to stream some sensors. Go into full mode on start
        // to avoid switching back and forth between modes and having to re-enable
        // components (such as LEDs)
        var streamCmd = new Uint8Array([
            /* Enable OI */
            128,
            /* Enter safe mode */
            safeModeOpcode,
            /* Request stream */
            148,
            /* Number of packets requested */
            12,
            /* List of packets */
            7,             // Bumps and wheel-drops
            9, 10, 11, 12, // Cliffs
            14,            // Overcurrents (handled internally)
            18,            // Buttons
            34,            // Charge source available
            35,            // OI Mode
            43, 44,        // Encoder counts
            45             // Light bumper
        ]);

        sendToRobot(streamCmd);

        watchdog = setTimeout(function() {
            // This device didn't get good data in time, so give up on
            // it. Clean up and then move on. If we get good data
            // then we'll terminate this watchdog.
            device.set_receive_handler(null);
            device.close();
            device = null;
            rawData = null;
            tryNextDevice();
        }, 3000); // Give Roomba 3 seconds to respond with a complete packet
    };

    ext._deviceRemoved = function (dev) {
        console.warn("Device removed");
        if(device !== dev) return;
        device = null;
        rawData = null;
    };

    function disconnectRobot() {
        var cmd = new Uint8Array([128, // Start command puts robot in passive mode
                                  173  // Stop sci
                                 ]);
        sendToRobot(cmd);
        device.close();
        device = null;
        rawData = null;
    }

    ext.disconnect = function() {
        if (device) disconnectRobot();
        device = null;
        rawData = null;
    }

    ext._shutdown = function() {
        console.info("shutdown...");
        if(device) disconnectRobot();
        device = null;
        rawData = null;
    };

    ext._getStatus = function () {
        if(!device) return {status: 1, msg: 'Roomba disconnected'};
        if(watchdog) return {status: 1, msg: 'Looking for Roomba'};
        return {status: 2, msg: 'Roomba connected'};
    };

    ext.clean = function() {
        // Send start command to put in passive first, then clean command
        var cmd = new Uint8Array([128, 135]);
        sendToRobot(cmd);
        disconnectRobot();
        alert("Running cleaning mission. Robot disconnected.");
    };

    function turnAfterDock(callback)
    {
        // Wait until the backup has completed to turn.
        if (!robotDriving)
        {
            turnUntil('clockwise', 180, callback)
        }
        else
        {
            window.setTimeout(function() {
                turnAfterDock(callback);
        }, 15);
        }
    }

    function checkIfDocked(callback)
    {
        // If robot docks or was picked up, we've 'docked'.
        if (   getBooleanSensor('robot on dock')
            || getBooleanSensor('any wheel') )
        {
            dockingq = 0;
            console.log("Robot docked!");
            callback();
        }
        else
        {
            window.setTimeout(function() {
                checkIfDocked(callback);
            }, 15);
        }
    }


    ext.dock = function(type, callback) {
        // Docking mission
        if (type === 'go to')
        {
            dockingq = 1;
            // Send start command to put in passive first, then dock command
            var cmd = new Uint8Array([128, 143]);
            sendToRobot(cmd);
            checkIfDocked(callback);
        }
        // Leavind dock mission.
        else if (type === 'leave')
        {
            // Only perform if robot is on the dock.
            if (getBooleanSensor('robot on dock'))
            {
                driveSpeed = 175;
                driveDistance(-20);
                turnAfterDock(callback);
            }
            else
            {
                callback();
            }
        }
    };


    // Most of the OI is not supported due to issues streaming more
    // than 13 or so packets, but I left in the supporting code.
    // The basic opcodes here should enable most elementary projects.

    var descriptor = {
        // [ Type, String, Callback, Default menu values ]
        // Types: 
        // ' ' 	Synchronous command
        // 'w' 	Asynchronous command
        // 'r' 	Synchronous reporter
        // 'R' 	Asynchronous reporter
        // 'h' 	Hat block (synchronous, returns boolean, true = run stack)

        blocks: [
            ['h', 'when %m.detectSensors detected',  'whenSensorConnected', 'wall'],
            ['h', 'when %m.buttonBumper is pressed', 'whenSensorConnected', 'any button'],
            /* Disable this for now. Wheel drops are not useful in safe mode.
             * ['h', 'when %m.wheelDrop is dropped', 'whenSensorConnected', 'any wheel'],
             */

            /* Disable this for now. No useful analog sensors currently.
             * ['h', 'when %m.sensor %m.lessMore %n', 'whenSensorPass', 'sensor', '>', 0],
             */

            ['b', '%m.detectSensors detected',  'booleanSensor', 'wall'],
            ['b', '%m.buttonBumper is pressed', 'booleanSensor', 'any button'],
            /* Disable this for now. Wheel drops are not useful in safe mode.
             * ['b', '%m.wheelDrop is dropped',    'booleanSensor', 'any wheel'],
             */

            /* Disable this for now. No useful analog sensors currently.
             * ['r', '%m.sensor value', 'sensor', 'sensor'],
             */

            // Songs
            [' ', 'set note tempo to %n bpm', 'setTempo', '60'],
            ['w', 'play note %d.note for %n beats', 'playNote', '60', '0.5'],

            // Motion
            [' ', 'set drive speed to %n%', 'setDriveSpeed', '100'],
            /* Disable this for now. Wait for full featured version.
             * [' ', 'set drive radius to %n mm', 'setDriveRadius', '32768'],
             */
            [' ', 'drive', 'drive'],
            ['w', 'drive %n cm', 'driveUntil', '100', 'driveUntil'],
            ['w', 'turn @turnLeft %n degrees', 'turnUntilLeft',  '90'],
            ['w', 'turn @turnRight %n degrees','turnUntilRight', '90'],
            [' ', 'stop driving', 'stop'],
            /* Disable this for now. Wait for full featured version.
             * [' ', 'turn vacuum %m.onOff', 'vacuumEnable', 'off'],
             * [' ', 'turn %m.motors motor %m.motorEnable', 'motorEnable', 'main brush', 'off'],
             */

            // LEDs
            [' ', 'turn %m.led LED %m.onOff','setLED','debris','on'],
            [' ', 'set display to %s %s %s %s', 'setDigits', '0', '0', '0', '0'],
            [' ', 'set clean LED brightness to %n%', 'setCleanLEDIntensity', '100'],
            [' ', 'set clean LED color to %m.color', 'setCleanLEDColor', 'green'],
            /* Disable this for now. Wait for full featured version.
             * [' ', 'set clean LED color to %n','setCleanLEDColor','0'],
             */

            // Commands
            ['w', '%m.dock the dock','dock', 'go to', 'dock']
        ],
        menus: {
            color:          [ 'green','red','amber','yellow'],
            led:            [ 'check-robot',
                              'debris'
                              /* Disable this for now. Wait for full featured version.
                              'dock',
                              'spot',
                              'Sun',
                              'Mon',
                              'Tue',
                              'Wed',
                              'Thu',
                              'Fri',
                              'Sat',
                              'AM',
                              'PM',
                              ':'
                              */
                              ],
            // Vacuum is not in the 'motor' list because it is only allowed
            // to be on or off (not forward, reverse, off).
            motors:         [ 'side brush','main brush'],
            motorEnable:    [ 'off', 'counter-clockwise', 'clockwise'],
            onOff:          [ 'on','off'],
            buttonBumper:   [ 'clean button',
                              'spot button',
                              'dock button',
                              /* Disable this for now. Wait for full featured version.
                              'minute button',
                              'hour button',
                              'day button',
                              */
                              /* Bug with these two buttons right now.
                              'schedule button',
                              'clock button',
                              */
                              'any button',
                              // Bumper
                              'right bumper',
                              'left bumper',
                              'bumper'
                            ],
            cliff:          [ 'left cliff',
                              'front-left cliff',
                              'front-right cliff',
                              'right cliff',
                              'any cliff' ],
            wheelDrop:      [ 'right wheel',
                              'left wheel',
                              'any wheel'],
            detectSensors:  [ 'wall',
                              'wall left',
                              'wall front',
                              'wall right',
                              'cliff'
                            ],
            booleanSensor:  [
                              /* Disable this for now. Wait for full featured version.
                              'charger plugged in',
                              'robot on dock',
                              'robot control allowed'
                              'virtual wall',
                              */
                              /*
                              'side-brush stall detected',
                              'main-brush stall detected',
                              'right wheel stall detected',
                              'left wheel stall detected',
                              'stall detected',
                              'charging error',
                              'recovery charging',
                              'full charging',
                              'trickle charging',
                              'not charging',
                              'charging',
                              'song playing',
                              'left light bumper activated',
                              'front-left light bumper activated',
                              'center-left light bumper activated',
                              'center-right light bumper activated',
                              'front-right light bumper activated',
                              'right light bumper activated',
                              'light bumper activated',
                              */ ],
            // Analog sensor values
            sensor:         [ /*
                              'ir-opcode',
                              'voltage',
                              'velocity',
                              'radius',
                              'distance',
                              'angle',
                              'current',
                              'temperature',
                              'battery-charge',
                              'battery-capacity',
                              'wall-signal',
                              'cliff-left-signal',
                              'cliff-front-left-signal',
                              'cliff-front-right-signal',
                              'cliff-right-signal',
                              'open-interface-mode',
                              'song-number',
                              'oi-stream-num-packets',
                              'velocity-right',
                              'velocity-left',
                              */
                              'encoder-counts-left',
                              'encoder-counts-right',
                              /*
                              'light-bumper',
                              'light-bump-left',
                              'light-bump-front-left',
                              'light-bump-center-left',
                              'light-bump-center-right',
                              'light-bump-front-right',
                              'light-bump-right',
                              'ir-opcode-left',
                              'ir-opcode-right',
                              'left-motor-current',
                              'right-motor-current',
                              'main-brush-current',
                              'side-brush-current',
                              'stasis'
                              */ ],
            dock:           ['go to',
                             'leave'
                            ],
            lessMore:       [ '>', '<', '=' ],
        },
        url: 'http://www.irobot.com/About-iRobot/STEM/Create-2/Projects.aspx'
    };
    ScratchExtensions.register('iRobot Create 2', descriptor, ext, {type: 'serial'});
})({});
