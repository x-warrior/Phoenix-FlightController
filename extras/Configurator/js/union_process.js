/*  Code below contains an exact representation of eeprom configuration array
    that is stored inside the flight controller.
    
    Rest of the code extends the DataView low-level interface, that is now included 
    by default inside the javascript engine (chrome v9, firefox v15, IE v10).
    
    You can read more about the DataView interface and its methods
    @ https://developer.mozilla.org/en-US/docs/JavaScript/Typed_arrays/DataView
*/

var eepromConfig = {
    version:      0,
    calibrateESC: 0,
    minimumArmedThrottle: 0,
    
    ACCEL_BIAS:  [0, 0, 0],
    
    CHANNEL_ASSIGNMENT: [0, 0, 0, 0, 0, 0, 0, 0],
    
    PID_YAW_c:   [0.0, 0.0, 0.0, 0],
    PID_PITCH_c: [0.0, 0.0, 0.0, 0],
    PID_ROLL_c:  [0.0, 0.0, 0.0, 0],
    
    PID_YAW_m:   [0.0, 0.0, 0.0, 0],
    PID_PITCH_m: [0.0, 0.0, 0.0, 0],
    PID_ROLL_m:  [0.0, 0.0, 0.0, 0],
    
    PID_BARO:    [0.0, 0.0, 0.0, 0],
    PID_SONAR:   [0.0, 0.0, 0.0, 0],
    PID_GPS:     [0.0, 0.0, 0.0, 0],
};

DataView.prototype.getString = function(str_length, byteOffset) {
    var value = '';
    for (var i = 0; i < str_length; ++i) {
        var char = this.getUint8(byteOffset + i);
        value += String.fromCharCode(char > 127 ? 65533 : char);
    }

    return value;
}

DataView.prototype.getChar = function(byteOffset) {
    return this.getString(1, byteOffset);
}

DataView.prototype.parseUNION = function (structure) {
    var needle = 0;
    
    structure.version = this.getUint8(needle++);
    structure.calibrateESC = this.getUint8(needle++);
    structure.minimumArmedThrottle = this.getUint16(needle, 1);
    needle += 2;
    
    for (var i = 0; i < structure.ACCEL_BIAS.length; i++) {
        structure.ACCEL_BIAS[i] = this.getInt16(needle, 1);
        needle += 2;
    }
    
    for (var i = 0; i < structure.CHANNEL_ASSIGNMENT.length; i++) {
        structure.CHANNEL_ASSIGNMENT[i] = this.getUint8(needle, 1);
        needle += 1;
    }    
    
    for (var i = 0; i < 4; i++) {
        structure.PID_YAW_c[i]   = this.getFloat32(needle,       1);
        structure.PID_PITCH_c[i] = this.getFloat32(needle + 16,  1);
        structure.PID_ROLL_c[i]  = this.getFloat32(needle + 32,  1);

        structure.PID_YAW_m[i]   = this.getFloat32(needle + 48,  1);
        structure.PID_PITCH_m[i] = this.getFloat32(needle + 64,  1);
        structure.PID_ROLL_m[i]  = this.getFloat32(needle + 80,  1);

        structure.PID_BARO[i]    = this.getFloat32(needle + 96,  1);
        structure.PID_SONAR[i]   = this.getFloat32(needle + 112, 1);
        structure.PID_GPS[i]     = this.getFloat32(needle + 128, 1);
        
        needle += 4;
    }   
}

DataView.prototype.setUNION = function (structure) {
    var needle = 0;
    
    this.setUint8(needle++, structure.version);
    this.setUint8(needle++, structure.calibrateESC);
    this.setUint16(needle, structure.minimumArmedThrottle, 1);
    needle += 2;
    
    for (var i = 0; i < structure.ACCEL_BIAS.length; i++) {
        this.setInt16(needle, structure.ACCEL_BIAS[i], 1);
        needle += 2;
    }

    for (var i = 0; i < structure.CHANNEL_ASSIGNMENT.length; i++) {
        this.setUint8(needle, structure.CHANNEL_ASSIGNMENT[i], 1);
        needle += 1;
    }  
    
    for (var i = 0; i < 4; i++) {
        this.setFloat32(needle, structure.PID_YAW_c[i],        1);
        this.setFloat32(needle + 16, structure.PID_PITCH_c[i], 1);
        this.setFloat32(needle + 32, structure.PID_ROLL_c[i],  1);

        this.setFloat32(needle + 48, structure.PID_YAW_m[i],   1);
        this.setFloat32(needle + 64, structure.PID_PITCH_m[i], 1);
        this.setFloat32(needle + 80, structure.PID_ROLL_m[i],  1);

        this.setFloat32(needle + 96, structure.PID_BARO[i],    1);
        this.setFloat32(needle + 112, structure.PID_SONAR[i],  1);
        this.setFloat32(needle + 128, structure.PID_GPS[i],    1);
        
        needle += 4;
    }   
}

function requestUNION() {
    var bufferOut = new ArrayBuffer(6);
    var bufView = new Uint8Array(bufferOut);        

    // sync char 1, sync char 2, command, payload length MSB, payload length LSB, payload
    bufView[0] = 0xB5; // sync char 1
    bufView[1] = 0x62; // sync char 2
    bufView[2] = 0x01; // command
    bufView[3] = 0x00; // payload length MSB
    bufView[4] = 0x01; // payload length LSB
    bufView[5] = 0x01; // payload
   
    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {
        console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
        command_log('Requesting configuration UNION from Flight Controller');
    }); 
}

function sendUNION() {
    var eepromConfigBytes = new ArrayBuffer(eepromConfigSize);
    var view = new DataView(eepromConfigBytes, 0);
    view.setUNION(eepromConfig); 

    var bufferOut = new ArrayBuffer(5);
    var bufView = new Uint8Array(bufferOut);
    
    // sync char 1, sync char 2, command, payload length MSB, payload length LSB, payload
    bufView[0] = 0xB5; // sync char 1
    bufView[1] = 0x62; // sync char 2
    bufView[2] = 0x02; // command
    bufView[3] = highByte(eepromConfigSize); // payload length MSB
    bufView[4] = lowByte(eepromConfigSize); // payload length LSB    
    
    chrome.serial.write(connectionId, bufferOut, function(writeInfo) {});
    
    // payload
    chrome.serial.write(connectionId, eepromConfigBytes, function(writeInfo) {
        if (writeInfo.bytesWritten > 0) {
            console.log("Wrote: " + writeInfo.bytesWritten + " bytes");
            
            command_log('Sending Configuration UNION to Flight Controller ...');
        }    
    });
}