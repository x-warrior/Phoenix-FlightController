/*
    Kinematics implementation using first order complementary filter by cTn
    
    Extra:
    1. accelerometer data normalization
    2. accelerometer data cut-off
    3. kinematics-gyroscope angle overflow handling

    Kinematics input is averaged (from multiple samples) and scaled (gyro is in radians/s) and accel is in m/s^2
    accel measurement is normalized before any angles are computed.
*/

unsigned long kinematics_timer;

void kinematics_update(double* accelX, double* accelY, double* accelZ, double* gyroX, double* gyroY, double* gyroZ) {

    // Normalize accel values
    double norm = sqrt(*accelX * *accelX + *accelY * *accelY + *accelZ * *accelZ);
    *accelX /= norm;
    *accelY /= norm;
    *accelZ /= norm;
    
    // Determinate Sensor orientation
    bool orientation = true; // up-side UP
    if (*accelZ < 0.00) orientation = false; // up-side DOWN    
    
    double accelXangle = atan2(*accelY, *accelZ);
    //double accelYangle = atan2(*accelX, *accelZ); 
    double accelYangle = atan(*accelX /sqrt(*accelY * *accelY + *accelZ * *accelZ));   
    
    // Accelerometer cut-off
    double accelWeight = 0.0025; // normal operation
    if (norm > 13.0 || norm < 7.0) accelWeight = 0.00; // gyro only
    
    // Save current time into variable for better computation time
    unsigned long now = micros();    
    
    // Fuse in gyroscope
    kinematicsAngleX = kinematicsAngleX + (*gyroX * (double)(now - kinematics_timer) / 1000000);
    kinematicsAngleY = kinematicsAngleY + (((kinematicsAngleX > HALF_PI || kinematicsAngleX < -HALF_PI) ? -1.0 : 1.0) * (*gyroY * (double)(now - kinematics_timer) / 1000000));
    kinematicsAngleZ = kinematicsAngleZ + (*gyroZ * (double)(now - kinematics_timer) / 1000000);  
    
    // Normalize gyro kinematics (+ - PI)    
    if (kinematicsAngleY > PI) {
        kinematicsAngleY = TWO_PI - kinematicsAngleY;
        kinematicsAngleX += PI;
        kinematicsAngleZ += PI;
        
        commandYawAttitude += PI;
    } else if (kinematicsAngleY < -PI) {
        kinematicsAngleY = -TWO_PI - kinematicsAngleY;
        kinematicsAngleX += PI;
        kinematicsAngleZ += PI;
        
        commandYawAttitude += PI;
    }    
    
    if (kinematicsAngleX > PI) kinematicsAngleX -= TWO_PI;
    else if (kinematicsAngleX < -PI) kinematicsAngleX += TWO_PI;        
    
    // While normalizing Z angle, attitudeYaw (angle desired by user) is also normalized
    // attitudeYaw variable was added to handle clean normalization of angles while still
    // allowing for a smooth rate/attitude mode switching.
    if (kinematicsAngleZ > PI) {
        kinematicsAngleZ -= TWO_PI;
        commandYawAttitude -= TWO_PI;
    } else if (kinematicsAngleZ < -PI) {
        kinematicsAngleZ += TWO_PI; 
        commandYawAttitude += TWO_PI;
    }
    
    // Fuse in accel (handling accel flip)
    // This is second order accelerometer cut off, which restricts accel data fusion in only
    // "up-side UP" angle estimation and restricts it further to avoid incorrect accelerometer
    // data correction.
    if ((kinematicsAngleX - accelXangle) > PI) {
        kinematicsAngleX = (1.00 - accelWeight) * kinematicsAngleX + accelWeight * (accelXangle + TWO_PI);
    } else if ((kinematicsAngleX - accelXangle) < -PI) {
        kinematicsAngleX = (1.00 - accelWeight) * kinematicsAngleX + accelWeight * (accelXangle - TWO_PI);
    } else {
        kinematicsAngleX = (1.00 - accelWeight) * kinematicsAngleX + accelWeight * accelXangle;
    }
    
    if ((kinematicsAngleY - accelYangle) > HALF_PI) {
        kinematicsAngleY = (1.00 - accelWeight) * kinematicsAngleY + accelWeight * (accelYangle + PI);
    } else if ((kinematicsAngleY - accelYangle) < -HALF_PI) {
        kinematicsAngleY = (1.00 - accelWeight) * kinematicsAngleY + accelWeight * (accelYangle - PI);
    } else {
        kinematicsAngleY = (1.00 - accelWeight) * kinematicsAngleY + accelWeight * accelYangle;
    } 
    
    // Saves time for next comparison
    kinematics_timer = now;
}