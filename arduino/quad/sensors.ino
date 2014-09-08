void initializeAccelGyro()
{
  Serial.println(F("Initializing Accelerometer and Gyroscope..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing MPU6050 DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
    {
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling MPU6050 DMP..."));
      mpu.setDMPEnabled(true);

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
  else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
}

void initializeMagnetometer()
{
  Serial.println(F("Initializing Magnetometer..."));
  mpu.setI2CBypassEnabled(true);
  mag.initialize();

  Serial.println("Testing HMC5883L connection...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void initializeBarometer()
{
  Serial.println(F("Initializing Barometer..."));
  barometer.initialize();
  Serial.println("Testing BMP085 connection...");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
}

void updateAccelGyroSensor(unsigned long dt)
{
  if(dmpReady)
    {
      if(fifoCount > packetSize)
        {
          mpuIntStatus = mpu.getIntStatus();
          
          // get current FIFO count
          fifoCount = mpu.getFIFOCount();

          if ((mpuIntStatus & 0x10) || fifoCount == 1024)
            {
              // reset so we can continue cleanly
              mpu.resetFIFO();
              //Serial.println(F("FIFO overflow!"));
            }
          else if (mpuIntStatus & 0x02)
            {
              // wait for correct available data length, should be a VERY short wait
              while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
              
              // read a packet from FIFO
              mpu.getFIFOBytes(fifoBuffer, packetSize);
        
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= packetSize;

              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              
              // display real acceleration, adjusted to remove gravity
              mpu.dmpGetAccel(&aa, fifoBuffer);
              mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
              
              // display initial world-frame acceleration, adjusted to remove gravity
              // and rotated based on known orientation from quaternion
              mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            }
        }
    }
}

void updateMagSensor(unsigned long dt)
{
  mag.getHeading(&mx, &my, &mz);
}

void updateBarSensor(unsigned long dt)
{
  barometer.setControl(BMP085_MODE_TEMPERATURE);
  temperature = barometer.getTemperatureC();

  barometer.setControl(BMP085_MODE_PRESSURE_3);
  if(micros() - lastBarReadingMicros > barometer.getMeasureDelayMicroseconds())
    {
      lastBarReadingMicros = micros();
      pressure = barometer.getPressure();
      altitude = barometer.getAltitude(pressure);
    }
}
