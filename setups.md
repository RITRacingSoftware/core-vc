# Endurance
 - Lat split `-0.00058f*vel*vel + 0.75f`
 - Endurance mode enabled
   - DISTANCE=22500, CHARGE=11000, TEMP=55
 - TC disabled (`BASIC_VEL` controls)
 - DRS enabled 
   - ACCEL=0.5, BRAKE=0.1, STEER=0.1, DELAY=25 (250ms)

# Autocross
 - Max torque 214 %Mn
 - Lat func: `-0.00058f*vel*vel + 0.70f`
 - Controls level `BASIC_VEL`

# Skidpad
 - Controls level `SKIDPAD`

# Accel
 - TC on (`ADVANCED` controls)
   - TMAX = 60, TBLEND1 = 0.05, TBLEND2 = 0.2
   - KP = 0.0025, KI = 0.00005, KD = 0.00125
   - Rear slip 0.2, front slip 0.17
 - DRS enabled, accel mode
   - VELOCITY = 18, DELAY = 5 (50ms)
 - First/second runs:
   - 65Nm ramp, 0.22 rear, 0.19 front
 - Third run:
   - 67Nm ramp, 0.4 rear and front
 - Fourth run:
   - 67Nm ramp, no TC
