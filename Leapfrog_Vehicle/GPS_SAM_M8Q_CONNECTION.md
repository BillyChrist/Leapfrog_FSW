# SparkFun GPS u-blox SAM-M8Q Connection Guide

## Hardware Connections

### STM32F4 to SAM-M8Q Wiring
```
STM32F4 Pin    →    SAM-M8Q Pin    →    Function
PA9 (USART1_TX) →   RX              →    GPS Data In
PA10 (USART1_RX) →  TX              →    GPS Data Out
3.3V           →    VCC             →    Power Supply
GND            →    GND             →    Ground
```

### Power Requirements
- **Voltage**: 3.3V (compatible with STM32F4)
- **Current**: ~20mA during operation
- **Power**: Can be powered from STM32 3.3V rail

## Software Configuration

### Current Settings
- **Baud Rate**: 9600 (SAM-M8Q default)
- **Data Format**: NMEA 0183
- **Update Rate**: 1 Hz (default)
- **Messages**: GPGGA, GPRMC, GPVTG

### NMEA Messages Parsed
1. **GPGGA** - Global Positioning System Fix Data
   - Latitude, Longitude, Altitude
   - Fix status, Number of satellites
   - HDOP (Horizontal Dilution of Precision)

2. **GPRMC** - Recommended Minimum Specific GPS/Transit Data
   - Position, Speed, Course
   - Status (Valid/Invalid)
   - Date and Time

3. **GPVTG** - Track Made Good and Ground Speed
   - True track made good
   - Speed over ground
   - Course over ground

## Debug Output

The system will display GPS data in the debug output:
```
GPS - Lat: 40.123456, Lon: -74.654321, Alt: 123.45, Fix: 1
GPS - Speed: 2.34 m/s, Heading: 45.6°, Valid: 1, Age: 150ms
```

Where:
- **Lat/Lon**: Position in decimal degrees
- **Alt**: Altitude in meters
- **Fix**: 0=No fix, 1=GPS fix, 2=DGPS fix
- **Speed**: Ground speed in m/s
- **Heading**: Course over ground in degrees
- **Valid**: 1=Valid data, 0=Invalid data
- **Age**: Data age in milliseconds

## Performance Optimization

### Potential Improvements
1. **Increase Update Rate**: Can be configured up to 10 Hz
2. **UBX Configuration**: Send UBX commands for advanced configuration
3. **Power Management**: Configure power modes for battery operation
4. **Message Filtering**: Enable only needed NMEA messages

### Current Limitations
- Using default 1 Hz update rate
- No UBX configuration commands implemented
- Basic NMEA parsing only

## Troubleshooting

### Common Issues
1. **No GPS Data**: Check wiring and power
2. **Invalid Coordinates**: Wait for GPS fix (can take 30+ seconds)
3. **Poor Accuracy**: Ensure clear sky view
4. **Data Corruption**: Check baud rate and connections

### Debug Steps
1. Monitor UART1 output for NMEA messages
2. Check GPS data validity flags
3. Verify antenna connection
4. Test with known good GPS coordinates

## Integration Notes

The GPS system is integrated with:
- **TVC System**: Position error for navigation
- **Altimeter**: Altitude cross-checking
- **Safety Systems**: Position monitoring for safe landing
- **Debug Output**: Real-time GPS status display

## Future Enhancements

1. **UBX Protocol**: Implement u-blox binary protocol
2. **RTK Support**: Add Real-Time Kinematic positioning
3. **Multi-Constellation**: Support GLONASS, Galileo, BeiDou
4. **Advanced Filtering**: Kalman filtering for position smoothing
