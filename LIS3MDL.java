package frc.robot.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class LIS3MDL implements Sendable {

    private final static int DEFAULT_I2C_DEVICE_ADDRESS = 0x1C;
    private final static int WHOAMI = 0xF;
    private final static byte CHIP_ID = 0x3D;

    // Addresses for the LIS3MDL
    private final static int WHO_AM_I = 0x0F;
    private final static int CTRL_REG1 = 0x20;
    private final static int CTRL_REG2 = 0x21;
    private final static int CTRL_REG3 = 0x22;
    private final static int CTRL_REG4 = 0x23;
    private final static int SENSOR_DATA = 0x28;
    private final static int TEMP_DATA = 0x2E;
    private final static int INT_CFG = 0x30;
    private final static int INT_THS_L = 0x32;

    private int LSB_Multiplier = 1711;
    private final static int GAUSS_TO_T = 100;
    private LIS3MDL_Result last_result = null;

    protected final static int CMD = 0x80;
    private I2C sensor;
    private volatile ByteBuffer readMagBuffer = ByteBuffer.allocate(6);
    private volatile ByteBuffer readTempBuffer = ByteBuffer.allocate(2);

    private volatile int bad_reads = 0;

    public LIS3MDL(I2C.Port port) {
        this(port, DEFAULT_I2C_DEVICE_ADDRESS);
    }

    public LIS3MDL(I2C.Port port, int address) {
        sensor = new I2C(port, address);
        readMagBuffer.order(ByteOrder.LITTLE_ENDIAN);
        readTempBuffer.order(ByteOrder.LITTLE_ENDIAN);

        byte[] sensorId = ByteBuffer.allocate(1).put(CHIP_ID).array();
        if(!sensor.verifySensor(WHO_AM_I, 1, sensorId)) {
            System.err.println("LIS3MDL - wrong Chip ID");
            bad_reads++;
        }

        setRate(LIS3MDL_Rate.RATE_155_HZ);
        setTemperatureEnabled(true);
        setRange(LIS3MDL_Range.RANGE_16_GAUSS);
        setPerformanceMode(LIS3MDL_PerformanceMode.ULTRA_PERFORMANCE);
        setOperationMode(LIS3MDL_OperationMode.CONTINUOUS);
    }

    public synchronized LIS3MDL_Result read() {
        readMagBuffer.clear();
        if(sensor.read(SENSOR_DATA, 6, readMagBuffer)) {
            System.out.println("Failed to get data");
            bad_reads++;
        }
        if(sensor.read(TEMP_DATA, 2, readTempBuffer)) {
            System.out.println("Failed to get data");
            bad_reads++;
        }

        short tempX = readMagBuffer.getShort(0);
        short tempY = readMagBuffer.getShort(2);
        short tempZ = readMagBuffer.getShort(4);
        short tempT = readTempBuffer.getShort(0);

        LIS3MDL_Result result = new LIS3MDL_Result(
            scale_mag(tempX), // x
            scale_mag(tempY), // y
            scale_mag(tempZ), // z
            scale_temp(tempT) // temperature
        );
        
        return last_result = result;
    }

    public int getBadReads() {
        return this.bad_reads;
    }

    private double scale_mag(short bits_in) {
        return (double) bits_in / this.LSB_Multiplier * GAUSS_TO_T;
    }

    private double scale_temp(short bits_in) {
        double temp_raw = (double)bits_in;
        return temp_raw / 256.0 + 25.0; // 8 LSB per degree C
    }

    private int getControlRegister(int register) {
        ByteBuffer buffer = ByteBuffer.allocate(2);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        this.sensor.read(register, 1, buffer);
        return buffer.getShort();
    }

    private void setRate(LIS3MDL_Rate rate) {
        
        int buffer = this.getControlRegister(CTRL_REG1);

        int output = 0b1110_0001;
        switch(rate) {
            case RATE_0_625_HZ:     output = 0b0000_0000; break;
            case RATE_1_25_HZ:      output = 0b0000_0100; break;
            case RATE_2_5_HZ:       output = 0b0000_1000; break;
            case RATE_5_HZ:         output = 0b0000_1100; break;
            case RATE_10_HZ:        output = 0b0001_0000; break;
            case RATE_20_HZ:        output = 0b0001_0100; break;
            case RATE_40_HZ:        output = 0b0001_1000; break;
            case RATE_80_HZ:        output = 0b0001_1100; break;
            case RATE_155_HZ:       output = 0b0000_0010; break;
            case RATE_300_HZ:       output = 0b0000_0110; break;
            case RATE_560_HZ:       output = 0b0000_1010; break;
            case RATE_1000_HZ:      output = 0b0000_1110; break;
        }

        buffer &= 0b1110_0001;
        buffer |= output;
        this.sensor.write(CTRL_REG1, buffer);
    }

    public void setTemperatureEnabled(boolean enabled) {
        int buffer = this.getControlRegister(CTRL_REG1);
        if(enabled) {
            buffer |= 0b1000_0000;
        } else {
            buffer &= 0b0111_1111;
        }
        
        this.sensor.write(CTRL_REG1, buffer);
    }

    public void setRange(LIS3MDL_Range range) {
        int output = 0b0000_0000;
        switch(range) {
            case RANGE_4_GAUSS:     output = 0b0000_0000; LSB_Multiplier = 6842; break;
            case RANGE_8_GAUSS:     output = 0b0010_0000; LSB_Multiplier = 3421; break;
            case RANGE_12_GAUSS:    output = 0b0100_0000; LSB_Multiplier = 2281; break;
            case RANGE_16_GAUSS:    output = 0b0110_0000; LSB_Multiplier = 1711; break;
        }

        this.sensor.write(CTRL_REG2, output);
    }

    public void setPerformanceMode(LIS3MDL_PerformanceMode mode) {
        // Z Performance Mode
        int output = 0b0000_0000;
        switch(mode) {
            case LOW_POWER:             output = 0b0000_0000; break;
            case MEDIUM_PERFORMANCE:    output = 0b0000_0100; break;
            case HIGH_PERFORMANCE:      output = 0b0000_1000; break;
            case ULTRA_PERFORMANCE:     output = 0b0000_1100; break;
        }
        this.sensor.write(CTRL_REG4, output);

        // X Y Performance Mode
        int buffer = this.getControlRegister(CTRL_REG1);
        switch(mode) {
            case LOW_POWER:             output = 0b0000_0000; break;
            case MEDIUM_PERFORMANCE:    output = 0b0010_0000; break;
            case HIGH_PERFORMANCE:      output = 0b0100_0000; break;
            case ULTRA_PERFORMANCE:     output = 0b0110_0000; break;
        }
        buffer &= 0b1001_1111;
        buffer |= output;
        sensor.write(CTRL_REG1, buffer);
    }

    public void setOperationMode(LIS3MDL_OperationMode mode) {
        int output = 0b0000_0000;
        switch(mode) {
            case CONTINUOUS:    output = 0b0000_0000; break;
            case SINGLE:        output = 0b0000_0001; break;
            case POWERDOWN:     output = 0b0000_0010; break;
        }
        this.sensor.write(CTRL_REG3, output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("LIS3MDL");
    }
    
    public enum LIS3MDL_Range {
        RANGE_4_GAUSS,
        RANGE_8_GAUSS,
        RANGE_12_GAUSS,
        RANGE_16_GAUSS
    }

    public enum LIS3MDL_PerformanceMode {
        LOW_POWER,
        MEDIUM_PERFORMANCE,
        HIGH_PERFORMANCE,
        ULTRA_PERFORMANCE
    }

    public enum LIS3MDL_Rate {
        RATE_0_625_HZ,
        RATE_1_25_HZ,
        RATE_2_5_HZ,
        RATE_5_HZ,
        RATE_10_HZ,
        RATE_20_HZ,
        RATE_40_HZ,
        RATE_80_HZ,
        RATE_155_HZ,
        RATE_300_HZ,
        RATE_560_HZ,
        RATE_1000_HZ
    }

    public enum LIS3MDL_OperationMode {
        CONTINUOUS,
        SINGLE,
        POWERDOWN
    }

    public static class LIS3MDL_Result {
        public double x = -1, y = -1, z = -1, tempF = -1, tempC = -1;

        public LIS3MDL_Result(double x, double y, double z, double temperature) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.tempC = temperature;
            this.tempF = (this.tempC * 9.0 / 5.0) + 32.0;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getZ() {
            return z;
        }

        public double getTempF() {
            return tempF;
        }

        public double getTempC() {
            return tempC;
        }
    }
}


