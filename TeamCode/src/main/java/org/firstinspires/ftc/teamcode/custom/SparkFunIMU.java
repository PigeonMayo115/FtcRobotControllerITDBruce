package org.firstinspires.ftc.teamcode.custom;

/*  I want to try using roadrunner, but the expansion hub doesn't
    include an IMU.  The SparkFun Odometery Module has one, but it
    doesn't implement the same interface as the "standard" ControlHub
    IMUs.  Here's my attempt at convincing roadrunner to use the
    one on the Odometry Module.

    - JM 10/8/2024

    In MecanumDrive.java, see line 235... lazyImu is instance of LazyImu
    In LazyImu.kt, line 30, uses hardwaremap.get(IMU.class, name).
    I think I need to overload this somehow.

    JM 10/26/24 - Updated to use ByteBuffer
    
*/

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class SparkFunIMU implements IMU {

    private final double LSM6DSO_RAW_TO_DPS = 0.07;     // conversion factor from SF firmware
    private final byte LSM6DSO_OUTX_L_G = 0x44;         // starting register for raw gyro data
    private SparkFunOTOS myOTOS;
    private RevHubOrientationOnRobot imuOrientation;

    private long previousSampleTime;                    // For integratinng velocity measures
    private long sampleTime;                            // nanotime when current sample was taken

    private double yaw;
    private double pitch;
    private double roll;
    private double yawV;
    private double pitchV;
    private double rollV;
    public short gyrOffsetP;
    public short gyrOffsetY;
    public short gyrOffsetR;
    public short pitch_16;
    public short roll_16;
    public short yaw_16;

    // constructor!
    // Really, it's an I2C
    public SparkFunIMU(HardwareMap hardwareMap, String imu, RevHubOrientationOnRobot revHubOrientationOnRobot) {
        myOTOS = hardwareMap.get(SparkFunOTOS.class, imu);
        imuOrientation = revHubOrientationOnRobot;
    }

    public IMU get() {
        return this;
    }

    @Override
    public boolean initialize(Parameters parameters) {
        yaw = 0.0;
        pitch = 0.0;
        roll = 0.0;
        determineOffsets(100);   // Run our own "calibrate" routine to eliminate drift
        updateData();                       // Get the first round of data for subsequent calcs
        return myOTOS.calibrateImu();       // This causes the cal routine to run on the OTOS uC
    }

    // Run same cal routine as firmware on the OTOS to determine offset
    private void determineOffsets(int numSamples) {
        int i;
        // Variables to store the running sum of raw IMU data, which we'll use to
        // compute the average
        int gyrSumX;
        int gyrSumY;
        int gyrSumZ;
        gyrSumX = 0;
        gyrSumY = 0;
        gyrSumZ = 0;
        ByteBuffer byteBuffer;

        // Loop until we get the requested number of samples
        long timeoutStart = System.currentTimeMillis();
        for (i=0; i<numSamples; i++){
            // Check if we've waited too long
            if((System.currentTimeMillis() - timeoutStart) > 5000){
                RobotLog.e("Timeout while attempting IMU Cal");
                break;
            }
            byteBuffer = getGyroRaw();
            // We will define pitch as rotation around the x axis, roll as rotation around y
            // and yaw (the important one for us) as rotation around z.
            gyrSumX += byteBuffer.getShort();
            gyrSumY += byteBuffer.getShort();
            gyrSumZ += byteBuffer.getShort();
        }

        gyrOffsetP = (short) (gyrSumX / numSamples);
        gyrOffsetR = (short) (gyrSumY / numSamples);
        gyrOffsetY = (short) (gyrSumZ / numSamples);
    }

    // Gets a round of raw Gyro data from the SparkFun module and returns it in a ByteBuffer
    private ByteBuffer getGyroRaw(){
        ByteBuffer byteBuffer;
        byte[] read_data = myOTOS.getDeviceClient().read(LSM6DSO_OUTX_L_G, 6);
        byteBuffer = ByteBuffer.wrap(read_data);
        byteBuffer.order(ByteOrder.LITTLE_ENDIAN);      // Default has byte order backwards vs sensor
        previousSampleTime = sampleTime;
        sampleTime = System.nanoTime();
        return byteBuffer;
    }

    // Trigger a read to update the values in the class fields, converting to engineering
    // Units and applying the offsets that we calculated in .determineOffsets()
    // These data can be used by the other methods to return the orientation info in whatever
    // format is required.
    private void updateData(){
        ByteBuffer byteBuffer = getGyroRaw();
        pitch_16 = (short) (byteBuffer.getShort() - gyrOffsetP);             // pitch rate raw in dps
        roll_16 = (short) (byteBuffer.getShort() - gyrOffsetR);              // roll rate raw in dps
        yaw_16 = (short) (byteBuffer.getShort() - gyrOffsetY);               // yaw rate raw in dps
        pitchV = pitch_16 * LSM6DSO_RAW_TO_DPS;
        rollV = roll_16 * LSM6DSO_RAW_TO_DPS;
        yawV = yaw_16 * LSM6DSO_RAW_TO_DPS;
    }

    @Override
    public void resetYaw() {
        yaw = 0.0;
    }

    // Important: Used in MecanumDrive.java
    // LSM6DSO gives yaw, pitch, roll rates
    // 0x44 X_L 0x45 X_H  << 8 bit registers (16 bit word in 2's compliment)
    // 0x46 Y_L 0x47 Y_H
    // 0x48 Z_L 0x49 z_H
    // x is pitch, y is roll, z is yaw
    // Units depend on CTRL2_G (0x11) configuration register (degrees per second)
    // 2000 dps, 16g, 416hZ (from firmware)
    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {

        long elapsedTime;
        double elapsedTimeS;
        updateData();

        // last val + (rate x time)
        elapsedTime = sampleTime - previousSampleTime;
        elapsedTimeS = (double)elapsedTime / 1e9;            // in seconds
        yaw += yawV * elapsedTimeS;
        pitch += pitchV * elapsedTimeS;
        roll += rollV * elapsedTimeS;

        // TODO: pitch and roll should really use the accelerometer data
        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, sampleTime);
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        // TODO: implement this
        return null;
    }

    @Override
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
    public Quaternion getRobotOrientationAsQuaternion() {

        updateData();

        double heading = Math.toRadians(yaw);
        double attitude = Math.toRadians(pitch);
        double bank = Math.toRadians(roll);

        // Assuming the angles are in radians.
        double c1 = Math.cos(heading/2);
        double s1 = Math.sin(heading/2);
        double c2 = Math.cos(attitude/2);
        double s2 = Math.sin(attitude/2);
        double c3 = Math.cos(bank/2);
        double s3 = Math.sin(bank/2);
        double c1c2 = c1*c2;
        double s1s2 = s1*s2;
        float w = (float) (c1c2*c3 - s1s2*s3);
        float x = (float) (c1c2*s3 + s1s2*c3);
        float y = (float) (s1*c2*c3 + c1*s2*s3);
        float z = (float) (c1*s2*c3 - s1*c2*s3);

        return new Quaternion(w, x, y, z, sampleTime);
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        updateData();
        return new AngularVelocity(
                AngleUnit.DEGREES, (float) rollV, (float) pitchV, (float) yawV, sampleTime);
    }

    @Override
    public Manufacturer getManufacturer() {
        // TODO: implement this
        return null;
    }

    @Override
    public String getDeviceName() {
        return "SparkFun OTOS IMU";
    }

    @Override
    public String getConnectionInfo() {
        // TODO: implement this
        return "";
    }

    @Override
    public int getVersion() {
        // TODO; implement this
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}


