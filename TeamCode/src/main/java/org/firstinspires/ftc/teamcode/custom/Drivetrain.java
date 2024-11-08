package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Enum used to track which robot we're running on now
enum Robot{
    BOGG, ELIOT, MERRICK
}

public class Drivetrain {
    // TODO: Select the robot programmatically.
    private Robot whichRobot = Robot.ELIOT;   // Change this to specify the robot we're using
    public DcMotor flMot;
    public DcMotor blMot;
    public DcMotor frMot;
    public DcMotor brMot;
    public IMU imu;
    public double targetHeading;
    public int targetDistance;
    int encoderResolution;
    double ticksPerInch;

    // Constructor
    public Drivetrain(HardwareMap hwMap, int robotConfig) {

        flMot = hwMap.dcMotor.get("frontLeftMotor");
        blMot = hwMap.dcMotor.get("backLeftMotor");
        frMot = hwMap.dcMotor.get("frontRightMotor");
        brMot = hwMap.dcMotor.get("backRightMotor");

        frMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // TODO: Change the constructor to take the enum as the argument, update all instances.
        switch (robotConfig) {       // 0 = bogg, 1 = home, 2 = eliot
            case 0:
                whichRobot = Robot.BOGG;
                break;
            case 1:
                whichRobot = Robot.MERRICK;
                break;
            case 2:
                whichRobot = Robot.ELIOT;
                break;
        }

        switch (whichRobot){
            case BOGG:
                flMot.setDirection(DcMotorSimple.Direction.FORWARD);
                blMot.setDirection(DcMotorSimple.Direction.FORWARD);
                frMot.setDirection(DcMotorSimple.Direction.REVERSE);
                brMot.setDirection(DcMotorSimple.Direction.REVERSE);
                encoderResolution= 550; // for bogg (yellowJacket) the resolution is 550
                break;
            case MERRICK:
                flMot.setDirection(DcMotorSimple.Direction.FORWARD);
                blMot.setDirection(DcMotorSimple.Direction.REVERSE);
                frMot.setDirection(DcMotorSimple.Direction.FORWARD);
                brMot.setDirection(DcMotorSimple.Direction.REVERSE);
                encoderResolution= 550;
                break;
            case ELIOT:
                flMot.setDirection(DcMotorSimple.Direction.REVERSE);
                blMot.setDirection(DcMotorSimple.Direction.FORWARD);
                frMot.setDirection(DcMotorSimple.Direction.FORWARD);
                brMot.setDirection(DcMotorSimple.Direction.FORWARD);
                encoderResolution= 440; // for the rev motors, the resolution is 440
                break;
        }
        ticksPerInch = encoderResolution/(4.1*Math.PI);
        frMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        blMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        flMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        brMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
                //this is correct for eliot TODO: make a case for bogg, home robot does not have a built in imu
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void driveLeft(double spdMult) {
        setMotPow(1, 1, 1, 1, spdMult);
    }

    public void driveRight(double spdMult) {
        setMotPow(-1, 1, -1, 1, spdMult);
    }

    public void driveForward(double spdMult) {
        setMotPow(1, -1, 1, -1, spdMult);
    }

    public void driveReverse(double spdMult) {
        setMotPow(-1, 1, -1, -1, spdMult);
    }

    public double getHeading(AngleUnit angleUnit){
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public void setTargetHeading(double degrees){
        double startHeading = getHeading(AngleUnit.DEGREES);
        double overShootAdjuster = 11.0;        // seems to overshoot by 11 degrees
        targetHeading = startHeading + degrees; //if result is >180 degrees, fix it!

        if (degrees < 0) { targetHeading = (targetHeading + overShootAdjuster);}
        if (degrees > 0){ targetHeading = (targetHeading - overShootAdjuster);}

        if(targetHeading>180){
            targetHeading = targetHeading-360;
        } else if (targetHeading<-180){
            targetHeading = targetHeading+ 360;
        }
    }

    //turn the robot to the LEFT if positive, right if negative
    //returns true if move is complete
    public boolean turnToHeading(double degrees){

        setMotRUE();        // Run Using Encoder

        //now we are going to do the turn
        if(degrees>0){
            //we are turning left
            if(getHeading(AngleUnit.DEGREES)<targetHeading){
                setMotPow(-0.3,-0.3,0.3,0.3,1);
                return false;
            } else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        }
        if (degrees<0){
            //we are turning right
            if(getHeading(AngleUnit.DEGREES)>targetHeading){
                setMotPow(0.3,0.3,-0.3,-0.3,1);
                return false;
            } else {
                setMotPow(0,0,0,0,0);
                return true;
            }
        } else {
            return true;
        }
    }
    public boolean dumbTurn(double degrees){
        setMotRUE();
        if (getHeading(AngleUnit.DEGREES)==degrees){
            setMotPow(0,0,0,0,0);
            return true;
        }else{
            setMotPow(0.1,0.1,-0.1,-0.1,1 );
            return false;
        }
    }


    public void stickDrive(double xCmd, double yCmd, double rxCmd, double spdMult, int robotConfig)
    // 0 = bogg, 1 = home robot, 2 = eliot
    {
        double denominator = Math.max(Math.abs(yCmd) + Math.abs(xCmd) + Math.abs(rxCmd), 1);
        xCmd = xCmd*-1;
        
        if (robotConfig == 0) {
            setMotPow(
                    (yCmd + xCmd - rxCmd) / denominator,
                    (yCmd - xCmd - rxCmd) / denominator,
                    (yCmd - xCmd + rxCmd) / denominator,
                    (yCmd + xCmd + rxCmd) / denominator,
                    spdMult);
        } else if (robotConfig == 1){
                setMotPow(
                        (yCmd + xCmd + rxCmd) / denominator,
                (yCmd - xCmd - rxCmd) / denominator,
                (yCmd - xCmd + rxCmd) / denominator,
                (yCmd + xCmd - rxCmd) / denominator,
                spdMult);
        } else if (robotConfig == 2){
            setMotPow(
                    (yCmd + xCmd + rxCmd) / denominator,
                    (yCmd - xCmd - rxCmd) / denominator,
                    (yCmd - xCmd + rxCmd) / denominator,
                    (yCmd + xCmd - rxCmd) / denominator,
                    spdMult);
        }

        
    }
    

    public void setMotPow(double flMotPow, double blMotPow, double frMotPow, double brMotPow, double spdMult) {
        flMot.setPower(flMotPow * spdMult);
        blMot.setPower(blMotPow * spdMult);
        frMot.setPower(frMotPow * spdMult);
        brMot.setPower(brMotPow * spdMult);

    }

    public void fullDrive(double x, double y, double rx, double spdMult, boolean up, boolean down, boolean left, boolean right) {
        if (!(y == 0) || !(x == 0) || !(rx == 0)) {
            this.stickDrive(x, y, rx, spdMult, 0);
        } else if (up || down || left || right) {
            if (up) {
                this.driveForward(spdMult);
            } else if (down) {
                this.driveReverse(spdMult);
            } else if (right) {
                this.driveRight(spdMult);
            } else if (left) {
                this.driveLeft(spdMult);
            }
        } else {
            this.setMotPow(0, 0, 0, 0, 0);
        }
    }

    public boolean moveForwardInches(int distance){
        int distanceTicks;
        if(targetDistance == 0){        // Move not started yet
            targetDistance = distance;
            return false;
        } else {
            distanceTicks = (int)(distance*ticksPerInch);
            flMot.setTargetPosition(distanceTicks);
            blMot.setTargetPosition(distanceTicks);
            frMot.setTargetPosition(distanceTicks);
            brMot.setTargetPosition(distanceTicks);
            this.setMotRTP();
            if (flMot.getCurrentPosition() >= distanceTicks){       // all done
                setMotPow(0,0,0,0,0);
                targetDistance = 0;
                return true;
            } else {                                                // run it forward
                this.setMotPow(0.3,0.3,0.3,0.3, 1);
                return false;
            }
        }




    }
    public void moveReverseInches(int distance){
        int distanceTicks = (int) (distance*ticksPerInch);
        flMot.setTargetPosition(-distanceTicks);
        blMot.setTargetPosition(-distanceTicks);
        frMot.setTargetPosition(-distanceTicks);
        brMot.setTargetPosition(-distanceTicks);
        this.setMotRTP();
    }
    public void moveLeftInches(int distance){
        int distanceTicks = (int) (distance*ticksPerInch);
        flMot.setTargetPosition(-distanceTicks);
        blMot.setTargetPosition(distanceTicks);
        frMot.setTargetPosition(distanceTicks);
        brMot.setTargetPosition(-distanceTicks);
        this.setMotRTP();
    }
    public void moveRightInches(int distance){
        int distanceTicks = (int) (distance*ticksPerInch);
        flMot.setTargetPosition(distanceTicks);
        blMot.setTargetPosition(-distanceTicks);
        frMot.setTargetPosition(-distanceTicks);
        brMot.setTargetPosition(distanceTicks);
        this.setMotRTP();
    }

    
    public void setMotRTP(){
        flMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotRUE(){
        flMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        blMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        brMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void setMotSRE(){
        flMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void singleMot (int whichMotor){
        if (whichMotor == 1 ){
            flMot.setPower(1);
        }
        if (whichMotor == 2 ){
            blMot.setPower(1);
        }
        if (whichMotor == 3 ){
            frMot.setPower(1);
        }
        if (whichMotor == 4 ){
            brMot.setPower(1);
        }
    }

}
