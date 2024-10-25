package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    DcMotor flMot = null;
    DcMotor blMot = null;
    DcMotor frMot = null;
    DcMotor brMot = null;

    public Drivetrain(HardwareMap hwMap) {

        flMot = hwMap.dcMotor.get("frontLeftMotor");
        blMot = hwMap.dcMotor.get("backLeftMotor");
        frMot = hwMap.dcMotor.get("frontRightMotor");
        brMot = hwMap.dcMotor.get("backRight");

        frMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMot.setDirection(DcMotorSimple.Direction.REVERSE);
        brMot.setDirection(DcMotorSimple.Direction.REVERSE);

        frMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        blMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        flMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        brMot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        


    }

    public void driveLeft(){
        setMotPow(1,1,1,1);
    }
    public void driveRight(){
        setMotPow(-1,1,-1,1);
    }
    public void driveForward(){
        setMotPow(1,-1,1,-1);
    }
    public void driveReverse(){
        setMotPow(-1,1,-1,-1);
    }
    
    public void stickDrive (double xCmd, double yCmd, double rxCmd){
        double denominator = Math.max(Math.abs(yCmd) + Math.abs(xCmd) + Math.abs(rxCmd), 1);
        setMotPow(
                (yCmd + xCmd - rxCmd) / denominator, 
                (yCmd - xCmd + rxCmd) / denominator, 
                (yCmd - xCmd - rxCmd) / denominator,
                (yCmd + xCmd + rxCmd) / denominator
                 );
    }

    /* double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    frontLeftPower = (y + x - rx) / denominator;
    backLeftPower = (y - x + rx) / denominator;
    frontRightPower = (y - x - rx) / denominator;
    backRightPower = (y + x + rx) / denominator; */

    public void setMotPow(double flMotPow, double blMotPow, double frMotPow, double brMotPow){
        flMot.setPower(flMotPow);
        blMot.setPower(blMotPow);
        frMot.setPower(frMotPow);
        brMot.setPower(brMotPow);

    }


}
