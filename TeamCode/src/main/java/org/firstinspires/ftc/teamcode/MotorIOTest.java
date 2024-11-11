package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.custom.Drivetrain;

/**
 * This is a basic opmode that runs one wheel forward when a button is pressed and displays
 * which motor should be running.  This is handy for debugging motor wiring issues by letting
 * the team confirm that the motors are wired to the proper channels and are wired with the
 * correct polarity.
 *
 *
 */

@TeleOp
public class MotorIOTest extends OpMode {
    Drivetrain myDriveTrain;
    @Override
    public void init() {
        myDriveTrain = new Drivetrain(hardwareMap, Drivetrain.Robot.MERRICK);
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.a){
            myDriveTrain.singleMot(1);
            telemetry.addData("motor: ","front left");
        }
        else if (gamepad1.b){
            myDriveTrain.singleMot(2);
            telemetry.addData("motor: ","back left");
        }
        else if (gamepad1.x){
            myDriveTrain.singleMot(3);
            telemetry.addData("motor: ","front right");
        }
        else if (gamepad1.y){
            myDriveTrain.singleMot(4);
            telemetry.addData("motor: ", "back right");

        }else {
            myDriveTrain.setMotPow(0,0,0,0,0);
            telemetry.addData("motor: ", "none");
        }

        telemetry.addData("front left position",myDriveTrain.flMot.getCurrentPosition());
        telemetry.addData("back left position",myDriveTrain.blMot.getCurrentPosition());
        telemetry.addData("front right position",myDriveTrain.frMot.getCurrentPosition());
        telemetry.addData("back right position",myDriveTrain.brMot.getCurrentPosition());

        if (gamepad1.dpad_down){
            myDriveTrain.setMotSRE();
        }else{
            myDriveTrain.setMotRUE();
        }


    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
