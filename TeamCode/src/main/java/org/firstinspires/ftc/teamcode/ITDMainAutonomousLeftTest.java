/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.CrServo;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;
import org.firstinspires.ftc.teamcode.custom.DummyMotor;
import org.firstinspires.ftc.teamcode.custom.Lift;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
public class ITDMainAutonomousLeftTest extends OpMode
{
    private Drivetrain myDrivetrain;
    private CrServo myCrServo;
    private Servo wristServo;
    private ArmMotor myArmMotor;
    private Lift myLift;
    int step = 0;
    boolean stepDone = false;
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain.Robot whichRobot;

    @Override
    public void init() {
        whichRobot = Drivetrain.Robot.MERRICK;      // Change it here if running on other robot
        if (whichRobot == Drivetrain.Robot.MERRICK){ // Set up dummy motors for lift and arm
            hardwareMap.put("linearSlideMotorLeft", (DcMotor)new DummyMotor());
            hardwareMap.put("linearSlideMotorRight", (DcMotor)new DummyMotor());
            hardwareMap.put("armMotor", (DcMotor)new DummyMotor());
        }
        myDrivetrain = new Drivetrain(hardwareMap, whichRobot);
        myCrServo = new CrServo(hardwareMap);
        myLift = new Lift(hardwareMap);
        myArmMotor = new ArmMotor(hardwareMap);
        wristServo = hardwareMap.servo.get("wristServo");



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("step: ",step);
        telemetry.addData("fl motor target", myDrivetrain.flMot.getTargetPosition());
        telemetry.addData("bl motor target", myDrivetrain.blMot.getTargetPosition());
        telemetry.addData("fr motor target", myDrivetrain.frMot.getTargetPosition());
        telemetry.addData("br motor target", myDrivetrain.brMot.getTargetPosition());
        telemetry.addData("armMotor",myArmMotor.armMot.getCurrentPosition());

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        myArmMotor.armMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //myDrivetrain.moveForwardInches(18);
        //myDrivetrain.setTargetHeading(-90);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("step: ",step);
        telemetry.addData("fl motor target", myDrivetrain.flMot.getTargetPosition());
        telemetry.addData("bl motor target", myDrivetrain.blMot.getTargetPosition());
        telemetry.addData("fr motor target", myDrivetrain.frMot.getTargetPosition());
        telemetry.addData("br motor target", myDrivetrain.brMot.getTargetPosition());
        telemetry.addData("armMotor",myArmMotor.armMot.getCurrentPosition());
        telemetry.addData("servo pos",wristServo.getPosition());



        /* move forward, turn left, move forward, turn towards the basket, move forward,
        * extend arm motor, extend linear slide, spit out block, turn, drive to chamber,
        * turn, keep going, turn, move towards rung, move arm to touch low rung */

        switch(step){
            case 0:
                // stops and resets all encoders
                myDrivetrain.setMotSRE();       // clear the encoders
                step = 5;
                break;
            case 5:                             // retract the wrist
                wristServo.setPosition(1);
                stepDone = (wristServo.getPosition() == 1);
                if(stepDone){
                    step = 6;
                }
                break;
            case 6:                             // lift the arm some
                stepDone = myArmMotor.armGoToAngle(-400);
                if (stepDone){
                    step = 7;
                }
                break;
            case 7:                             // center the wrist
                wristServo.setPosition(0.5);
                stepDone = (wristServo.getPosition() == 0.5);
                if (stepDone){
                    step = 10;
                }
                break;

            case 10:                            //forward 6 inches
                stepDone = myDrivetrain.moveForwardInches(15);
                if(stepDone){
                    step = 20;
                }
                break;
            case 20:                            //turn left 90 degrees
                stepDone = myDrivetrain.turnToHeading(90);
                if (stepDone){
                    step = 30;
                }
                break;
            case 30:                            //forward 39 inches
                stepDone = myDrivetrain.moveForwardInches(24);
                if(stepDone){
                    step = 40;
                }
                break;
            case 40:                            //turn toward baskets
                stepDone = myDrivetrain.turnToHeading(135);
                if (stepDone){
                    step = 50;
                }
                break;
            case 50:                            //move to baskets
                stepDone = myDrivetrain.moveForwardInches(10);
                if (stepDone){
                    step = 60;
                }
                break;
            case 60:                            //extend arm
                stepDone = myArmMotor.armGoToAngle(-3220);
                if (stepDone){
                    step = 70;
                }
                break;
            case 70:                            //raise lift
                stepDone = myLift.holdPosition(1610,1610);
                if (stepDone){
                    step = 80;
                }
                break;
            case 80:                            //spit out sample
                stepDone = myCrServo.spit(1,time);
                if (stepDone){
                    step = 90;
                }
                break;
                // TODO: I think you need to lower the lift before you start driving again
            case 90:                            //turn away from baskets
                stepDone = myDrivetrain.turnToHeading(-45);
                if (stepDone){
                    step = 100;
                }
                break;
            case 100:                           //drive toward submersible
                stepDone = myDrivetrain.moveForwardInches(24);
                if (stepDone){
                    step = 110;
                }
                break;
            case 110:                           //turn parallel to submersible
                stepDone = myDrivetrain.turnToHeading(0);
                if (stepDone){
                    step = 120;
                }
                break;
            case 120:                           //drive along submersible
                stepDone = myDrivetrain.moveForwardInches( 24);
                if (stepDone){
                    step = 130;
                }
                break;
            case 130:                           //turn toward submersible
                stepDone = myDrivetrain.turnToHeading(-90, Drivetrain.Turn.RIGHT);
                if (stepDone){
                    step = 135;
                }
                break;
            case 135:                           //drop lift (I think this needs to be done around step 90)
                stepDone = myLift.holdPosition(0,0);
                if (stepDone){
                    step = 140;
                }
                break;
            case 140:                           //extend arm to touch the bar
                stepDone = myArmMotor.armGoToAngle(-4000);
                if (stepDone) {
                    step = 150;
                }
                break;

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
