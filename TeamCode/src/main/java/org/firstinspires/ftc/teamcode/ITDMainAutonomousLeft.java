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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.custom.ArmMotor;
import org.firstinspires.ftc.teamcode.custom.CrServo;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;
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
public class ITDMainAutonomousLeft extends OpMode
{
    private Drivetrain myDrivetrain;
    private CrServo myCrServo;
    private ArmMotor myArmMotor;
    private Lift myLift;
    int step = 0;
    boolean stepDone = false;
    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap, 2);
        myCrServo = new CrServo(hardwareMap);
        myLift = new Lift(hardwareMap);



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("heading",myDrivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
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

        /* move forward, turn left, move forward, turn towards the basket, move forward,
        * extend arm motor, extend linear slide, spit out block, turn, drive to chamber,
        * turn, keep going, turn, move towards rung, move arm to touch low rung */

        switch(step){
            case 0:
                myDrivetrain.setMotSRE();       // clear the encoders
                step = 10;
                break;
            case 10://forwards 6 inches
                stepDone = myDrivetrain.moveForwardInches(6);
                if(stepDone){
                    myDrivetrain.setTargetHeading(90);
                    step = 20;
                }
                break;
            case 20: //turn right 90 degrees
                stepDone = myDrivetrain.turnToHeading(90);
                if (stepDone){
                    myDrivetrain.setMotSRE();       // clear the encoders
                    step = 30;
                }
                break;
            case 30: //forward 39 inches
                stepDone = myDrivetrain.moveForwardInches(39);
                if(stepDone){
                    myDrivetrain.setTargetHeading(135);
                    step = 40;
                }
                break;
            case 40:
                stepDone = myDrivetrain.turnToHeading(135);
                if (stepDone){
                    step = 50;
                }
                break;
            case 50:
                stepDone = myDrivetrain.moveForwardInches(10);
                if (stepDone){
                    step = 60;
                }
                break;
            case 60:
                stepDone = myArmMotor.armGoToAngle(135);
                if (stepDone){
                    step = 70;
                }
                break;
            case 70:
                stepDone = myLift.holdPosition(800,800);
                if (stepDone){
                    step = 80;
                }
                break;
            case 80:
                stepDone = myCrServo.spit();
                if (stepDone){
                    // I need it to wait like 1 second before this code is excecuted, Dont know how to do that
                    myCrServo.stop();
                    myDrivetrain.setTargetHeading(-45);
                    step = 90;
                }
            case 90:
                stepDone = myDrivetrain.turnToHeading(-45);
                if (stepDone){
                    step = 100;
                }
                break;
            case 100:
                stepDone = myDrivetrain.moveForwardInches(24);
                if (stepDone){
                    step = 110;
                    myDrivetrain.setTargetHeading(0);
                }
                break;
            case 110:
                stepDone = myDrivetrain.turnToHeading(0);
                if (stepDone){
                    step = 120;
                }
                break;
            case 120:
                stepDone = myDrivetrain.moveForwardInches( 6);
                if (stepDone){
                    step = 130;
                }
                break;
            case 130:
                stepDone = myArmMotor.armGoToAngle(0);
                if (stepDone){
                    step = 140;
                }

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
