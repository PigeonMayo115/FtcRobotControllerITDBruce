package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.custom.Drivetrain;

// OpMode to Test Drivetrain.turnToHeading()
@Autonomous
public class TurnToHeadingTest extends OpMode {

    Drivetrain myDrivetrain;
    boolean done;

    @Override
    public void init() {
        myDrivetrain = new Drivetrain(hardwareMap, Drivetrain.Robot.MERRICK);
        done = false;
    }

    @Override
    public void init_loop(){
        telemetry.addData("heading", myDrivetrain.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public void loop() {
        telemetry.addData("heading", myDrivetrain.getHeading(AngleUnit.DEGREES));
        if(!done) {
            done = myDrivetrain.turnToHeading(-90, Drivetrain.Turn.LEFT);
        }
    }
}
