package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.custom.DummyMotor;

/** Op Mode to test the dummy motor
 *  Also serves as an example of dummy motor usage
 *  JM 11/11/24
 */

@TeleOp
public class DummyMotorTest extends OpMode {

    public DcMotor armMot;
    public DummyMotor dummy;

    @Override
    public void init() {
        dummy = new DummyMotor();
        hardwareMap.put("armMotor", (DcMotor)dummy);
        armMot = hardwareMap.get(DcMotor.class, "armMotor");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        armMot.setPower(-gamepad1.right_stick_y);
        telemetry.addData("Postion", armMot.getCurrentPosition());
    }
}
