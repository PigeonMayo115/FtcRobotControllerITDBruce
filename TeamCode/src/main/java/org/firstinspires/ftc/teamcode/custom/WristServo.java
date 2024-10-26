package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristServo {

    Servo servo = null;
    public WristServo(HardwareMap hwMap) {
        servo = hwMap.servo.get("wristServo");

    }
}
