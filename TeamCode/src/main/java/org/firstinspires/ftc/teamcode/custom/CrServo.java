package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CrServo {
    CRServo crServo = null;

    public CrServo(HardwareMap hwMap) {
        crServo = hwMap.crservo.get("crServoRubberWheel");
    }

    public boolean suck (){
        crServo.setPower(1);
        return true;

    }

    public boolean spit (){
        crServo.setPower(-1);
        return true;
    }

    public boolean stop(){
        crServo.setPower(0);
        return true;
    }
}
