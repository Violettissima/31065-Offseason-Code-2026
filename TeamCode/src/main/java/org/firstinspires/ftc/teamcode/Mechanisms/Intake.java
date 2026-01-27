package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake1;
    private DcMotor intake2;
    public void init(HardwareMap hwMap) {
        intake1 = hwMap.get(DcMotor.class, "M");
        intake2 = hwMap.get(DcMotor.class, "M2");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed) {
        intake1.setPower(speed);
        intake2.setPower(speed);
    }
}