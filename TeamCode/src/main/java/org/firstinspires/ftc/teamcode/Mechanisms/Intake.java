package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intake1;
    private DcMotor intake2;
    private Servo guideR;
    private Servo guideL;

    public enum Direction {
        RIGHT,
        LEFT,
        OPEN,
        CROSS
    }

    public void init(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(DcMotor.class, "M");
        intake2 = hardwareMap.get(DcMotor.class, "M2");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        guideR = hardwareMap.get(Servo.class, "gr");
        guideL = hardwareMap.get(Servo.class, "gl");
    }

    public void setSpeed(double speed) {
        intake1.setPower(speed);
        intake2.setPower(-speed);
    }

    public void moveGuides(Direction direction) {
        if (direction == Direction.LEFT) {
            guideR.setPosition(0.65);
            guideL.setPosition(0.097);
        } else if (direction == Direction.RIGHT) {
            guideR.setPosition(0.95);
            guideL.setPosition(0.4);
        } else if (direction == Direction.CROSS) {
            guideR.setPosition(0.742);
            guideL.setPosition(0.3);
        } else if (direction == Direction.OPEN) {
            guideR.setPosition(0.95);
            guideL.setPosition(0.097);
        }
    }
}