package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Guides {
    private Servo guideR;
    private Servo guideL;
    public enum Direction {
        RIGHT,
        LEFT,
        OPEN,
        CROSS
    }

    public void init(HardwareMap hardwareMap) {
        guideR = hardwareMap.get(Servo.class, "gr");
        guideL = hardwareMap.get(Servo.class, "gl");
    }

    public void move(Direction direction) {
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