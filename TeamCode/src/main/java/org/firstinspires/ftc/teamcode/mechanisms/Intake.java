package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor outerMotor;
    private DcMotor innerMotor;
    private CRServo rightRoller;
    private CRServo leftRoller;

    public void init(HardwareMap hardwareMap) {
        outerMotor = hardwareMap.get(DcMotor.class, "intake_outer");
        innerMotor = hardwareMap.get(DcMotor.class, "intake_inner");
        outerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        innerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        innerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRoller = hardwareMap.get(CRServo.class, "right_roller");
        leftRoller = hardwareMap.get(CRServo.class, "left_roller");
    }
}
