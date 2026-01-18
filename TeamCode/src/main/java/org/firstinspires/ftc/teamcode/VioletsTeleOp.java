package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VioletsTeleOp extends OpMode {
    Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init(){
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop(){
        double robotX = drivetrain.getX();
        double robotY = drivetrain.getY();
        double robotH = drivetrain.getH();

        if (gamepad1.right_stick_button){
            drivetrain.setSpeed(1);
        } else if (gamepad1.left_stick_button){
            drivetrain.setSpeed(0.2);
        } else {
            drivetrain.setSpeed(0.5);
        }
        if (gamepad1.a){
            drivetrain.moveToPos(0, 0, 0);
        } else {
            drivetrain.fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        telemetry.addData("X", robotX);
        telemetry.addData("Y", robotY);
        telemetry.addData("H", robotH);
    }
}