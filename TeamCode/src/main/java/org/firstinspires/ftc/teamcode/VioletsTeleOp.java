package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class
VioletsTeleOp extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    Catapults catapults = new Catapults();
    Intake muncher = new Intake();
    Guides guides = new Guides();

    @Override
    public void init(){
        catapults.init(hardwareMap);
        drivetrain.init(hardwareMap);
        muncher.init(hardwareMap);
        guides.init(hardwareMap);
    }

    @Override
    public void loop(){
        catapults.update();

        if (gamepad1.right_stick_button){
            drivetrain.setSpeed(1);
        } else if (gamepad1.left_stick_button) {
            drivetrain.setSpeed(0.2);
        } else {
            drivetrain.setSpeed(0.5);
        }

        if (gamepad1.right_bumper){
            drivetrain.moveToPos(0, 0, 0);
        } else if (gamepad1.left_bumper) {
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.right_trigger > 0) {
            muncher.setSpeed(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            muncher.setSpeed(-gamepad1.left_trigger);
        } else {
            muncher.setSpeed(0);
        }

        if (!catapults.isBusy()) {
            if (gamepad1.a) {
                catapults.reload();
            } else if (!gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up){
                if (gamepad1.x) {
                    catapults.releaseCatapult1();
                }
                if (gamepad1.y) {
                    catapults.releaseCatapult2();
                }
                if (gamepad1.b) {
                    catapults.releaseCatapult3();
                }
            }
        }

        if (gamepad1.dpad_left) {
            guides.move(Guides.Direction.LEFT);
        } else if (gamepad1.dpad_right) {
            guides.move(Guides.Direction.RIGHT);
        } else if (gamepad1.dpad_up) {
            guides.move(Guides.Direction.CROSS);
        } else {
            guides.move(Guides.Direction.OPEN);
        }
    }
}