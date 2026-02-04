package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Catapults;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Guides;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

@TeleOp
public class
VioletsTeleOp extends OpMode {
    private boolean aligning = false;
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
        drivetrain.setXyTolerance(1);
    }

    @Override
    public void loop(){
        if (gamepad1.dpadDownWasPressed()) {
            aligning = !aligning;
        }

        if (gamepad1.right_stick_button){
            drivetrain.setSpeed(1);
        } else if (gamepad1.left_stick_button) {
            drivetrain.setSpeed(0.2);
        } else {
            drivetrain.setSpeed(0.5);
        }

        if (aligning) {
            drivetrain.driveAndAim(-gamepad1.left_stick_y, gamepad1.left_stick_x, 20);
        } else  if (gamepad1.right_bumper){
            drivetrain.setTarget(0, 0, 0);
        }else if (gamepad1.left_bumper) {
            drivetrain.cancelPath();
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.cancelPath();
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

        if (gamepad1.start && gamepad1.back) {
            drivetrain.resetOtos();
        }

        catapults.update();
        drivetrain.update();
        telemetry.addData("drivetrain is busy", drivetrain.isBusy());
        telemetry.addData("state", drivetrain.getState());
        telemetry.addData("x", drivetrain.getX());
        telemetry.addData("y", drivetrain.getY());
        telemetry.addData("h", drivetrain.getH());
    }
}