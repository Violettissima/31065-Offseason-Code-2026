package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Mechanisms.Catapults;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class VioletsTeleOp extends OpMode {
    private boolean aligning = false;
    Drivetrain drivetrain = new Drivetrain();
    Catapults catapults = new Catapults();
    Intake muncher = new Intake();
    public AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    int stepIndex = 0;
    double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};

    @Override
    public void init(){
        catapults.init(hardwareMap, telemetry);
        drivetrain.init(hardwareMap, telemetry);
        muncher.init(hardwareMap);
        aprilTagWebcam.init(hardwareMap, telemetry);
        drivetrain.setXyTolerance(1);
    }

    @Override
    public void loop(){
        catapults.update();
        drivetrain.update();
        aprilTagWebcam.update();
        catapults.setMotif(aprilTagWebcam.getMotif());

        if (gamepad1.dpadDownWasPressed()) {
            aligning = !aligning;
        }

        if (gamepad1.right_stick_button){
            drivetrain.setSpeed(0.5);
        } else if (gamepad1.left_stick_button) {
            drivetrain.setSpeed(0.1);
        } else {
            drivetrain.setSpeed(1);
        }

        if (aligning) {
            AprilTagDetection id20 = aprilTagWebcam.getTagById(20);
            drivetrain.setAiming();
            drivetrain.temporaryDriveAndAim(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, id20);
            telemetry.addLine("Aiming");
        } else if (gamepad1.left_bumper) {
            drivetrain.setIdle();
            drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.setIdle();
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
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
            } else if (gamepad2.a) {
                catapults.autoShoot();
            } else {
                if (gamepad1.x) {
                    catapults.releaseCatapult(0);
                }
                if (gamepad1.y) {
                    catapults.releaseCatapult(1);
                }
                if (gamepad1.b) {
                    catapults.releaseCatapult(2);
                }
            }
        }

        if (gamepad1.dpad_left) {
            muncher.moveGuides(Intake.Direction.LEFT);
        } else if (gamepad1.dpad_right) {
            muncher.moveGuides(Intake.Direction.RIGHT);
        } else if (gamepad1.dpad_up) {
            muncher.moveGuides(Intake.Direction.CROSS);
        } else {
            muncher.moveGuides(Intake.Direction.OPEN);
        }

        if (gamepad1.start) {
            drivetrain.resetOtos();
        }

        {
            if (gamepad2.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                drivetrain.ANGLE_KP -= stepSizes[stepIndex];
            } else if (gamepad2.dpadRightWasPressed()) {
                drivetrain.ANGLE_KP += stepSizes[stepIndex];
            }
            if (gamepad2.dpadUpWasPressed()) {
                drivetrain.ANGLE_KD += stepSizes[stepIndex];
            } else if (gamepad2.dpadDownWasPressed()) {
                drivetrain.ANGLE_KD -= stepSizes[stepIndex];
            }
        }
    }
}