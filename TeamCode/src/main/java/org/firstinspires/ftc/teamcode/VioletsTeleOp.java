package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Mechanisms.Catapults;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Guides;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class VioletsTeleOp extends OpMode {
    private boolean aligning = false;
    Drivetrain drivetrain = new Drivetrain();
    Catapults catapults = new Catapults();
    Intake muncher = new Intake();
    Guides guides = new Guides();
    public AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    int stepIndex = 0;
    double[] stepSizes = {0.01, 0.001, 0.0001, 0.00001};

    @Override
    public void init(){
        catapults.init(hardwareMap, telemetry);
        drivetrain.init(hardwareMap, telemetry);
        muncher.init(hardwareMap);
        guides.init(hardwareMap);
        aprilTagWebcam.init(hardwareMap, telemetry);
        drivetrain.setXyTolerance(1);
    }

    @Override
    public void loop(){

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
            drivetrain.driveAndAim(-gamepad1.left_stick_y, gamepad1.left_stick_x, id20);
            telemetry.addLine("Aiming");
        } else if (gamepad1.right_bumper){
            drivetrain.setTarget(0, 0, 0);
        }else if (gamepad1.left_bumper) {
            drivetrain.cancelPath();
            drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drivetrain.cancelPath();
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
            } else if (!gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_up){
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
            guides.move(Guides.Direction.LEFT);
        } else if (gamepad1.dpad_right) {
            guides.move(Guides.Direction.RIGHT);
        } else if (gamepad1.dpad_up) {
            guides.move(Guides.Direction.CROSS);
        } else {
            guides.move(Guides.Direction.OPEN);
        }

        if (gamepad1.start) {
            drivetrain.resetOtos();
        }

        catapults.update();
        drivetrain.update();
        aprilTagWebcam.update();
        catapults.setMotif(aprilTagWebcam.getMotif());

        telemetry.addData("kp", drivetrain.KP);
        telemetry.addData("kd", drivetrain.KD);
        telemetry.addData("ki", drivetrain.KI);
        telemetry.addData("stepsize", stepSizes[stepIndex]);
        telemetry.addData("Drivetrain busy", drivetrain.isBusy());
        telemetry.addData("x", drivetrain.getX());
        telemetry.addData("y", drivetrain.getY());
        telemetry.addData("h", drivetrain.getH());
    }
}