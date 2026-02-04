package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class WebcamTest extends OpMode {
    Drivetrain drivetrain = new Drivetrain();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap);
        drivetrain.init(hardwareMap);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        drivetrain.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagById(20);
    }
}
