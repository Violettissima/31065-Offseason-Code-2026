package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class VioletsAuto extends OpMode {

    Drivetrain drivetrain = new Drivetrain();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        drivetrain.setTarget(10, 10, 90);
        drivetrain.setDriveTrainState(Drivetrain.DriveTrainState.MOVING);
        drivetrain.setSpeed(0.1);
    }

    @Override
    public void loop() {
        drivetrain.update();
    }
}
