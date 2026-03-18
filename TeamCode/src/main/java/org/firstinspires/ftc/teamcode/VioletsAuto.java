package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

@Autonomous
public class VioletsAuto extends OpMode {

    Drivetrain drivetrain = new Drivetrain();

    enum State {
        START_CENTER,
        UPPER_RIGHT,
        LOWER_RIGHT,
        LOWER_LEFT,
        UPPER_LEFT,
        RETURN_CENTER
    }
    State state = State.START_CENTER;
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 0;

    @Override
    public void init() {
        drivetrain.init(hardwareMap, telemetry);
        drivetrain.setSpeed(1);
    }

    @Override
    public void start() {
        drivetrain.start();
    }

    @Override
    public void loop() {

        switch(state) {
            case START_CENTER:
                drivetrain.setTarget(10, 10, 0);
                state = State.UPPER_RIGHT;
                break;
            case UPPER_RIGHT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(10, -10, 0);
                    state = State.LOWER_RIGHT;
                }
                break;
            case LOWER_RIGHT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(-10, -10, 0);
                    state = State.LOWER_LEFT;
                }
                break;
            case LOWER_LEFT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(-10, 10, 0);
                    state = State.UPPER_LEFT;
                }
                break;
            case UPPER_LEFT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(0, 0, 180);
                    state = State.RETURN_CENTER;
                }
                break;
            case RETURN_CENTER:
                break;
        }

        drivetrain.update();
        telemetry.addData("X", drivetrain.getX());
        telemetry.addData("Y", drivetrain.getY());
        telemetry.addData("H", drivetrain.getH());
        telemetry.addData("state", state);
    }
}