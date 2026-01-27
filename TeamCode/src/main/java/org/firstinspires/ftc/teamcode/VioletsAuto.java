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

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        drivetrain.setSpeed(0.25);
    }

    @Override
    public void loop() {

        switch(state) {
            case START_CENTER:
                drivetrain.setTarget(30, 30, 0);
                state = State.UPPER_RIGHT;
                break;
            case UPPER_RIGHT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(30, -30, 0);
                    state = State.LOWER_RIGHT;
                }
                break;
            case LOWER_RIGHT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(-30, -30, 0);
                    state = State.LOWER_LEFT;
                }
                break;
            case LOWER_LEFT:
                if (!drivetrain.isBusy()) {
                    drivetrain.setTarget(-30, 30, 0);
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