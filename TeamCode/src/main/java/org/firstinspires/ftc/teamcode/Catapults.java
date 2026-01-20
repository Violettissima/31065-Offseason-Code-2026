package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Catapults {
    private final int CATAPULT_BACK_POS = 2700;
    private final double SERVO_MOVE_TIME = 0.5;
    private boolean timerSet;
    private DcMotor catapult1;
    private DcMotor catapult2;
    private Servo release1;
    private Servo release2;
    private Servo release3;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    private TouchSensor touchSensor;
    private boolean catapult1Released = true;
    private boolean catapult2Released = true;
    private boolean catapult3Released = true;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum CatapultState {
        RELOAD_SERVO_RELEASE,
        RELOAD_PULL_BACK,
        RELOAD_SERVO_CLOSE,
        SET_SHOT_POWER,
        LAUNCH_RELEASE,
    }

    public CatapultState state = CatapultState.RELOAD_SERVO_RELEASE;

    public void init(HardwareMap hardwareMap) {
        catapult1 = hardwareMap.get(DcMotor.class, "C");
        catapult2 = hardwareMap.get(DcMotor.class, "C2");
        catapult1.setDirection(DcMotorSimple.Direction.FORWARD);
        catapult2.setDirection(DcMotorSimple.Direction.REVERSE);
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setPower(1);
        catapult2.setPower(1);
        catapult1.setTargetPosition(0);
        catapult2.setTargetPosition(0);
        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        release1 = hardwareMap.get(Servo.class, "r1");
        release2 = hardwareMap.get(Servo.class, "r2");
        release3 = hardwareMap.get(Servo.class, "r3");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "rs");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ms");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "ls");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
    }

    public void update() {
        switch(state) {
            case RELOAD_SERVO_RELEASE:
                pullBackServos();
                state = CatapultState.RELOAD_PULL_BACK;
                break;
            case RELOAD_PULL_BACK:
                pullBackCatapults();
                if (touchSensor.isPressed()) {
                    state = CatapultState.RELOAD_SERVO_CLOSE;
                    catapult1.getCurrentPosition();
                    stateTimer.reset();
                }
                break;
            case RELOAD_SERVO_CLOSE:
                closeServos();
                if (stateTimer.seconds()  >= SERVO_MOVE_TIME) {
                    state = CatapultState.SET_SHOT_POWER;
                }
                break;
            case SET_SHOT_POWER:
                catapult1.setTargetPosition(0);
                catapult2.setTargetPosition(0);
                if (!catapult1.isBusy()) {
                    state = CatapultState.LAUNCH_RELEASE;
                    timerSet = false;
                }
                break;
            case LAUNCH_RELEASE:
                if (catapult1Released && catapult2Released && catapult3Released) {
                    if (!timerSet) {
                        stateTimer.reset();
                        timerSet = true;
                    } else if (stateTimer.seconds() >= SERVO_MOVE_TIME) {
                        state = CatapultState.RELOAD_SERVO_RELEASE;
                    }
                }
                break;
        }
    }

    private void pullBackCatapults() {
        catapult1.setTargetPosition(CATAPULT_BACK_POS);
        catapult2.setTargetPosition(CATAPULT_BACK_POS);
    }
    private void pullBackServos() {
        if (catapult1Released) {
            release1.setPosition(0.3);
            catapult1Released = false;
        } if (catapult2Released) {
            release2.setPosition(0.67);
            catapult2Released = false;
        } if (catapult3Released) {
            release3.setPosition(0.75);
            catapult3Released = false;
        }
    }
    private void closeServos() {
        release1.setPosition(0.7);
        release2.setPosition(0.22);
        release3.setPosition(0.35);
    }
    public void releaseCatapult1() {
        release1.setPosition(0.3);
        catapult1Released = true;
    }
    public void releaseCatapult2() {
        release2.setPosition(0.67);
        catapult2Released = true;
    }
    public void releaseCatapult3() {
        release3.setPosition(0.75);
        catapult3Released = true;
    }
    public void reload() {
        if (catapult1Released || catapult2Released || catapult3Released) {
            state = CatapultState.RELOAD_SERVO_RELEASE;
        }
    }
    public boolean isBusy() {
        return state != CatapultState.LAUNCH_RELEASE;
    }
}