package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Catapults {
    private final double SERVO_CLOSE_TIME = 0.5;
    private final double CATAPULT_LAUNCH_TIME = 0.5;
    private final double LAUNCH_INTERVAL = 0.5;
    private final double CATAPULT_TOLERANCE = 0.01;
    private int catapultBackPos = 2700;
    private double shotPower = 0.2;
    private int motifStage = 0;
    private boolean motifSet = false;
    private boolean timerSet;
    private DcMotor catapult1;
    private DcMotor catapult2;
    private boolean[] catapultReleased = {false, false, false};
    private Servo[] releases = new Servo[3];
    private NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];
    private TouchSensor touchSensor;
    private ElapsedTime stateTimer = new ElapsedTime();
    private Telemetry telemetry;

    private enum CatapultState {
        RELOAD_PULL_BACK,
        RELOAD_SERVO_CLOSE,
        SET_SHOT_POWER,
        IDLE,
        AUTO_SHOOT
    }
    public enum Color {
        PURPLE,
        GREEN
    }

    private CatapultState state = CatapultState.RELOAD_PULL_BACK;
    private Color[] catapultColors = {Color.PURPLE, Color.PURPLE, Color.PURPLE};
    private Color[] motif = {Color.PURPLE, Color.PURPLE, Color.PURPLE};

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        catapult1 = hardwareMap.get(DcMotor.class, "C");
        catapult2 = hardwareMap.get(DcMotor.class, "C2");
        catapult2.setDirection(DcMotorSimple.Direction.REVERSE);
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setPower(1);
        catapult2.setPower(1);
        catapult1.setTargetPosition(0);
        catapult2.setTargetPosition(0);
        catapult1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        releases[0] = hardwareMap.get(Servo.class, "r1");
        releases[1] = hardwareMap.get(Servo.class, "r2");
        releases[2] = hardwareMap.get(Servo.class, "r3");
        colorSensors[0] = hardwareMap.get(NormalizedColorSensor.class, "rs");
        colorSensors[1] = hardwareMap.get(NormalizedColorSensor.class, "ms");
        colorSensors[2] = hardwareMap.get(NormalizedColorSensor.class, "ls");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        this.telemetry = telemetry;

        for (int i = 0; i < 3; i++) {
            releaseCatapult(i);
            colorSensors[i].setGain(10);
        }
    }

    public void update() {
        telemetry.addData("catapultBackPos", catapultBackPos);
        telemetry.addData("catapult encoder 1", catapult1.getCurrentPosition());
        telemetry.addData("catapult encoder 2", catapult2.getCurrentPosition());
        switch(state) {
            case RELOAD_PULL_BACK:
                setCatapultPos(0);
                if (touchSensor.isPressed()) {
                    catapultBackPos = catapult1.getCurrentPosition() + 100;
                    state = CatapultState.RELOAD_SERVO_CLOSE;
                    stateTimer.reset();
                }
                break;
            case RELOAD_SERVO_CLOSE:
                setCatapultPos(0);
                telemetry.setAutoClear(false);
                closeServos();
                if (stateTimer.seconds() >= SERVO_CLOSE_TIME) {
                    state = CatapultState.SET_SHOT_POWER;
                }
                break;
            case SET_SHOT_POWER:
                setCatapultPos(shotPower);
                if (shotPower - CATAPULT_TOLERANCE < getCatapultPos()  && getCatapultPos() < CATAPULT_TOLERANCE + shotPower) {
                    state = CatapultState.IDLE;
                    timerSet = false;
                }
                break;
            case IDLE:
                setCatapultPos(shotPower);
                if (catapultReleased[0] && catapultReleased[1] && catapultReleased[2]) {
                    // Wait until sure that the last catapult has fully fired
                    if (!timerSet) {
                        stateTimer.reset();
                        timerSet = true;
                    } else if (stateTimer.seconds() >= CATAPULT_LAUNCH_TIME) {
                        state = CatapultState.RELOAD_PULL_BACK;
                    }
                }
                break;
            case AUTO_SHOOT:
                setCatapultPos(shotPower);
                if (!catapultReleased[0] && !catapultReleased[1] && !catapultReleased[2]) {
                    fireNextCatapult();
                    stateTimer.reset();
                } else if (!catapultReleased[0] || !catapultReleased[1] || !catapultReleased[2]) {
                    if (stateTimer.seconds() >= LAUNCH_INTERVAL) {
                        fireNextCatapult();
                        stateTimer.reset();
                    }
                } else {
                    if (stateTimer.seconds() >= CATAPULT_LAUNCH_TIME) {
                        state = CatapultState.RELOAD_PULL_BACK;
                    }
                }
                break;
        }
        for (int i = 0; i < 3; i++) {
            catapultColors[i] = getColor(i);
        }
    }

    private void fireNextCatapult() {
        for (int i = 0; i < 3; i++) {
            if (motif[motifStage] == catapultColors[i] && !catapultReleased[i]) {
                releaseCatapult(i);
                motifStage++;
                return;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (!catapultReleased[i]) {
                releaseCatapult(i);
                motifStage++;
                return;
            }
        }
    }

    private Color getColor(int catapultNum) {
        NormalizedRGBA colors = colorSensors[catapultNum].getNormalizedColors();
        float red = colors.red / colors.alpha;
        float green = colors.green / colors.alpha;
        float blue = colors.blue / colors.alpha;
        if (blue > green) {
            return Color.PURPLE;
        } else {
            return Color.GREEN;
        }
    }

    public void releaseCatapult(int catapultNum) {
        if (catapultNum == 0) {
            releases[0].setPosition(0.3);
        } else if (catapultNum == 1) {
            releases[1].setPosition(0.67);
        } else if (catapultNum == 2) {
            releases[2].setPosition(0.75);
        }
        catapultReleased[catapultNum] = true;
    }

    private void setCatapultPos(double power) {
        int position = (int) ((1 - power) * catapultBackPos);
        catapult1.setTargetPosition(position);
        catapult2.setTargetPosition(position);
    }

    private double getCatapultPos() {
        return (double) 1 - (double) catapult1.getCurrentPosition() / catapultBackPos;
    }
    private void closeServos() {
        releases[0].setPosition(0.7);
        releases[1].setPosition(0.22);
        releases[2].setPosition(0.35);
        catapultReleased[0] = false;
        catapultReleased[1] = false;
        catapultReleased[2] = false;
    }

    public void reload() {
        if (catapultReleased[0] || catapultReleased[1] || catapultReleased[2]) {
            state = CatapultState.RELOAD_PULL_BACK;
        }
    }

    public boolean isBusy() {
        return state != CatapultState.IDLE;
    }

    public void setMotif(int tagId) {
        if (tagId == 21) {
            motif = new Color[] {Color.GREEN, Color.PURPLE, Color.PURPLE};
        } else if (tagId == 22) {
            motif = new Color[] {Color.PURPLE, Color.GREEN, Color.PURPLE};
        } else if (tagId == 23) {
            motif = new Color[] {Color.PURPLE, Color.PURPLE, Color.GREEN};
        }
        motifSet = true;
    }

    public void autoShoot() {
        state = CatapultState.AUTO_SHOOT;
        motifStage = 0;
    }

    public void setShotPower(double distanceIn) {
        double power = 0;
        shotPower = Range.clip(power, 0, 1);
    }

    public boolean isMotifSet() {
        return motifSet;
    }
}