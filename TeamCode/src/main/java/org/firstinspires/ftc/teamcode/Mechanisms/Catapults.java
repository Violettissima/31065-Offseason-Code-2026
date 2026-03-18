package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Catapults {
    private int catMaxBackPos = 0;
    private int touchSensorHitEncoderTicks = 0;
    private final double SERVO_CLOSE_TIME = 0.5;
    private final double CATAPULT_LAUNCH_TIME = 0.5;
    private final double LAUNCH_INTERVAL = 0.5;
    private final double CATAPULT_TOLERANCE = 0.01;
    private final int TOUCH_SENSOR_TO_STOP_TICKS = 120;
    private final int CATAPULT_RANGE = 2700;
    private int encoderOffset = 0;
    private double shotPower = 0.5;
    private int motifStage = 0;
    private boolean motifSet = false;
    private boolean timerSet;
    private DcMotor catapult1;
    private DcMotor catapult2;
    private final boolean[] catapultsReleased = {false, false, false};
    private final boolean[] catapultsFull = {false, false, false};
    private final Servo[] releases = new Servo[3];
    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[3];
    private TouchSensor touchSensor;
    private final ElapsedTime stateTimer = new ElapsedTime();
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
    private final Color[] catapultColors = {Color.PURPLE, Color.PURPLE, Color.PURPLE};
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
        distanceSensors[0] = hardwareMap.get(DistanceSensor.class, "rs");
        distanceSensors[1] = hardwareMap.get(DistanceSensor.class, "ms");
        distanceSensors[2] = hardwareMap.get(DistanceSensor.class, "ls");
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        this.telemetry = telemetry;

        for (int i = 0; i < 3; i++) {
            releaseCatapult(i);
            colorSensors[i].setGain(10);
        }
    }

    public void update() {
        catMaxBackPos = Math.max(catMaxBackPos, catapult1.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.addData("current power", getCatapultPos());
        telemetry.addData("target", shotPower);
        telemetry.addData("catapult maximum encoder reading", catMaxBackPos);
        telemetry.addData("encoder touch sensor hit position", touchSensorHitEncoderTicks);
        switch(state) {
            case RELOAD_PULL_BACK:
                setCatapultPos(0);
                if (touchSensor.isPressed()) {
                    touchSensorHitEncoderTicks = catapult1.getCurrentPosition();
                    encoderOffset = catapult1.getCurrentPosition() - (CATAPULT_RANGE - TOUCH_SENSOR_TO_STOP_TICKS);
                    state = CatapultState.RELOAD_SERVO_CLOSE;
                    stateTimer.reset();
                }
                break;
            case RELOAD_SERVO_CLOSE:
                setCatapultPos(0);
                closeServos();
                if (stateTimer.seconds() >= SERVO_CLOSE_TIME) {
                    state = CatapultState.SET_SHOT_POWER;
                }
                break;
            case SET_SHOT_POWER:
                setCatapultPos(shotPower);
                if (Math.abs(getCatapultPos() - shotPower) < CATAPULT_TOLERANCE) {
                    state = CatapultState.IDLE;
                    timerSet = false;
                }
                break;
            case IDLE:
                setCatapultPos(shotPower);
                if (catapultsReleased[0] && catapultsReleased[1] && catapultsReleased[2]) {
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
                if (!catapultsReleased[0] && !catapultsReleased[1] && !catapultsReleased[2]) {
                    fireNextCatapult();
                    stateTimer.reset();
                } else if (!catapultsReleased[0] || !catapultsReleased[1] || !catapultsReleased[2]) {
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
            catapultsFull[i] = distanceSensors[i].getDistance(DistanceUnit.CM) < 7;
        }
    }

    private void fireNextCatapult() {
        for (int i = 0; i < 3; i++) {
            if (motif[motifStage] == catapultColors[i] && !catapultsReleased[i]) {
                releaseCatapult(i);
                motifStage++;
                return;
            }
        }
        for (int i = 0; i < 3; i++) {
            if (!catapultsReleased[i]) {
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
        catapultsReleased[catapultNum] = true;
    }

    private void setCatapultPos(double power) {
        int catapult2Offset = catapult2.getCurrentPosition() - catapult1.getCurrentPosition();
        int position = (int) ((1 - power) * CATAPULT_RANGE);
        catapult1.setTargetPosition(position + encoderOffset);
        catapult2.setTargetPosition(position + encoderOffset + catapult2Offset);
    }

    private double getCatapultPos() {
        return 1 - (double) (catapult1.getCurrentPosition() - encoderOffset) / CATAPULT_RANGE;
    }
    private void closeServos() {
        releases[0].setPosition(0.7);
        releases[1].setPosition(0.22);
        releases[2].setPosition(0.35);
        catapultsReleased[0] = false;
        catapultsReleased[1] = false;
        catapultsReleased[2] = false;
    }

    public void reload() {
        if (catapultsReleased[0] || catapultsReleased[1] || catapultsReleased[2]) {
            state = CatapultState.RELOAD_PULL_BACK;
        }
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

    public void setShotDistance(double distanceIn) {
        double power = (int) (155 * Math.pow(1.0160, (1.54 * distanceIn - 15)) + 750);
        shotPower = Range.clip(power, 0, 1);
    }

    public void setShotPower(double shotPower) {
        this.shotPower = shotPower;
    }

    public boolean isMotifSet() {
        return motifSet;
    }

    public boolean isBusy() {
        return state != CatapultState.IDLE;
    }

    public boolean isLaunching() {
        return state == CatapultState.AUTO_SHOOT;
    }

    public boolean[] getCatapultsFull() {
        return catapultsFull;
    }
    public boolean[] getCatapultsReleased() {
        return catapultsReleased;
    }
}