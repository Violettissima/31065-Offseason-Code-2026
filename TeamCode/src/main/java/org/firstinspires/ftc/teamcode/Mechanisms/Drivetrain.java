package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Drivetrain {
    private final double KP = 0;
    private final double KI = 0;
    private final double KD = 0;
    public double ANGLE_KP = 0.02;
    public double ANGLE_KD = 0.000001;
    private double lastAngleError = 0;
    private double angleTolerance = 0.4;
    private double targetX;
    private double targetY;
    private double targetAngle;
    private double lastYError;
    private double lastXError;
    private double xyTolerance = 0.5;
    private double xIntegralSum = 0;
    private double yIntegralSum = 0;
    private double speed = 1;
    private double lastTime = 0;
    private boolean isDrivetrainBusy = false;
    private ElapsedTime timer = new ElapsedTime();
    public enum DrivetrainState {
        MOVING,
        AIMING,
        IDLE
    }
    private DrivetrainState state = DrivetrainState.IDLE;
    private DcMotor FRdrive;
    private DcMotor FLdrive;
    private DcMotor BRdrive;
    private DcMotor BLdrive;
    SparkFunOTOS myOtos;
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        FRdrive = hwMap.get(DcMotor.class, "RF");
        FLdrive = hwMap.get(DcMotor.class, "LF");
        BRdrive = hwMap.get(DcMotor.class, "RB");
        BLdrive = hwMap.get(DcMotor.class, "LB");
        FRdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLdrive.setDirection(DcMotor.Direction.REVERSE);
        BLdrive.setDirection(DcMotor.Direction.REVERSE);
        FRdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        this.telemetry = telemetry;
    }

    public void start() {
        timer.reset();
    }

    public void update(){
        switch (state) {
            case MOVING:
                if (getX() > targetX - xyTolerance && getX() < targetX + xyTolerance &&
                        getY() > targetY - xyTolerance && getY() < targetY + xyTolerance &&
                        AngleUnit.normalizeDegrees(targetAngle - getH()) > -angleTolerance &&
                        AngleUnit.normalizeDegrees(targetAngle - getH()) < angleTolerance) {
                    isDrivetrainBusy = false;
                    drive(0, 0, 0);
                } else {
                    isDrivetrainBusy = true;
                    moveToPos(targetX, targetY, targetAngle);
                }
                break;
            case AIMING:
                lastXError = 0;
                lastYError = 0;
                break;
            case IDLE:
                lastAngleError = 0;
                lastXError = 0;
                lastYError = 0;
                break;
        }
        lastTime = timer.seconds();
    }

    private void moveToPos(double x, double y, double h){
        double yError = y - getY();
        double xError = x - getX();
        double elapsedTime = timer.seconds() - lastTime;

        xIntegralSum = xIntegralSum + xError * elapsedTime;
        xIntegralSum = Range.clip(xIntegralSum, -10, 10);
        double xDerivative = (xError - lastXError) / elapsedTime;
        double right = KP * xError + KI * xIntegralSum + KD * xDerivative;

        yIntegralSum = yIntegralSum + yError * elapsedTime;
        yIntegralSum = Range.clip(yIntegralSum, -10, 10);
        double yDerivative = (yError - lastYError) / elapsedTime;
        double forward = KP * yError + KI * yIntegralSum + KD * yDerivative;

        drive(forward, right, 0);
        lastXError = xError;
        lastYError = yError;
        lastTime = timer.seconds();
        telemetry.addData("x error", xError);
        telemetry.addData("y error", yError);
        telemetry.addData("forward", forward);
        telemetry.addData("right", right);
    }

    public void temporaryDriveAndAim(double forward, double right, double defaultRotate, AprilTagDetection tag) {
        double rotate;
        if (tag != null) {
            double angleError = -tag.ftcPose.bearing;
            telemetry.addData("apriltag", "seen");
            if (Math.abs(angleError) > angleTolerance) {
                double proportionalTerm = angleError * ANGLE_KP;
                double elapsedTime = timer.seconds() - lastTime;
                double derivativeTerm = (angleError - lastAngleError) / elapsedTime * ANGLE_KD;
                rotate = Range.clip(proportionalTerm + derivativeTerm, -0.4, 0.4);
                telemetry.addData("elapsed time", elapsedTime * 1000);
                lastAngleError = angleError;
            } else {
                rotate = 0;
                lastAngleError = 0;
            }
        } else {
            rotate = defaultRotate;
            lastAngleError = 0;
        }
        drive(forward, right, rotate);
    }

    public void driveAndAim(double forward, double right, AprilTagDetection tag) {
        double angleError;
        double rotate;
        if (tag != null) {
            angleError = -tag.ftcPose.bearing;
            telemetry.addData("apriltag", "seen");
        } else {
            angleError = getH();
        }
        if (Math.abs(angleError) > angleTolerance) {
            double proportionalTerm = angleError * ANGLE_KP;
            double elapsedTime = timer.seconds() - lastTime;
            double derivativeTerm = (angleError - lastAngleError) / elapsedTime * ANGLE_KD;
            rotate = Range.clip(proportionalTerm + derivativeTerm, -0.4, 0.4);
            lastAngleError = angleError;
        } else {
            rotate = 0;
            lastAngleError = 0;
        }
        drive(forward, right, rotate);
    }

    public void setTarget(double x, double y, double h) {
        targetX = x;
        targetY = y;
        targetAngle = h;
        xIntegralSum = 0;
        yIntegralSum = 0;
        state = DrivetrainState.MOVING;
    }

    public void drive(double forward, double right, double rotate){
        double robotAngle = Math.toRadians(getH());
        // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);
        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        driveRobotCentric(newForward, newRight, rotate);
    }

    public void driveRobotCentric(double forward, double right, double rotate){
        double frontRightPower = forward - right - rotate;
        double frontLeftPower = forward + right + rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;
        double maxSpeed = 1.0;

        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));

        frontRightPower = frontRightPower / maxSpeed * speed;
        frontLeftPower = frontLeftPower / maxSpeed * speed;
        backRightPower = backRightPower / maxSpeed * speed;
        backLeftPower = backLeftPower / maxSpeed * speed;

        FRdrive.setPower(frontRightPower);
        FLdrive.setPower(frontLeftPower);
        BRdrive.setPower(backRightPower);
        BLdrive.setPower(backLeftPower);
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    public void resetOtos() {
        myOtos.resetTracking();
    }

    public double getX(){
        return myOtos.getPosition().y;
    }

    public double getY(){
        return -myOtos.getPosition().x;
    }

    public double getH(){
        return myOtos.getPosition().h;
    }

    public boolean isBusy(){
        return isDrivetrainBusy;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setXyTolerance(double xyTolerance) {
        this.xyTolerance = xyTolerance;
    }

    public void setAngleTolerance(double angleTolerance) {
        this.angleTolerance = angleTolerance;
    }

    public void setIdle() {
        state = DrivetrainState.IDLE;
    }

    public void setAiming() {
        state = DrivetrainState.AIMING;
    }

}