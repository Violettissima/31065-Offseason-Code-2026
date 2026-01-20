package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drivetrain {
    private DcMotor FRdrive;
    private DcMotor FLdrive;
    private DcMotor BRdrive;
    private DcMotor BLdrive;
    SparkFunOTOS myOtos;
    private double targetX = 0;
    private double targetY = 0;
    private double targetAngle = 0;
    private double speed = 1.0;
    private double xyTolerance = 0.5;
    private double angleTolerance = 2;
    private enum DrivetrainState {
        MOVING,
        IDLE
    }
    private DrivetrainState state = DrivetrainState.IDLE;

    public void init(HardwareMap hwMap){
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
    }

    public void drive(double forward, double right, double rotate){
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

    public void fieldCentricDrive(double forward, double right, double rotate){
        double robotAngle = Math.toRadians(getH());
        // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);
        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }

    public void moveToPos(double x, double y, double h){
        double forward;
        double right;
        double rotate;
        double yDistance = y - getY() ;
        double xDistance = x - getX() ;
        double hDistance = AngleUnit.normalizeDegrees(h - getH());
        if (yDistance > xyTolerance) {
            forward = Math.min(Math.pow(yDistance / 10, 4) + 0.1, 1);
        } else if (yDistance < -xyTolerance){
            forward = Math.min(-Math.pow(yDistance / 10, 4) - 0.1, -1);
        } else {
            forward = 0;
        }
        if (xDistance > xyTolerance) {
            right = Math.min(Math.pow(xDistance / 10, 4) + 0.1, 1);
        } else if (xDistance < -xyTolerance) {
            right = Math.max(-Math.pow(xDistance / 10, 4) - 0.1, -1);
        } else {
            right = 0;
        }
        if (hDistance > angleTolerance){
            rotate = Math.max(-Math.pow(hDistance / 20, 4) - 0.05, -1);
        } else if (hDistance < -angleTolerance){
            rotate = Math.max(Math.pow(hDistance / 20, 4) + 0.05, 1);
        } else {
            rotate = 0;
        }
        fieldCentricDrive(forward, right, rotate);
    }

    public void update(){
        switch (state) {
            case MOVING:
                if (getX() > targetX - xyTolerance && getX() < targetX + xyTolerance &&
                        getY() > targetY - xyTolerance && getY() < targetY + xyTolerance &&
                        AngleUnit.normalizeDegrees(targetAngle - getH()) > -angleTolerance &&
                        AngleUnit.normalizeDegrees(targetAngle - getH()) < angleTolerance){
                    state = DrivetrainState.IDLE;
                    drive(0, 0, 0);
                } else {
                    moveToPos(targetX, targetY, targetAngle);
                }
                break;
            case IDLE:
                break;
        }
    }

    private void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.CM);
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
        return state != DrivetrainState.IDLE;
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

    public void setTarget(double x, double y, double h) {
        targetX = x;
        targetY = y;
        targetAngle = h;
        state = DrivetrainState.MOVING;
    }
}