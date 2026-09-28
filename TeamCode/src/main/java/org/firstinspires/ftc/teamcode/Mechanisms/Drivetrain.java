package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drivetrain {

    private DcMotor frDrive;
    private DcMotor flDrive;
    private DcMotor brDrive;
    private DcMotor blDrive;
    private SparkFunOTOS otos;
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        frDrive = hardwareMap.get(DcMotor.class, "fr");
        flDrive = hardwareMap.get(DcMotor.class, "fl");
        brDrive = hardwareMap.get(DcMotor.class, "br");
        blDrive = hardwareMap.get(DcMotor.class, "bl");
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOtos();
        this.telemetry = telemetry;
    }

    public void drive(double forward, double right, double rotate){
        double robotAngle = Math.toRadians(otos.getPosition().h);
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

        frontRightPower = frontRightPower / maxSpeed;
        frontLeftPower = frontLeftPower / maxSpeed;
        backRightPower = backRightPower / maxSpeed;
        backLeftPower = backLeftPower / maxSpeed;

        frDrive.setPower(frontRightPower);
        flDrive.setPower(frontLeftPower);
        brDrive.setPower(backRightPower);
        blDrive.setPower(backLeftPower);

        telemetry.addData("x", otos.getPosition().x);
        telemetry.addData("y", otos.getPosition().y);
        telemetry.addData("h", otos.getPosition().h);
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

}
