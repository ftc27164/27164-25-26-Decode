package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="Move out red", group="Competition")
//@Disabled
public class MoveOutBlue extends LinearOpMode {

    // Constants - removed duplicates
    static final double DRIVE_SPEED = 0.6;
    static final double DRIVE_SPEED_REVERSE = -0.6;
    static final double STRAFE_SPEED = 0.6;
    static final double TURN_SPEED = 0.2;
    static final double INTAKE_SPEED = 1.0;
    static final double LAUNCHER_SPEED_UP = 1.0;
    static final double LAUNCHER_SPEED_DOWN = -1.0;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.1;
    static final double P_DRIVE_GAIN = 0.03;

    // Declare OpMode members
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    public DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Variables for heading control
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double SLIDE_MOTOR_TICKS_PER_MM = (100548.0 / 187.0) / 120.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "left_front_drive");
        backLeft = hardwareMap.get(DcMotor.class, "left_back_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        // Initialize IMU - CONFIGURED FOR VERTICAL MOUNTING
        // Logo facing BACKWARD, USB pointing UP
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset IMU heading
        imu.resetYaw();

        // Autonomous sequence
        sleep(25000);
        moveRobotTime(DRIVE_SPEED,0,0,0.1);
        moveRobotTime(0,-STRAFE_SPEED,0,0.5);

    }

    public void moveRobotTime(double drive, double strafe, double turn, double time) {
        // function to move robot along desired axis, mimicking gamepad control
        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        // normalize wheel powers to not exceed +/- 1.0 range
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // send powers to the wheels
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveIntake(double time, double power) {
        runtime.reset();
        while (runtime.seconds() < time) {
            intake.setPower(power);
        }
        intake.setPower(0.0);
    }

    public void moveArm(double time, double direction) {
        runtime.reset();
        while (runtime.seconds() < time) {
            catapult2.setPower(direction);
            catapult1.setPower(direction);
        }
        catapult2.setPower(0);
        catapult1.setPower(0);
    }

    public void moveRobotTimeAndAction(double drive, double strafe, double turn, double time, double slidePower) {
        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            // Extend the arm simultaneously
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobotMecanum(0, 0, turnSpeed);

            // Display drive status for the driver.
            telemetry.addData("Heading Target", "%.1f", targetHeading);
            telemetry.addData("Heading Current", "%.1f", getHeading());
            telemetry.addData("Error", "%.1f", headingError);
            telemetry.update();
        }

        // Stop all motion
        moveRobotMecanum(0, 0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobotMecanum(double drive, double strafe, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        // Scale speeds down if either one exceeds +/- 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
