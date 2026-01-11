package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;




@TeleOp(name="mecanumDrive", group="Old")
@Disabled
public class MecanumTeleOp extends LinearOpMode {









    /* Declare OpMode members. */
    public DcMotor front_left = null; //the left drivetrain motor
    public DcMotor front_right = null; //the right drivetrain motor
    public DcMotor back_left = null; //the left drivetrain motor
    public DcMotor back_right = null; //the right drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    final double ARM_COLLECT       = 1.0;
    final double ARM_COLLAPSED_INTO_ROBOT       = 0.0;
    final double SPEED_MODIFIER = .5;




    private ElapsedTime runtime = new ElapsedTime();




    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;






    /* A number in degrees that the triggers can adjust the arm position by */


    /* Variables that are used to set the arm to a specific position */






    @Override
    public void runOpMode() {
     /*
















     /* Define and Initialize Motors */
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        armMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
















     /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
     for this robot, we reverse the right motor.*/
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);








     /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
     much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
     stops much quicker. */
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
















        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
















     /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
     Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
     If you do not have the encoder plugged into this motor, it will not run in this code. */












        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");








        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();








        /* Wait for the game driver to press play */
        waitForStart();








        /* Run until the driver presses stop */
        while (opModeIsActive()) {








         /* Set the drive and turn variables to follow the joysticks on the gamepad.
         the joysticks decrease as you push them up. So reverse the Y axis. */
            //forward = -gamepad1.left_stick_y;
            //rotate  = gamepad1.right_stick_x;








            double drive  = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist  = gamepad1.right_stick_x;








            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
            double[] speeds = {
                    (drive - strafe + twist),
                    (drive + strafe - twist),
                    (drive + strafe + twist),
                    (drive - strafe - twist)
            };








            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.








            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) )
                    max = Math.abs(speeds[i]);
            }








            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++)
                {
                    speeds[i] /= max;
                    speeds[i] *= SPEED_MODIFIER;
                }








            }












            front_left.setPower(speeds[0]);
            front_right.setPower(speeds[1]);
            back_left.setPower(speeds[2]);
            back_right.setPower(speeds[3]);








            // apply the
         /* Here we "mix" the input channels together to find the power to apply to each motor.
         The both motors need to be set to a mix of how much you're retesting the robot move
         forward, and how much you're requesting the robot turn. When you ask the robot to rotate
         the right and left motors need to move in opposite directions. So we will add rotate to
         forward for the left motor, and subtract rotate from forward for the right motor. */








         /*left  = forward + rotate;
         right = forward - rotate;








         Normalize the values so neither exceed +/- 1.0 */
         /*max = Math.max(Math.abs(left), Math.abs(right));
         if (max > 0.5)
          if (max > 1.0)
         {
             left /= max;
             right /= max;
         }
         /*
         /* Set the motor power to the variables we've mixed and normalized */
            //back_left.setPower(left);
            //back_right.setPower(right);
            //front_left.setPower(left);
            //front_right.setPower(right);
























         /* Here we handle the three buttons that have direct control of the intake speed.
         These control the continuous rotation servo that pulls elements into the robot,
         If the user presses A, it sets the intake power to the final variable that
         holds the speed we want to collect at.
         If the user presses X, it sets the servo to Off.
         And if the user presses B it reveres the servo to spit out the element.*/








         /* TECH TIP: If Else statements:
         We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
         multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
         at the same time. "a" will win over and the intake will turn on. If we just had
         three if statements, then it will set the intake servo's power to multiple speeds in
         one cycle. Which can cause strange behavior. */








            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }








            if(gamepad1.right_bumper){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                armMotor.setPower(1.0);  // 1.0 is full speed forward




                // Reset the timer when the button is pressed
                runtime.reset();




                // Wait for 3 seconds (or until the time reaches 3 seconds)
                while (runtime.seconds() < 0.5) {
                    // Do nothing here, just waiting for the time to pass
                    // You could add telemetry or any other checks if needed
                }




                // After 3 seconds, stop the motor
                armMotor.setPower(0.0);
            }




            // Check if the right bumper is pressed on the gamepad
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
        }












        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
















        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
        telemetry.update();








    }
}
















