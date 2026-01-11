/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp Control", group = "Teleop")
public class OneManAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();
    private ElapsedTime autoReturnTimer = new ElapsedTime();

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // END EFFECTORS
    private DcMotorEx intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;
    private boolean gamepad = false;

    // Intake settings - using power instead of RPM for more reliable control
    private double INTAKE_POWER = 0.6;    // Adjust between 0.0 to 1.0
    private double OUTTAKE_POWER = -0.8;  // Adjust between -1.0 to 0.0

    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.5 ;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER = 1.0;
    private double CATAPULT_DOWN_POWER = -0.95;
    private double CATAPULT_HOLD_POWER = -0.4;

    // Auto-return settings
    private double AUTO_RETURN_DELAY = 1.0; // seconds to wait before auto-lowering
    private boolean autoReturnActive = false;

    private enum CatapultModes {UP, DOWN, HOLD, AUTO_RETURNING}
    private CatapultModes pivotMode;

    private enum FootMode {UP, DOWN, BRAKE}
    private FootMode footmode;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        // DRIVETRAIN
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // END EFFECTORS
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        foot = hardwareMap.get(DcMotor.class, "foot");

        // Directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot.setDirection(DcMotor.Direction.REVERSE);

        // Zero Power Behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Use power control
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        catatime.reset();

        while (opModeIsActive()) {

            // ----------- DRIVE INPUTS -----------
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.right_stick_x;
            double yaw     =  gamepad1.left_stick_x;

            // Mecanum math
            double leftFrontPower  = axial + lateral - yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial + lateral + yaw;
            double rightBackPower  = axial - lateral + yaw;

            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            if (gamepad1.dpad_down){
                boolean intakeInButton = gamepad1.left_trigger > 0.2;
                boolean intakeOutButton = gamepad1.left_bumper;

                if (intakeInButton && intakeOutButton) intakeInButton = false;

                if (intakeInButton) {
                    intake.setPower(INTAKE_POWER);
                } else if (intakeOutButton) {
                    intake.setPower(OUTTAKE_POWER);
                } else {
                    intake.setPower(0);
                }

                // ----------- FOOT ----------
                boolean footOutButton = gamepad1.a;
                boolean footUpButton = gamepad1.b;

                if (footOutButton && footUpButton) footOutButton = false;

                if (footOutButton) {
                    footmode = FootMode.DOWN;
                    footPower = FOOT_DOWN_POWER;
                } else if (footUpButton) {
                    footmode = FootMode.UP;
                    footPower = FOOT_UP_POWER;
                } else {
                    footmode = FootMode.BRAKE;
                    footPower = FOOT_OFF_POWER;
                }

                // ----------- CATAPULT WITH AUTO-RETURN -----------

                boolean catapultUpButton = gamepad1.right_bumper;
                boolean catapultDownButton = gamepad1.right_trigger > 0.2;

                if (catapultUpButton && catapultDownButton) catapultUpButton = false;

                // Manual control overrides auto-return
                if (catapultUpButton || catapultDownButton) {
                    autoReturnActive = false;
                }

                if (catapultUpButton) {
                    pivotMode = CatapultModes.UP;
                    catapult1.setPower(CATAPULT_UP_POWER);
                    catapult2.setPower(CATAPULT_UP_POWER);
                    // Start auto-return timer when UP is pressed
                    autoReturnTimer.reset();
                    autoReturnActive = true;
                } else if (catapultDownButton) {
                    pivotMode = CatapultModes.DOWN;
                    catapult1.setPower(CATAPULT_DOWN_POWER);
                    catapult2.setPower(CATAPULT_DOWN_POWER);
                } else if (autoReturnActive && autoReturnTimer.seconds() >= AUTO_RETURN_DELAY) {
                    // Auto-return: lower the catapult after delay
                    pivotMode = CatapultModes.AUTO_RETURNING;
                    catapult1.setPower(CATAPULT_DOWN_POWER);
                    catapult2.setPower(CATAPULT_DOWN_POWER);
                } else {
                    pivotMode = CatapultModes.HOLD;
                    catapult1.setPower(CATAPULT_HOLD_POWER);
                    catapult2.setPower(CATAPULT_HOLD_POWER);
                }

                // ----------- WRITE TO MOTORS -----------
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                foot.setPower(footPower);


            }
            // ----------- INTAKE -----------
            boolean intakeInButton = gamepad2.left_trigger > 0.2;
            boolean intakeOutButton = gamepad2.left_bumper;

            if (intakeInButton && intakeOutButton) intakeInButton = false;

            if (intakeInButton) {
                intake.setPower(INTAKE_POWER);
            } else if (intakeOutButton) {
                intake.setPower(OUTTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            // ----------- FOOT ----------
            boolean footOutButton = gamepad1.a;
            boolean footUpButton = gamepad1.b;

            if (footOutButton && footUpButton) footOutButton = false;

            if (footOutButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else if (footUpButton) {
                footmode = FootMode.UP;
                footPower = FOOT_UP_POWER;
            } else {
                footmode = FootMode.BRAKE;
                footPower = FOOT_OFF_POWER;
            }

            // ----------- CATAPULT WITH AUTO-RETURN -----------

            boolean catapultUpButton = gamepad2.right_bumper;
            boolean catapultDownButton = gamepad2.right_trigger > 0.2;

            if (catapultUpButton && catapultDownButton) catapultUpButton = false;

            // Manual control overrides auto-return
            if (catapultUpButton || catapultDownButton) {
                autoReturnActive = false;
            }

            if (catapultUpButton) {
                pivotMode = CatapultModes.UP;
                catapult1.setPower(CATAPULT_UP_POWER);
                catapult2.setPower(CATAPULT_UP_POWER);
                // Start auto-return timer when UP is pressed
                autoReturnTimer.reset();
                autoReturnActive = true;
            } else if (catapultDownButton) {
                pivotMode = CatapultModes.DOWN;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else if (autoReturnActive && autoReturnTimer.seconds() >= AUTO_RETURN_DELAY) {
                // Auto-return: lower the catapult after delay
                pivotMode = CatapultModes.AUTO_RETURNING;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                pivotMode = CatapultModes.HOLD;
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
            }

            // ----------- WRITE TO MOTORS -----------
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            foot.setPower(footPower);

            // ----------- TELEMETRY -----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Foot Mode", footmode);
            telemetry.addData("Catapult Mode", pivotMode);
            if (autoReturnActive) {
                telemetry.addData("Auto Return", "%.1f / %.1f sec",
                        autoReturnTimer.seconds(), AUTO_RETURN_DELAY);
            }
            telemetry.update();
        }
    }
}