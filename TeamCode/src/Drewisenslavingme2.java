/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
// nsfiojaspofji vijsgpioadg iodfj;as

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Drew Drive2", group="Linear Opmode")
//@Disabled
public class Drewisenslavingme2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor lift = null;
    private DcMotor arm = null;
    private Servo leftHand = null;
    private Servo rightHand = null;
    private Servo hookServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(DcMotor.class,"arm");
        leftHand = hardwareMap.get(Servo.class, "leftHand");
        rightHand = hardwareMap.get(Servo.class, "rightHand");
        hookServo
                = hardwareMap.get(Servo.class, "hookServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //mode id, 0 = normal sped, 1 = speedivided b dy the slow modifier
        int mode = 0;
        int slow = 3;
        //int driveMode = 0;

        //open or closed state for the mineral arm
        int handState = 0;
        leftHand.setPosition(0);
        rightHand.setPosition(0);

        int hookState = 0;
        hookServo.setPosition(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = gamepad1.left_stick_y ; //USE THIS FOR UNIFORM DRIVE
            rightPower = gamepad1.right_stick_y ;

            /* code to make the robot drive straight easier
            if(driveMode == 0){
                if(mode == 0){
                    drive(leftPower, rightPower);
                } else {
                    drive(leftPower / slow, rightPower / slow);
                }
            } else {
                if(mode == 0){
                    drive(leftPower);
                } else {
                    drive(leftPower / slow);
                }
            }
             */

            //mode change
            if(gamepad1.a) {
                if(mode == 0){
                    mode = 1;
                } else {
                    mode = 0;
                }
            }

            // Send calculated power to wheels
            if(mode == 0){
                drive(leftPower, rightPower);
            } else {
                drive(leftPower / slow, rightPower / slow);
            }
            /*if(gamepad1.b){
                if(driveMode == 0){
                    driveMode = 1;
                } else {
                    driveMode = 0;
                }
            }*/
            //move the lift
            if(gamepad2.dpad_down) {
                lift.setPower(1);
            }
            else if(gamepad2.dpad_up) {
                lift.setPower(-1);
            }
            else {
                lift.setPower(0);
            }


            if(gamepad2.a){
                if(handState == 0){
                    handState = 1;
                } else {
                    handState = 0;
                }
            }
            if(handState == 0){
                leftHand.setPosition(180);
                rightHand.setPosition(180);
                telemetry.addData("Hand State: ", "Open - 180");
            } else {
                leftHand.setPosition(0);
                rightHand.setPosition(0);
                telemetry.addData("Hand State: ", "Closed - 0");
            }


            if(gamepad2.x) {
                if(hookState == 0){
                    hookState = 1;
                } else {
                    hookState = 0;
                }
            }

            if(hookState == 1){
                hookServo.setPosition(180);
            } else {
                hookServo.setPosition(0);
            }

            if(gamepad2.right_bumper){
                arm.setPower(.3);
            } else if(gamepad2.left_bumper) {
                arm.setPower(-.3);
            } else {
                arm.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
    public void drive(double lpower, double rpower){
        leftBackDrive.setPower(lpower);
        leftFrontDrive.setPower(lpower);
        rightBackDrive.setPower(rpower);
        rightFrontDrive.setPower(rpower);
    }

    public void drive(double power) {
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }

}
