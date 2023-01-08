
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP", group="Iterative Opmode")
public class TeleOP extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor wheel = null;
    private DcMotor arm = null;
    private Servo hand = null;
    double wheelSpeed = 0;

//    double[] speeds = {0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 1};
    double[] speeds = {0.5, 1};
    int currentSpeed = 0;

    double leftBackInitial;
    double rightBackInitial;
    double leftFrontInitial;
    double rightFrontInitial;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftBackInitial = leftBack.getCurrentPosition();
        rightBackInitial = rightBack.getCurrentPosition();
        leftFrontInitial = leftFront.getCurrentPosition();
        rightFrontInitial = rightFront.getCurrentPosition();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    boolean aPressed = false;
    double powerMultiplier = 0.0;
    double rotationMultiplier = 0.5;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double forward = -gamepad1.left_stick_y*powerMultiplier;
        double side  =  gamepad1.left_stick_x*powerMultiplier;
        double rotation = -gamepad1.right_stick_x*rotationMultiplier;
        rightBackPower    = Range.clip((forward + side + rotation), -1.0, 1.0) ;
        leftBackPower   = Range.clip((forward - side - rotation), -1.0, 1.0);
        rightFrontPower    = Range.clip((forward - side + rotation), -1.0, 1.0);
        leftFrontPower   = Range.clip((forward + side - rotation), -1.0, 1.0);

        if(gamepad1.dpad_up){
            leftFrontPower = powerMultiplier;
        }
        if(gamepad1.dpad_down){
            leftBackPower = powerMultiplier;
        }
        if(gamepad1.dpad_left){
            rightFrontPower = powerMultiplier;
        }
        if(gamepad1.dpad_right){
            rightBackPower = powerMultiplier;
        }

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

        if (gamepad1.a) {
            arm.setPower(1);
        } else if (gamepad1.y) {
            arm.setPower(-1);
        } else {
            arm.setPower(0);
        }
        if (gamepad1.left_bumper) {
            hand.setPosition(0.5);
        } else if (gamepad1.right_bumper) {
            hand.setPosition(0);
        }
        wheelSpeed = 0;
        wheel.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if(gamepad1.b) {
            if (!aPressed) {
                currentSpeed++;
                if(currentSpeed == speeds.length)
                    currentSpeed = 0;
            }
            aPressed = true;
        }else{
            aPressed = false;
        }
        if(gamepad1.x) {
            leftBackInitial = leftBack.getCurrentPosition();
            rightBackInitial = rightBack.getCurrentPosition();
            leftFrontInitial = leftFront.getCurrentPosition();
            rightFrontInitial = rightFront.getCurrentPosition();
        }
        powerMultiplier = speeds[currentSpeed];

        telemetry.clear();
//        telemetry.addData("Left front initial", leftFrontInitial);
//        telemetry.addData("Right front initial", rightFrontInitial);
//        telemetry.addData("Left back initial", leftBackInitial);
//        telemetry.addData("Right back initial", rightBackInitial);
        telemetry.addData("Left front", leftFront.getCurrentPosition() - leftFrontInitial);
        telemetry.addData("Right front", rightFront.getCurrentPosition() - rightFrontInitial);
        telemetry.addData("Left back", leftBack.getCurrentPosition() - leftBackInitial);
        telemetry.addData("Right back", rightBack.getCurrentPosition() - rightBackInitial);
        telemetry.addData("Power", powerMultiplier);

//        telemetry.addData("x", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() - rightFront.getCurrentPosition() - leftBack.getCurrentPosition());
//        telemetry.addData("y", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() + rightFront.getCurrentPosition() + leftBack.getCurrentPosition());
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
