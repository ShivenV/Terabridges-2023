package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

@Autonomous(name="RedOdometer DON'T USE", group="Pushbot")


public class AutonomousRedOdometer extends LinearOpMode {
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor arm = null;
    private Servo hand = null;
    private DcMotor wheel = null;
    private ElapsedTime runtime = new ElapsedTime();
    public void setPowers(double direction, double speed, double rotation){
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;


        double forward = Math.cos(direction)*speed;
        double side  =  Math.sin(direction)*speed;

        rightBackPower    = Range.clip((forward + side + rotation), -1.0, 1.0);
        leftBackPower   = Range.clip((forward - side - rotation), -1.0, 1.0);
        rightFrontPower    = Range.clip((forward - side + rotation), -1.0, 1.0);
        leftFrontPower   = Range.clip((forward + side - rotation), -1.0, 1.0);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }
    private void waitOneSecond() {
        sleep(1000);
    }
    @Override
    public void runOpMode() {


//        ArrayList<double[]> Instructions = new ArrayList<double[]>();
//        double instruction1[] = {1, 2, 3};
//        Instructions.add(instruction1);
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        wheel.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        waitForStart();

        double startTime = getRuntime();

        double xAmt = 0.125;//(Math.random() - 0.5)/2;
        double yAmt = 0.25;//(Math.random() - 0.5)/2;
        double leftFrontPower = yAmt + xAmt;
        double rightFrontPower = yAmt - xAmt;
        double leftBackPower = yAmt - xAmt;
        double rightBackPower = yAmt + xAmt;
        int leftFrontPreMovePosition = leftFront.getCurrentPosition();
        int rightFrontPreMovePosition = rightFront.getCurrentPosition();
        int leftBackPreMovePosition = leftBack.getCurrentPosition();
        int rightBackPreMovePosition = rightBack.getCurrentPosition();
        double leftFrontMultiplier = 1;
        double rightFrontMultiplier = 1;
        double leftBackMultiplier = 1;
        double rightBackMultiplier = 1;

        while(opModeIsActive()) {

            double time = getRuntime() + 1;
            while(getRuntime() < time){
                telemetry.clear();
                double xPos = leftFront.getCurrentPosition() + rightBack.getCurrentPosition() - rightFront.getCurrentPosition() - leftBack.getCurrentPosition();
                double yPos = leftFront.getCurrentPosition() + rightBack.getCurrentPosition() + rightFront.getCurrentPosition() + leftBack.getCurrentPosition();
                if(xPos > 1000){
                    xAmt = Math.abs(xAmt);
                    leftFrontPreMovePosition = leftFront.getCurrentPosition();
                    rightFrontPreMovePosition = rightFront.getCurrentPosition();
                    leftBackPreMovePosition = leftFront.getCurrentPosition();
                    rightBackPreMovePosition = rightFront.getCurrentPosition();
                    leftFrontPower = yAmt + xAmt;
                    rightFrontPower = yAmt - xAmt;
                    leftBackPower = yAmt - xAmt;
                    rightBackPower = yAmt + xAmt;

                }
                if(xPos < -1000){
                    xAmt = -Math.abs(xAmt);
                    leftFrontPreMovePosition = leftFront.getCurrentPosition();
                    rightFrontPreMovePosition = rightFront.getCurrentPosition();
                    leftBackPreMovePosition = leftFront.getCurrentPosition();
                    rightBackPreMovePosition = rightFront.getCurrentPosition();
                    leftFrontPower = yAmt + xAmt;
                    rightFrontPower = yAmt - xAmt;
                    leftBackPower = yAmt - xAmt;
                    rightBackPower = yAmt + xAmt;

                }
                if(yPos > 1000){
                    yAmt = Math.abs(yAmt);
                    leftFrontPreMovePosition = leftFront.getCurrentPosition();
                    rightFrontPreMovePosition = rightFront.getCurrentPosition();
                    leftBackPreMovePosition = leftFront.getCurrentPosition();
                    rightBackPreMovePosition = rightFront.getCurrentPosition();
                    leftFrontPower = yAmt + xAmt;
                    rightFrontPower = yAmt - xAmt;
                    leftBackPower = yAmt - xAmt;
                    rightBackPower = yAmt + xAmt;
                }
                if(yPos < -1000){
                    yAmt = -Math.abs(yAmt);
                    leftFrontPreMovePosition = leftFront.getCurrentPosition();
                    rightFrontPreMovePosition = rightFront.getCurrentPosition();
                    leftBackPreMovePosition = leftFront.getCurrentPosition();
                    rightBackPreMovePosition = rightFront.getCurrentPosition();
                    leftFrontPower = yAmt + xAmt;
                    rightFrontPower = yAmt - xAmt;
                    leftBackPower = yAmt - xAmt;
                    rightBackPower = yAmt + xAmt;
                }
                double leftFrontProgress = Math.abs((leftFront.getCurrentPosition() - leftFrontPreMovePosition)/leftFrontPower);
                double leftBackProgress = Math.abs((leftBack.getCurrentPosition() - leftBackPreMovePosition)/leftBackPower);
                double rightFrontProgress = Math.abs((rightFront.getCurrentPosition() - rightFrontPreMovePosition)/rightFrontPower);
                double rightBackProgress = Math.abs((rightBack.getCurrentPosition() - rightBackPreMovePosition)/rightBackPower);
                double averageProgress = (leftFrontProgress + leftBackProgress + rightFrontProgress + rightBackProgress)/4;
                double sensitivity = 2500; // Higher numbers mean more distance before change.
                leftFrontMultiplier = 1 + (averageProgress - leftFrontProgress)/sensitivity;
                rightFrontMultiplier = 1 + (averageProgress - rightFrontProgress)/sensitivity;
                leftBackMultiplier = 1 + (averageProgress - leftBackProgress)/sensitivity;
                rightBackMultiplier = 1 + (averageProgress - rightBackProgress)/sensitivity;
//                if(leftFrontProgress <= leftFrontProgress && leftFrontProgress <= leftBackProgress && leftFrontProgress <= rightFrontProgress && leftFrontProgress <= rightBackProgress){
//                    telemetry.addData("wheel is falling behind", "Left Front");
//                    leftFrontMultiplier *= 1.01;
//
//                }
//                if(rightFrontProgress <= leftFrontProgress && rightFrontProgress <= leftBackProgress && rightFrontProgress <= rightFrontProgress && rightFrontProgress <= rightBackProgress){
//                    telemetry.addData("wheel is falling behind", "Right Front");
//                    rightFrontMultiplier *= 1.02;
//                }
//                if(leftBackProgress <= leftFrontProgress && leftBackProgress <= leftBackProgress && leftBackProgress <= rightFrontProgress && leftBackProgress <= rightBackProgress){
//                    telemetry.addData("wheel is falling behind", "Left Back");
//                    leftBackMultiplier *= 1.02;
//                }
//                if(rightBackProgress <= leftFrontProgress && rightBackProgress <= leftBackProgress && rightBackProgress <= rightFrontProgress && rightBackProgress <= rightBackProgress){
//                    telemetry.addData("wheel is falling behind", "Right Back");
//                    rightBackMultiplier *= 1.02;
//                }
//                if(leftFrontMultiplier > 1){leftFrontMultiplier /= 1.004;}
//                if(rightFrontMultiplier > 1){rightFrontMultiplier /= 1.004;}
//                if(leftBackMultiplier > 1){leftBackMultiplier /= 1.004;}
//                if(rightBackMultiplier > 1){rightBackMultiplier /= 1.004;}
                telemetry.addData("LF", leftFrontProgress);
                telemetry.addData("RF", rightFrontProgress);
                telemetry.addData("LB", leftBackProgress);
                telemetry.addData("RB", rightBackProgress);



//                telemetry.addData("time", time - getRuntime());
//                telemetry.addData("x", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() - rightFront.getCurrentPosition() - leftBack.getCurrentPosition());
//                telemetry.addData("y", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() + rightFront.getCurrentPosition() + leftBack.getCurrentPosition());
                telemetry.update();
                leftFront.setPower(leftFrontPower*leftFrontMultiplier);
                rightFront.setPower(rightFrontPower*rightFrontMultiplier);
                leftBack.setPower(leftBackPower*leftBackMultiplier);
                rightBack.setPower(rightBackPower*rightBackMultiplier);

            }

            waitOneSecond();
        }


    }


}