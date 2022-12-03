package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "powerplay", name = "2022/3 tele-op")

public class PowerPlayTele extends LinearOpMode {
    public static double MOTOR_POWER = 3;
    public static double RIGHT_START = 0.54;
    public static double RIGHT_END = 0.61;
    public static double LEFT_START = 0.62;
    public static double LEFT_END = 0.54;
    private DcMotorEx frontLift;
    private Servo leftHand;
    private Servo rightHand;




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        frontLift = hardwareMap.get(DcMotorEx.class, "lift");
        leftHand = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");
        //inital position
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while (!isStopRequested()) {

            //updating localizer
            drive.update();

            // settings powers
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            // Set drive power
            drive.mecanumPower(x, y, rx);

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // lift code
            if(gamepad1.x) {
                drive.runLift(MOTOR_POWER);
            } else if (gamepad1.y) {
                drive.runLift(-MOTOR_POWER);
            } else {
                drive.runLift(0);
            }

            //TODO: fix claw code
            if(gamepad1.left_bumper) {
                leftHand.setPosition(LEFT_START);
                rightHand.setPosition(RIGHT_START);
            }
            if(gamepad1.right_bumper) {
                leftHand.setPosition(LEFT_END);
                rightHand.setPosition(RIGHT_END);
            }
        }
    }
}
