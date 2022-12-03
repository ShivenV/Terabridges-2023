package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(group = "powerplay", name = "servotest")

public class TestServo extends LinearOpMode {
    public static double MOTOR_POWER = 3;
    public static double SERVO_DISTANCE_1 = 0.1;
    public static double SERVO_DISTANCE_2 = 0;
    private DcMotorEx frontLift;
    private Servo rightHand;




    @Override
    public void runOpMode() throws InterruptedException {
        frontLift = hardwareMap.get(DcMotorEx.class, "lift");
        rightHand = hardwareMap.get(Servo.class, "left_hand");

        waitForStart();

        rightHand.setPosition(SERVO_DISTANCE_1);
        wait(500);
        rightHand.setPosition(SERVO_DISTANCE_2);
    }
}
