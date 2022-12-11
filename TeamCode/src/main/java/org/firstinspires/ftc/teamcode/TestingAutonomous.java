package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Autonomous + Cone Detection")
public class TestingAutonomous extends LinearOpMode {
    public static double MOTOR_POWER = 3;
    public static double RIGHT_START = 0.54;
    public static double RIGHT_END = 0.61;
    public static double LEFT_START = 0.62;
    public static double LEFT_END = 0.54;
    private DcMotorEx frontLift;
    private Servo leftHand;
    private Servo rightHand;


    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        frontLift = hardwareMap.get(DcMotorEx.class, "lift");
        leftHand = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");
        //inital position
        drive.setPoseEstimate(new Pose2d(-40, 60, Math.toRadians(-90.0)));

        Trajectory leftPark = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-22.0, 60.0), 0)
                .splineTo(new Vector2d(-12.0, 45.0), -1.5707963267948966)
                .build();

        Trajectory centerPark = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-35.0, 36.0), 0)
                .build();

        Trajectory rightPark = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-60, 60), 0)
                .splineTo(new Vector2d(-60, 40), 0)
                .build();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);


        waitForStart();

        drive.followTrajectory(leftPark);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            dashboardTelemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}