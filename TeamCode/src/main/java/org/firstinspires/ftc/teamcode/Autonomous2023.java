package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Autonomous", group="Pushbot")
public class Autonomous2023 extends LinearOpMode {
    MecanumDrivetrain drivetrain;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    SleeveDetectionPipeline pipline = new SleeveDetectionPipeline();

    int FORWARD_TIME = 3000;
    int SIDE_TIME = 2000;

    @Override
    public void runOpMode() {

        drivetrain = new MecanumDrivetrain(
                hardwareMap,
                "left_front",
                "right_front",
                "left_back",
                "right_back"
        );
        drivetrain.setDirections(
                DcMotor.Direction.REVERSE,
                DcMotor.Direction.FORWARD,
                DcMotor.Direction.REVERSE,
                DcMotor.Direction.FORWARD
        );

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.setPipeline(pipline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {}
        });
        dashboard.startCameraStream(webcam, 30);

        waitForStart();

        SleeveDetectionPipeline.Color color = pipline.color;

        drivetrain.setPowersMecanum(0, 0.25, 0);
        sleep(500);

        dashboardTelemetry.clear();
        switch (color){
            case YELLOW:
            {
                dashboardTelemetry.addLine("Yellow");
                drivetrain.setPowersMecanum(0.4, 0, 0);
                sleep(SIDE_TIME);
                break;

            }
            case CYAN:
            {
                dashboardTelemetry.addLine("Cyan");
                break;
            }
            case MAGENTA:
            {
                dashboardTelemetry.addLine("Magenta");
                drivetrain.setPowersMecanum(-0.4, 0, 0);
                sleep(SIDE_TIME);
                break;
            }
        }


        drivetrain.setPowersMecanum(0, 0.25, 0);
        sleep(FORWARD_TIME);


        drivetrain.setPowersMecanum(0 , 0, 0);

        sleep(10000);

    }
}
