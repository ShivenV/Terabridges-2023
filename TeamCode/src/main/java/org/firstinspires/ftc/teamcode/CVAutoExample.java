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


/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from ContourPipeline
 */
@Autonomous
public class CVAutoExample extends LinearOpMode
{
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;

    OpenCvWebcam webcam;
    PurplePipeline.findObjPipe pipelinePurp;
    PurplePipeline.findObjPipe.ObjectPosition snapshotAnalysisPurp = PurplePipeline.findObjPipe.ObjectPosition.NOTPRESENT; // default
    OrangePipeline.findObjPipe pipelineOrang;
    OrangePipeline.findObjPipe.ObjectPosition snapshotAnalysisOrang = OrangePipeline.findObjPipe.ObjectPosition.NOTPRESENT; // default
    GreenPipeline.findObjPipe pipelineGreen;
    GreenPipeline.findObjPipe.ObjectPosition snapshotAnalysisGreen = GreenPipeline.findObjPipe.ObjectPosition.NOTPRESENT; // default
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipelinePurp = new PurplePipeline.findObjPipe();
        pipelineOrang = new OrangePipeline.findObjPipe();
        pipelineGreen = new GreenPipeline.findObjPipe();
        webcam.setPipeline(pipelinePurp);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
            telemetry.addData("Realtime analysis purple", pipelinePurp.getAnalysis());
            telemetry.addData("Realtime analysis orange", pipelineOrang.getAnalysis());
            telemetry.addData("Realtime analysis green", pipelineGreen.getAnalysis());
            telemetry.update();

            webcam.setPipeline(pipelinePurp);
            sleep(50);
            webcam.setPipeline(pipelineGreen);
            sleep(50);
            webcam.setPipeline(pipelineOrang);
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysisPurp = pipelinePurp.getAnalysis();
        snapshotAnalysisGreen = pipelineGreen.getAnalysis();
        snapshotAnalysisOrang = pipelineOrang.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Purple post-START analysis", snapshotAnalysisPurp);
        telemetry.addData("Green post-START analysis", snapshotAnalysisGreen);
        telemetry.addData("Orange post-START analysis", snapshotAnalysisOrang);

        telemetry.update();

        switch (snapshotAnalysisPurp)
        {
            case PRESENT:
            {
                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(0.5);
                sleep(65);
                break;
            }

            case NOTPRESENT:
            {
                dashboardTelemetry.addData("x", snapshotAnalysisPurp);
                dashboardTelemetry.update();
                break;
            }
        }

        switch (snapshotAnalysisGreen)
        {
            case PRESENT:
            {
                dashboardTelemetry.addData("x", snapshotAnalysisGreen);
                dashboardTelemetry.update();
                break;
            }

            case NOTPRESENT:
            {
                dashboardTelemetry.addData("x", snapshotAnalysisGreen);
                dashboardTelemetry.update();
                break;
            }
        }

        switch (snapshotAnalysisOrang)
        {
            case PRESENT:
            {
                dashboardTelemetry.addData("x", snapshotAnalysisOrang);
                dashboardTelemetry.update();
                break;
            }

            case NOTPRESENT:
            {
                dashboardTelemetry.addData("x", snapshotAnalysisOrang);
                dashboardTelemetry.update();
                break;
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}