package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class Detectareautomata extends LinearOpMode
{
    OpenCvCamera camera;
    detectionpipline detectionpipline;

    static final double FEET_PER_METER = 3.28084;

    //calibrare camera
    //unitate ca pixeli
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // unitate in metri
    double tagsize = 0.166;

    // id la tag
    int left = 11;
    int mid = 12;
    int right = 13;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detectionpipline = new detectionpipline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(detectionpipline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //init loooooooooooooooooooooooooooooooooop

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = detectionpipline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == mid || tag.id ==right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("AM GASIT!\n\nLOCATIE:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("NU VAD NIMIC :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(NU AM VAZUT NICIODATA)");
                    }
                    else
                    {
                        telemetry.addLine("\nAM VAZUT TAGUL LA UN MOMENT DAT; ULTIMA LOCATIE VAZUTA:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("NU VAD NIMIC :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(NU AM VAZUT NICIODATA)");
                }
                else
                {
                    telemetry.addLine("\nAM VAZUT TAGUL LA UN MOMENT DAT; ULTIMA LOCATIE VAZUTA:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        //telemetry update
        if(tagOfInterest != null)
        {
            telemetry.addLine("TAG SNAPSHOT:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("Niciun tag vazut in init loop :(");
            telemetry.update();
        }

        if(tagOfInterest == null || tagOfInterest.id == left){
            //parcare stanga
        }else if(tagOfInterest.id == mid) {
            //parcare mijloc
        }else if(tagOfInterest.id == right){
            //parcare dreapta
        }


        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nTag detectat ID=%d", detection.id));
        telemetry.addLine(String.format("X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}