package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoMainDreapta2 extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();

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

    enum State {
        TRAJ_1,
        TRAJ_2,
        TRAJ_3REPEAT,
        TRAJ4_1,
        TRAJ4_2,
        TRAJ_POS1,
        TRAJ_POS2,
        TRAJ_POS3,
        IDLE
    }
    State currentState = State.IDLE;

    int coneOrder = 1;
    Trajectory MidJToPos12;
    Trajectory PushCone2;
    Trajectory PushCone3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.EncoderReset();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        String POSITION = "left";

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detectionpipline = new detectionpipline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(detectionpipline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d StartBottom = new Pose2d(35.5, -61, Math.toRadians(90));
        drive.setPoseEstimate(StartBottom);
        //pt inchis cleste
        robot.servoLeft.setPosition(0.07);
        robot.servoRight.setPosition(0.32);


        /** Build trajectories **/
        Trajectory StartToLow = drive.trajectoryBuilder(StartBottom) //TODO: sugiuc
                .lineToLinearHeading(new Pose2d(28, -52, Math.toRadians(135)),
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0,()->{
                    runToPosition(2, "up");
                })
                .build();


        Trajectory PushCone1 = drive.trajectoryBuilder(StartToLow.end())
                .lineToLinearHeading(new Pose2d(36.5, -56, Math.toRadians(120)))
                .addDisplacementMarker(()->drive.followTrajectoryAsync(PushCone2))
                .build();
        PushCone2 = drive.trajectoryBuilder(PushCone1.end())
                .lineToLinearHeading(new Pose2d(35.5, -8, Math.toRadians(90)), //prev y8
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0, ()->{
                    brateCleste("open");
                    runToPosition(1, "down");
                })
                .addDisplacementMarker(()->drive.followTrajectoryAsync(PushCone3))
                .build();
        PushCone3 = drive.trajectoryBuilder(PushCone2.end(), true)
                .splineToLinearHeading(new Pose2d(64.5, -13, Math.toRadians(0)), 0)
                .addDisplacementMarker(1, ()->{
                    // grab 5th cone
                    int ticks = (int)(8 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();


        Trajectory ConesToMidJ = drive.trajectoryBuilder(PushCone3.end(), true)
                .splineToSplineHeading(new Pose2d(27.5, -17.5, Math.toRadians(225)), 3.7, //todo: fine tune speed to go faster, prev 28
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2,()->{
                    runToPosition(3,"up");
                })
                .build();

        Trajectory MidJToCones2 = drive.trajectoryBuilder(ConesToMidJ.end(), true) //todo: fine tune speed to go faster, prev 28
                .splineToSplineHeading(new Pose2d(64.5, -13, Math.toRadians(0)), 0.1)
                .addDisplacementMarker(2, ()->{
                    // grab 5th cone
                    int ticks = (int)(6 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();

        Trajectory MidJToCones3 = drive.trajectoryBuilder(ConesToMidJ.end(), true) //todo: fine tune speed to go faster, prev 28
                .splineToSplineHeading(new Pose2d(64.5, -13, Math.toRadians(0)), 0.1)
                .addDisplacementMarker(2, ()->{
                    // grab 5th cone
                    int ticks = (int)(4 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();

        Trajectory MidJtoPos11 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(0)))
                .addDisplacementMarker(()-> drive.followTrajectoryAsync(MidJToPos12))
                .build();

        MidJToPos12 = drive.trajectoryBuilder(MidJtoPos11.end())
                .lineToLinearHeading(new Pose2d(57, -10, Math.toRadians(270)))
                .addDisplacementMarker(1.5, ()->{
                    runToPosition(1, "down");
                })
                .build();
        Trajectory MidJToPos2 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(270)))
                .addDisplacementMarker(2, ()->{
                    runToPosition(1, "down");
                })
                .build();
        Trajectory MidJToPos3 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(10, -10, Math.toRadians(270)))
                .addDisplacementMarker(2, ()->{
                    runToPosition(1, "down");
                })
                .build();

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
            POSITION ="left";
        }else if(tagOfInterest.id == mid) {
            POSITION="mid";
        }else if(tagOfInterest.id == right){
            POSITION="right";
        }

        waitForStart();
        if(isStopRequested()) return;

        currentState = State.TRAJ_1;
        drive.followTrajectoryAsync(StartToLow);

        while(opModeIsActive() && !isStopRequested()){

            // Finite state
            switch(currentState){
                case TRAJ_1:
                    if(!drive.isBusy()){
                        brateCleste("open");
                        sleep(200);
                        currentState = State.TRAJ_2;
                        drive.followTrajectoryAsync(PushCone1); //prev LowToCones
                    }
                    break;
                case TRAJ_2:
                    if(drive.isBusy()){
                        brateCleste("open");
                        if(!robot.digitalTouch.getState()){ //if button is pressed against the wall
                            drive.breakFollowing();
                            /**  5th cone  */
                            brateCleste("closed");
                            sleep(300);
                            runToPosition(2, "up");
                            while(robot.bratz.getCurrentPosition() > -(int)(23 * TICKS_PER_CM_Z) && opModeIsActive()){
                                //do nothing
                            }
                            currentState = State.TRAJ_3REPEAT;
                            drive.followTrajectoryAsync(ConesToMidJ);
                        }
                    }
                    else if(!drive.isBusy()){
                        /**  5th cone  */
                        brateCleste("closed");
                        sleep(300);
                        runToPosition(2, "up");
                        while(robot.bratz.getCurrentPosition() > -(int)(23 * TICKS_PER_CM_Z) && opModeIsActive()){
                            //do nothing
                        }
                        currentState = State.TRAJ_3REPEAT;
                        drive.followTrajectoryAsync(ConesToMidJ);
                    }
                    break;
                case TRAJ_3REPEAT:
                    if(!drive.isBusy()){
                        sleep(100);
                        brateCleste("open");
                        sleep(100);
                        if(coneOrder==1){
                            currentState = State.TRAJ4_1;
                            coneOrder++;
                            drive.followTrajectoryAsync(MidJToCones2);
                        } else if(coneOrder==2){
                            currentState = State.TRAJ4_2;
                            coneOrder++;
                            drive.followTrajectoryAsync(MidJToCones3);
                        }
                    }
                    break;
                case TRAJ4_1:
                    if(drive.isBusy()){
                        if(!robot.digitalTouch.getState()){
                            drive.breakFollowing();
                            /**  5th cone  */
                            brateCleste("closed");
                            sleep(300);
                            runToPosition(2, "up");
                            while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
                                //do nothing
                            }
                            currentState = State.TRAJ_3REPEAT;
                            drive.followTrajectoryAsync(ConesToMidJ);
                        }
                    }
                    if(!drive.isBusy()){
                        /**  5th cone  */
                        brateCleste("closed");
                        sleep(300);
                        runToPosition(2, "up");
                        while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
                            //do nothing
                        }
                        currentState = State.TRAJ_3REPEAT;
                        drive.followTrajectoryAsync(ConesToMidJ);
                    }
                    break;
                case TRAJ4_2:
                    if(drive.isBusy()){
                        if(!robot.digitalTouch.getState()){
                            drive.breakFollowing();
                            /**  5th cone  */
                            brateCleste("closed");
                            sleep(300);
                            runToPosition(2, "up");
                            while(robot.bratz.getCurrentPosition() > -(int)(19 * TICKS_PER_CM_Z) && opModeIsActive()){
                                //do nothing
                            }
                            coneOrder++;
                            if(POSITION =="left"){
                                currentState = State.TRAJ_POS1;
                                drive.followTrajectoryAsync(MidJtoPos11);
                            } else if(POSITION == "mid"){
                                currentState = State.TRAJ_POS2;
                                drive.followTrajectoryAsync(MidJToPos2);
                            }
                            else if(POSITION == "right"){
                                currentState = State.TRAJ_POS3;
                                drive.followTrajectoryAsync(MidJToPos3);
                            } else {
                                currentState = State.TRAJ_POS1;
                                drive.followTrajectoryAsync(MidJtoPos11);
                            }
                        }
                    }
                    if(!drive.isBusy()){
                        /**  5th cone  */
                        brateCleste("closed");
                        sleep(300);
                        runToPosition(2, "up");
                        while(robot.bratz.getCurrentPosition() > -(int)(19 * TICKS_PER_CM_Z) && opModeIsActive()){
                            //do nothing
                        }
                        coneOrder++;
                        if(POSITION =="left"){
                            currentState = State.TRAJ_POS1;
                            drive.followTrajectoryAsync(MidJtoPos11);
                        } else if(POSITION == "mid"){
                            currentState = State.TRAJ_POS2;
                            drive.followTrajectoryAsync(MidJToPos2);
                        }
                        else if(POSITION == "right"){
                            currentState = State.TRAJ_POS3;
                            drive.followTrajectoryAsync(MidJToPos3);
                        } else {
                            currentState = State.TRAJ_POS1;
                            drive.followTrajectoryAsync(MidJtoPos11);
                        }
                    }
                    break;
                case TRAJ_POS2:
                case TRAJ_POS3:
                case TRAJ_POS1:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            /** Transfer last pose to TeleOP */
            TransferPose.currentPose = drive.getPoseEstimate();

            drive.update();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("coneOrder:", coneOrder);
            telemetry.addData("Current trajectory: ", drive.getCurrentTrajectory());
            telemetry.addData("Current state:", currentState);
            telemetry.addData("Button pressed:", !robot.digitalTouch.getState());
            telemetry.update();
        }
    }
    @SuppressLint("DefaultLocale")
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
    double PI = 3.1415;
    double GEAR_MOTOR_40_TICKS = 1120;
    double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);


    public void runToPosition(int level, String rotation) {
        int ticks;
        if(rotation == "up"){
            switch(level){
                case 2:
                    ticks = (int)(24 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                    break;
                case 3:
                    ticks = (int)((18+24) * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                    break;
                case 4:
                    ticks = (int)((16+18+24) * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                    break;
            }
        } else if(rotation == "down"){
            switch(level){
                case 1:
                    robot.bratz.setTargetPosition(0);
                    brateCleste("closed");
                    break;
                case 2:
                    ticks = (int)((18+24) * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(ticks);
                    break;
                case 3:
                    ticks = (int)((16+18+24) * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(ticks);
                    break;
            }
        }
        robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratz.setPower(0.6);
    }
    public void brateCleste(String state) {
        if(state == "open"){ //pt deschis
            robot.servoLeft.setPosition(0);
            robot.servoRight.setPosition(0.4);
        }
        else if(state == "closed"){ //pt inchis
            robot.servoLeft.setPosition(0.07);
            robot.servoRight.setPosition(0.32);
        }
    }
}