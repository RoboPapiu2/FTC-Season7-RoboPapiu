//package org.firstinspires.ftc.teamcode;
//
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Autonomous
//public class AUTOTHINGDREAPTA extends LinearOpMode {
//    hardwarePapiu robot = new hardwarePapiu();
//
//    OpenCvCamera camera;
//    detectionpipline detectionpipline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    //calibrare camera
//    //unitate ca pixeli
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // unitate in metri
//    double tagsize = 0.166;
//
//    /**id la tag**/
//    int left = 11;
//    int mid = 12;
//    int right = 13;
//
//    AprilTagDetection tagOfInterest = null;
//
//    enum State {
//        TRAJ_1,
//        TRAJ_2,
//        TRAJ_3REPEAT,
//        TRAJ4_1,
//        TRAJ4_2,
//        TRAJ_POS1,
//        TRAJ_POS2,
//        TRAJ_POS3,
//        IDLE
//    }
//    State currentState = State.IDLE;
//
//    int coneOrder = 1;
//    Trajectory PushCone2;
//    TrajectorySequence testPushCone3;
//    double conePos =-13;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        robot.EncoderReset();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        String POSITION = "left";
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        detectionpipline = new detectionpipline(tagsize, fx, fy, cx, cy);
//        camera.setPipeline(detectionpipline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        Pose2d StartBottom = new Pose2d(35.5, -61, Math.toRadians(90));
//        drive.setPoseEstimate(StartBottom);
//        //pt inchis cleste
//        brateCleste("closed");
//
//
//        /** Build trajectories **/
//        Trajectory mid = drive.trajectoryBuilder(StartBottom) //TODO: sugiuc
//                .lineToLinearHeading(new Pose2d(35.5, -15, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence left = drive.trajectorySequenceBuilder(StartBottom) //TODO: sugiuc
//                .lineToLinearHeading(new Pose2d(35.5, -15, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(15, -15, Math.toRadians(90)))
//                .build();
//
//        TrajectorySequence right = drive.trajectorySequenceBuilder(StartBottom) //TODO: sugiuc
//                .lineToLinearHeading(new Pose2d(35.5, -15, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(50, -15, Math.toRadians(90)))
//                .build();
//
//
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = detectionpipline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == left || tag.id == mid || tag.id ==right)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("AM GASIT!\n\nLOCATIE:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("NU VAD NIMIC :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(NU AM VAZUT NICIODATA)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nAM VAZUT TAGUL LA UN MOMENT DAT; ULTIMA LOCATIE VAZUTA:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("NU VAD NIMIC :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(NU AM VAZUT NICIODATA)");
//                }
//                else
//                {
//                    telemetry.addLine("\nAM VAZUT TAGUL LA UN MOMENT DAT; ULTIMA LOCATIE VAZUTA:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//
//        //telemetry update
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("TAG SNAPSHOT:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("Niciun tag vazut in init loop :(");
//            telemetry.update();
//        }
//
//        if(tagOfInterest == null || tagOfInterest.id == left){
//            POSITION ="left";
//        }else if(tagOfInterest.id == mid) {
//            POSITION="mid";
//        }else if(tagOfInterest.id == right){
//            POSITION="right";
//        }
//
//        waitForStart();
//        if(isStopRequested()) return;
//
//        currentState = State.TRAJ_1;
//        drive.followTrajectoryAsync(StartToLow);
//
//        while(opModeIsActive() && !isStopRequested()){
//
//            // Finite state
//            switch(currentState){
//                case TRAJ_1:
//                    if(!drive.isBusy()){
//                        brateCleste("open");
//                        sleep(400);
//                        currentState = State.TRAJ_2;
//                        drive.followTrajectoryAsync(PushCone1); //prev LowToCones
//                    }
//                    break;
//                case TRAJ_2:
//                    if(drive.isBusy()){
//                        brateCleste("open");
//                        if(!robot.digitalTouch.getState()){ //if button is pressed against the wall
//                            drive.breakFollowing();
//                            drive.setDrivePower(new Pose2d());
//                            /**  5th cone  */
//                            brateCleste("closed");
//                            sleep(300);
//                            runToPosition(2, "up");
//                            while(robot.bratz.getCurrentPosition() > -(int)(23 * TICKS_PER_CM_Z) && opModeIsActive()){
//                                //do nothing
//                            }
//                            currentState = State.TRAJ_3REPEAT;
//                            drive.followTrajectoryAsync(ConesToMidJ);
//                        }
//                    }
//                    else if(!drive.isBusy()){
//                        /**  5th cone  */
//                        brateCleste("closed");
//                        sleep(300);
//                        runToPosition(2, "up");
//                        while(robot.bratz.getCurrentPosition() > -(int)(23 * TICKS_PER_CM_Z) && opModeIsActive()){
//                            //do nothing
//                        }
//                        currentState = State.TRAJ_3REPEAT;
//                        drive.followTrajectoryAsync(ConesToMidJ);
//                    }
//                    break;
//                case TRAJ_3REPEAT:
//                    if(!drive.isBusy()){
//                        sleep(100);
//                        brateCleste("open");
//                        sleep(300);
//                        if(coneOrder==1){
//                            currentState = State.TRAJ4_1;
//                            coneOrder=2;
//                            drive.followTrajectorySequenceAsync(MidJToCones2);
//                        } else if(coneOrder==2){
//                            currentState = State.TRAJ4_1;
//                            coneOrder=3;
//                            drive.followTrajectorySequenceAsync(MidJToCones3);
//                        } else if(coneOrder==3){
//                            switch (POSITION) {
//                                case "left":
//                                    currentState = State.TRAJ_POS1;
//                                    drive.followTrajectorySequenceAsync(MidJtoPos1);
//                                    break;
//                                case "right":
//                                    currentState = State.TRAJ_POS3;
//                                    drive.followTrajectorySequenceAsync(MidJToPos3Sequence);
//                                    break;
//                                case "mid":
//                                default:
//                                    currentState = State.TRAJ_POS2;
//                                    drive.followTrajectorySequenceAsync(MidJToPos2);
//                                    break;
//                            }
//                        }
//                    }
//                    break;
//                case TRAJ4_1:
//                    if(drive.isBusy()){
//                        if(!robot.digitalTouch.getState()){
//                            drive.breakFollowing();
//                            drive.setDrivePower(new Pose2d());
//                            /**  5th cone  */
//                            brateCleste("closed");
//                            sleep(300);
//                            runToPosition(2, "up");
//                            while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
//                                telemetry.addLine("IN WHILE 1");
//                                telemetry.update();
//                            }
//                            currentState = State.TRAJ_3REPEAT;
//
//                            drive.followTrajectoryAsync(ConesToMidJ);
//                        }
//                    }
//                    if(!drive.isBusy()){
//                        /**  5th cone  */
//                        brateCleste("closed");
//                        sleep(300);
//                        runToPosition(2, "up");
//                        while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
//                            telemetry.addLine("IN WHILE 1");
//                            telemetry.update();
//                        }
//                        currentState = State.TRAJ_3REPEAT;
//
//                        drive.followTrajectoryAsync(ConesToMidJ);
//                    }
//                    break;
//                case TRAJ4_2:
//                    if(drive.isBusy()){
//                        if(!robot.digitalTouch.getState()){
//                            drive.breakFollowing();
//                            drive.setDrivePower(new Pose2d());
//                            /**  5th cone  */
//                            brateCleste("closed");
//                            sleep(300);
//                            runToPosition(2, "up");
//                            while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
//                                telemetry.addLine("IN WHILE 2");
//                                telemetry.update();
//                            }
//                            coneOrder=3;
//                            currentState = State.TRAJ_3REPEAT;
//                            drive.followTrajectory(ConesToMidJ);
//                        }
//                    }
//                    if(!drive.isBusy()){
//                        /**  5th cone  */
//                        brateCleste("closed");
//                        sleep(300);
//                        runToPosition(2, "up");
//                        while(robot.bratz.getCurrentPosition() > -(int)(21 * TICKS_PER_CM_Z) && opModeIsActive()){
//                            telemetry.addLine("IN WHILE 2");
//                            telemetry.update();
//                        }
//                        coneOrder=3;
//                        currentState = State.TRAJ_3REPEAT;
//                        drive.followTrajectory(ConesToMidJ);
//                    }
//                    break;
//                case TRAJ_POS2:
//                case TRAJ_POS3:
//                case TRAJ_POS1:
//                    if(!drive.isBusy()){
//                        currentState = State.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    break;
//            }
//
//            /** Transfer last pose to TeleOP */
//            TransferPose.currentPose = drive.getPoseEstimate();
//
//            drive.update();
//
//            telemetry.addData("x", drive.getPoseEstimate().getX());
//            telemetry.addData("y", drive.getPoseEstimate().getY());
//            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
//            telemetry.addData("coneOrder:", coneOrder);
//            telemetry.addData("Current trajectory: ", drive.getCurrentTrajectory());
//            telemetry.addData("Current state:", currentState);
//            telemetry.addData("Button pressed:", !robot.digitalTouch.getState());
//            telemetry.update();
//        }
//    }
//    @SuppressLint("DefaultLocale")
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nTag detectat ID=%d", detection.id));
//        telemetry.addLine(String.format("X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//    double PI = 3.1415;
//    double GEAR_MOTOR_40_TICKS = 1120;
//    double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
//    double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
//    double WHEEL_DIAMETER_CM = 4;
//    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
//    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);
//
//
//    public void runToPosition(int level, String rotation) {
//        int ticks;
//        if(rotation == "up"){
//            switch(level){
//                case 2:
//                    ticks = (int)(24 * TICKS_PER_CM_Z);
//                    robot.bratz.setTargetPosition(-ticks);
//                    break;
//                case 3:
//                    ticks = (int)((18+24) * TICKS_PER_CM_Z);
//                    robot.bratz.setTargetPosition(-ticks);
//                    break;
//                case 4:
//                    ticks = (int)((16+18+24) * TICKS_PER_CM_Z);
//                    robot.bratz.setTargetPosition(-ticks);
//                    break;
//            }
//        } else if(rotation == "down"){
//            switch(level){
//                case 1:
//                    robot.bratz.setTargetPosition(0);
//                    brateCleste("closed");
//                    break;
//                case 2:
//                    ticks = (int)((18+24) * TICKS_PER_CM_Z);
//                    robot.bratz.setTargetPosition(ticks);
//                    break;
//                case 3:
//                    ticks = (int)((16+18+24) * TICKS_PER_CM_Z);
//                    robot.bratz.setTargetPosition(ticks);
//                    break;
//            }
//        }
//        robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.bratz.setPower(0.6);
//    }
//    public void brateCleste(String state) {
//        if(state == "open"){ //pt deschis
//            robot.servoLeft.setPosition(0);
//            robot.servoRight.setPosition(0.4);
//        }
//        else if(state == "closed"){ //pt inchis
//            robot.servoLeft.setPosition(0.17);
//            robot.servoRight.setPosition(0.35);
//        }
//    }
//}