package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoMain2 extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        String POSITION = "left";

        Pose2d StartBottom = new Pose2d(-35.5, -61, Math.toRadians(90));
        drive.setPoseEstimate(StartBottom);

        robot.init(hardwareMap);
        robot.EncoderReset();
        //pt inchis cleste
        robot.servoLeft.setPosition(0.07);
        robot.servoRight.setPosition(0.32);


        /** Build trajectories **/
        Trajectory StartToLow = drive.trajectoryBuilder(StartBottom) //TODO: sugiuc
                .lineToLinearHeading(new Pose2d(-28, -52, Math.toRadians(45)),
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0,()->{
                    runToPosition(2, "up");
                })
                .build();

        Trajectory LowToMid = drive.trajectoryBuilder(StartToLow.end(), true) //TODO: check if it needs lower power, that spline looks sussy on
                                                                                      // the acceleration side
                //it does in fact, otherwise it undershoots because funny field
                .splineToSplineHeading(new Pose2d(-34, -25, Math.toRadians(90)), 1.5,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        Trajectory PushCone1 = drive.trajectoryBuilder(StartToLow.end())
                .lineToLinearHeading(new Pose2d(-35.5, -55, Math.toRadians(60)))
                .addDisplacementMarker(()->drive.followTrajectoryAsync(PushCone2))
                .build();
        PushCone2 = drive.trajectoryBuilder(PushCone1.end())
                .lineToLinearHeading(new Pose2d(-35.5, -8, Math.toRadians(90)))
                .addDisplacementMarker(0, ()->{
                    brateCleste("open");
                    runToPosition(1, "down");
                })
                .addDisplacementMarker(()->drive.followTrajectoryAsync(PushCone3))
                .build();
        PushCone3 = drive.trajectoryBuilder(PushCone2.end(), true)
                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2)
                .addDisplacementMarker(1, ()->{
                    // grab 5th cone
                    int ticks = (int)(8 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();


        Trajectory MidToCones = drive.trajectoryBuilder(LowToMid.end())
                .splineTo(new Vector2d(-61,-10), Math.toRadians(180))
                .addDisplacementMarker(1, ()->{
                    // grab 5th cone
                    int ticks = (int)(8 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .addDisplacementMarker(4, ()->{
                    brateCleste("open");
                })
                .build();


        Trajectory LowToCones = drive.trajectoryBuilder(StartToLow.end(), true)
                .splineToSplineHeading(new Pose2d(-35.5, -25, Math.toRadians(45)), 1.5,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-62,-10,Math.toRadians(180)), 3,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(6, ()->{
                    // grab 5th cone
                    int ticks = (int)(8 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .addDisplacementMarker(4, ()->{
                    brateCleste("open");
                })
                .build();


        Trajectory ConesToMidJ = drive.trajectoryBuilder(MidToCones.end(), true)
                .splineToSplineHeading(new Pose2d(-26.5, -17.5, Math.toRadians(315)), 5.5, //todo: fine tune speed to go faster, prev 28
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2,()->{
                    runToPosition(3,"up");
                })
                .build();

        Trajectory MidJToCones2 = drive.trajectoryBuilder(ConesToMidJ.end(), true) //todo: fine tune speed to go faster, prev 28
                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2, ()->{
                    // grab 5th cone
                    int ticks = (int)(6 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();

        Trajectory MidJToCones3 = drive.trajectoryBuilder(ConesToMidJ.end(), true) //todo: fine tune speed to go faster, prev 28
                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2, ()->{
                    // grab 5th cone
                    int ticks = (int)(4 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
                })
                .build();

        Trajectory MidJtoPos11 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(-48, -10, Math.toRadians(180)),
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(MidJToPos12))
                .build();

        MidJToPos12 = drive.trajectoryBuilder(MidJtoPos11.end())
                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(270)),
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(1.5, ()->{
                    runToPosition(1, "down");
                })
                .build();
        Trajectory MidJToPos2 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(270)))
                .addDisplacementMarker(2, ()->{
                    runToPosition(1, "down");
                })
                .build();
        Trajectory MidJToPos3 = drive.trajectoryBuilder(MidJToCones3.end())
                .lineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(270)))
                .addDisplacementMarker(2, ()->{
                    runToPosition(1, "down");
                })
                .build();


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
                        if(!robot.digitalTouch.getState()){ //if button is pressed against the wall
                            drive.breakFollowing();
                            /**  5th cone  */
                            brateCleste("closed");
                            sleep(300);
                            runToPosition(2, "up");
                            while(robot.bratz.getCurrentPosition() > -(int)(23 * TICKS_PER_CM_Z) && opModeIsActive()){
                                telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                                telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                                telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                                telemetry.addData("while target: ", (int)(23 * TICKS_PER_CM_Z));
                                telemetry.update();
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
                            telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                            telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                            telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                            telemetry.addData("while target: ", (int)(23 * TICKS_PER_CM_Z));
                            telemetry.update();
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
                        } else if(coneOrder==3){
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
                                telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                                telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                                telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                                telemetry.addData("while target: ", (int)(21 * TICKS_PER_CM_Z));
                                telemetry.update();
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
                            telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                            telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                            telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                            telemetry.addData("while target: ", (int)(21 * TICKS_PER_CM_Z));
                            telemetry.update();
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
                                telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                                telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                                telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                                telemetry.addData("while target: ", (int)(19 * TICKS_PER_CM_Z));
                                telemetry.update();
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
                        while(robot.bratz.getCurrentPosition() > -(int)(19 * TICKS_PER_CM_Z) && opModeIsActive()){
                            telemetry.addData("glisiera busy: ", robot.bratz.isBusy());
                            telemetry.addData("current position: ", robot.bratz.getCurrentPosition());
                            telemetry.addData("target position: ", robot.bratz.getTargetPosition());
                            telemetry.addData("while target: ", (int)(19 * TICKS_PER_CM_Z));
                            telemetry.update();
                        }
                        currentState = State.TRAJ_3REPEAT;
                        drive.followTrajectoryAsync(ConesToMidJ);
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
            telemetry.addData("Button pressed:", robot.digitalTouch.getState());
            telemetry.update();
        }
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
