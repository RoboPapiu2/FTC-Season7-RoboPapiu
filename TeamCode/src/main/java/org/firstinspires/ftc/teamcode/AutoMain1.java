package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoMain1 extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();
    @Override
    public void runOpMode() throws InterruptedException{
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

        Trajectory LowToMid = drive.trajectoryBuilder(StartToLow.end(), true) //TODO: check if it needs lower power, that spline looks sussy on the acceleration side
                                                                                        //it does in fact, otherwise it undershoots because funny field
                .splineToSplineHeading(new Pose2d(-34, -25, Math.toRadians(90)), 1.5,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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


        Trajectory LowToCones1 = drive.trajectoryBuilder(StartToLow.end())
                .lineToLinearHeading(new Pose2d(-35.5, -55, Math.toRadians(45)))
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

        Trajectory LowToConesOld = drive.trajectoryBuilder(LowToCones1.end())
                .lineToLinearHeading(new Pose2d(-35.5, -25, Math.toRadians(45)))
                .splineToSplineHeading(new Pose2d(-61, -10, Math.toRadians(180)), 3.2,
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory ConesToMidJ = drive.trajectoryBuilder(MidToCones.end(), true)
                .splineToSplineHeading(new Pose2d(-26.5, -18.5, Math.toRadians(315)), 5.5, //todo: fine tune speed to go faster, prev 28
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2,()->{
                    runToPosition(3,"up");
                })
                .build();

        Trajectory MidJToCones = drive.trajectoryBuilder(ConesToMidJ.end(), true)
                .splineToSplineHeading(new Pose2d(-62, -10, Math.toRadians(180)), 3.2, //todo: fine tune speed to go faster, prev 28
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(2, ()->{
                    // grab 5th cone
                    int ticks = (int)(6 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(-ticks);
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
                .build();
        Trajectory MidJToPos12 = drive.trajectoryBuilder(MidJtoPos11.end())
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
        if(opModeIsActive()){

            /**  preload  */
            drive.followTrajectory(StartToLow);
            brateCleste("open");
            sleep(200);


            //drive.followTrajectory(LowToCones1);
            drive.followTrajectory(LowToCones);

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
            drive.followTrajectory(ConesToMidJ);
            sleep(100);
            brateCleste("open");
            sleep(100);

            /** 4th cone */
            drive.followTrajectory(MidJToCones2);
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
            drive.followTrajectory(ConesToMidJ);
            sleep(100);
            brateCleste("open");
            sleep(100);


            // Take last cone and park
            drive.followTrajectory(MidJToCones3);
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

            if(POSITION == "left") {
                drive.followTrajectory(MidJtoPos11);
                drive.followTrajectory(MidJToPos12);
            }
            else if(POSITION == "mid")
                drive.followTrajectory(MidJToPos2);
            else if(POSITION == "right")
                drive.followTrajectory(MidJToPos3);


            /** Transfer last pose to TeleOP */
            TransferPose.currentPose = drive.getPoseEstimate();
//            JSONObject j = new JSONObject();
//            try {
//                j.put("x", drive.getPoseEstimate().getX());
//                j.put("y", drive.getPoseEstimate().getY());
//                j.put("heading", drive.getPoseEstimate().getHeading());
//            } catch (JSONException e) {
//                e.printStackTrace();
//            }
//            try(FileWriter file = new FileWriter("PoseEstimate.json")){
//                file.write(j.toString());
//            } catch(IOException e){
//                e.printStackTrace();
//            }
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
