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
public class TestAutoAnything extends LinearOpMode {
    hardwarePapiu robot = new hardwarePapiu();
    Trajectory PushCone1;
    Trajectory PushCone2;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.EncoderReset();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        Pose2d StartBottom = new Pose2d(35.5, -61, Math.toRadians(90));
        drive.setPoseEstimate(StartBottom);

        Trajectory StartToLow = drive.trajectoryBuilder(StartBottom) //TODO: sugiuc
                .lineToLinearHeading(new Pose2d(27, -52, Math.toRadians(135)),
                        // Limit speed of trajectory
                        SampleMecanumDrive.getVelocityConstraint(26, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0,()->{
                    runToPosition(2, "up");
                })
                .addDisplacementMarker(()->drive.followTrajectory(PushCone1))
                .build();
        PushCone1 = drive.trajectoryBuilder(StartToLow.end())
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
                .build();

        if (isStopRequested()) return;

        drive.followTrajectory(StartToLow);
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
        robot.bratz.setPower(0.8);
    }
    public void brateCleste(String state) {
        if(state == "open"){ //pt deschis
            robot.servoLeft.setPosition(0);
            robot.servoRight.setPosition(0.4);
        }
        else if(state == "closed"){ //pt inchis
            robot.servoLeft.setPosition(0.12);
            robot.servoRight.setPosition(0.3);
        }
    }
}
