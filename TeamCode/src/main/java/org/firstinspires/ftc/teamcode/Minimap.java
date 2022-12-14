package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.json.JSONException;
import org.json.JSONObject;

//TODO: NOT TESTED, 99% won't work

public class Minimap extends AutoMain1{
    //TODO: get current position
    // when pressing A, enter automode
    // select with Y, X, A, B desired junction
    // head to junction and lift glisiera asyncron
    // relenquish control after the glisiera has reached required level and robot is at 0 momentum
    double positionX, positionY, squareLength = 24, goToX, goToY, cx, cy, precision=3;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public void minimap() {
        positionX = drive.getPoseEstimate().getX();
        positionY = drive.getPoseEstimate().getY();   //-Aici pt coordonate

        if(positionX < squareLength)
            cx = squareLength / positionX;
        else cx = positionX / squareLength;

        if(positionY < squareLength)
            cy = squareLength / positionY;
        else cy = positionY / squareLength;

        if(gamepad1.y){                 /**up**/
            if(positionY < 0) {
                if (positionY > -squareLength)
                    goToY = 0 + precision;
                if (positionY < -squareLength)
                    goToY = squareLength * (cy - 1) + precision;
            }
            else{
                if (positionY < squareLength)
                    goToY = squareLength + 3;
                if (positionY > squareLength)
                    goToY = squareLength * (cy + 1) + precision;
            }
            if(gamepad1.x) {             /**left**/
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = - squareLength + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * (cx - 1) + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = 0 + precision;
                    if(positionX > squareLength)
                        goToX = cx * squareLength + precision;
                }
                Pose2d StartPos = new Pose2d(goToX, goToX, Math.toRadians(90));
                drive.setPoseEstimate(StartPos);
                Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
                        .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(135)))
                        .addDisplacementMarker(0, ()->{
                            if(goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
                                if(goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
                                    runToPosition(1, "up");

                            if(goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
                                    goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                    runToPosition(2, "up");

                            if(goToX == -squareLength + precision || goToX == squareLength + precision)
                                if(goToY == -squareLength + precision || goToY == squareLength + precision)
                                    runToPosition(3, "up");

                            if(goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(4, "up");
                        })
                        .build();
                drive.followTrajectory(GoToJunction);
            }
            if(gamepad1.b){             /**right**/
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = 0 + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * cx + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = squareLength + precision;
                    if(positionX > squareLength)
                        goToX = squareLength * (cx + 1) + precision;
                }

                Trajectory GoToJunction = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(45)))
                        .addDisplacementMarker(0, ()->{
                            if(goToX == -2 * squareLength + precision || goToX == precision || goToX == 5 * squareLength + precision)
                                if(goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
                                    runToPosition(1, "up");

                            if(goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
                                    goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(2, "up");

                            if(goToX == -squareLength + precision || goToX == squareLength + precision)
                                if(goToY == -squareLength + precision || goToY == squareLength + precision)
                                    runToPosition(3, "up");

                            if(goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(4, "up");
                        })
                        .build();
                drive.followTrajectory(GoToJunction);
            }
        }
        if(gamepad1.a){                 /**down**/
            if(positionY < 0) {
                if (positionY > -squareLength)
                    goToY = -squareLength + precision;
                if (positionY < -squareLength)
                    goToY = squareLength * (cy - 1) + precision;
            }
            else{
                if (positionY < squareLength)
                    goToY = squareLength + precision;
                if (positionY > squareLength)
                    goToY = squareLength * (cy + 1) + precision;
            }
            if(gamepad1.x){             /**left**/
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = - squareLength + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * (cx - 1) + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = 0 + precision;
                    if(positionX > squareLength)
                        goToX = cx * squareLength + precision;
                }

                Pose2d StartPos = new Pose2d(goToX, goToY, Math.toRadians(-90));       // || 270?
                drive.setPoseEstimate(StartPos);
                Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
                        .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(-135)))
                        .addDisplacementMarker(0, ()->{
                            if(goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
                                if(goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
                                    runToPosition(1, "up");

                            if(goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
                                    goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(2, "up");

                            if(goToX == -squareLength + precision || goToX == squareLength + precision)
                                if(goToY == -squareLength + precision || goToY == squareLength + precision)
                                    runToPosition(3, "up");

                            if(goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(4, "up");
                        })
                        .build();
                drive.followTrajectory(GoToJunction);
            }
            if(gamepad1.b){             /**right**/
                if(positionX < 0) {
                    if (positionX > -squareLength)
                        goToX = 0 + precision;
                    if (positionX < -squareLength)
                        goToX = squareLength * cx + precision;
                }
                else{
                    if(positionX < squareLength)
                        goToX = squareLength + precision;
                    if(positionX > squareLength)
                        goToX = squareLength * (cx + 1) + precision;
                }
                Pose2d StartPos = new Pose2d(goToX, goToY, Math.toRadians(-90));
                drive.setPoseEstimate(StartPos);
                Trajectory GoToJunction = drive.trajectoryBuilder(StartPos)
                        .lineToLinearHeading(new Pose2d(goToX, goToY, Math.toRadians(-45)))
                        .addDisplacementMarker(0, ()->{
                            if(goToX == -2 * squareLength + precision || goToX == precision || goToX == 2 * squareLength + precision)
                                if(goToY == -2 * squareLength + precision || goToY == precision || goToY == 2 * squareLength + precision)
                                    runToPosition(1, "up");

                            if(goToX == -2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision) || goToX == -squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) ||
                                    goToX == squareLength + precision && (goToY == -2 * squareLength + precision || goToY == 2 * squareLength + precision) || goToX == 2 * squareLength + precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(2, "up");

                            if(goToX == -squareLength + precision || goToX == squareLength + precision)
                                if(goToY == -squareLength + precision || goToY == squareLength + precision)
                                    runToPosition(3, "up");

                            if(goToY == precision && (goToX == -squareLength + precision || goToX == squareLength + precision) || goToX == precision && (goToY == -squareLength + precision || goToY == squareLength + precision))
                                runToPosition(4, "up");
                        })
                        .build();
                drive.followTrajectory(GoToJunction);
            }
        }
    }
}