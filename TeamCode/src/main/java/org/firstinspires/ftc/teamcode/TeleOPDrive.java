package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.json.JSONException;
import org.json.JSONObject;

import java.sql.Time;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@TeleOp (name = "TeleOPDrive")
public class TeleOPDrive extends LinearOpMode {

    static public class ICirclePoints {
        // Object used to store array values.
        double x,y;
        public ICirclePoints(double x, double y){
            this.x = x;
            this.y = y;
        }
    }

    final int NUMPOINTS = 40; //number of array slots
    final double RADIUS = 100d; //radius of robot
    ICirclePoints[] CirclePoints = new ICirclePoints[NUMPOINTS];

    //Global values.
    boolean isMoving=false, isInBounds=true, isOverridden=false;
    boolean isOpen=true;
    double joystick_r_y, joystick_r_x, joystick_l_y, joystick_l_x, xStart=0, yStart=0, headingStart=0;
    int liftCountZ=1;

    //Transfer autonomous pose to TeleOP. Parse JSON file then store it into json object.
    Pose2d startPose;
    public void initPose(){
        try {
            JSONObject obj = new JSONObject("PoseEstimate.json");
            xStart = obj.getInt("x");
            yStart = obj.getInt("y");
            headingStart = obj.getInt("heading");
            startPose = new Pose2d(xStart, yStart, headingStart);
        } catch (JSONException e){
            e.printStackTrace();
        }
    }

    hardwarePapiu robot = new hardwarePapiu();
    //Minimap test = new Minimap();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        /** Initialise pose **/
        //initPose();
        drive.setPoseEstimate(TransferPose.currentPose);

        /** Default motor values / Reset encoders **/
        robot.bratz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bratz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.servoLeft.setPosition(0);
        robot.servoRight.setPosition(0.4);

        waitForStart();
        while(!isStopRequested()){
            /** variables **/

            joystick_l_y = gamepad2.left_stick_y;
            joystick_r_x = gamepad2.right_stick_x;
            joystick_r_y = gamepad2.right_stick_y;
            joystick_l_x = gamepad2.left_stick_x;
            float joystic_r_trigger = gamepad2.right_trigger+1;
            isMoving= joystick_l_x != 0 && joystick_r_y != 0 && joystick_l_y != 0 && joystick_r_x != 0;
            /** Base Movement**/

            /// Check if you're too close to a junction
//            if((generatePoints(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY()) && isInBounds) || isOverridden){

                /// Limit speed while left bumper is pressed
                if(joystic_r_trigger>1){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -joystick_l_y/(1.5*joystic_r_trigger),
                                    -joystick_l_x/(1.5*joystic_r_trigger),
                                    -joystick_r_x/(5*joystic_r_trigger)
                            )
                    );
                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -joystick_l_y/1.2,
                                    -joystick_l_x/1.2,
                                    -joystick_r_x/1.2
                            )
                    );
                }
            drive.update();
//            } else telemetry.addLine("you stepped on a junction idiot");
            if(gamepad2.dpad_up)
                moveBrat("up");

            if(gamepad2.dpad_down)
                moveBrat("down");

            if(gamepad2.y)
                brateCleste();



            telemetry.addData("lift level bratz: ", liftCountZ);
//            telemetry.addData("isOverriden: ", isOverridden);
//            telemetry.addData("IsMoving: ", isMoving);
//            telemetry.addData("isInBounds: ", isInBounds);
            telemetry.addData("cleste deschis:", isOpen);
            telemetry.addData("Cleste position left:", robot.servoLeft.getPosition());
            telemetry.addData("Cleste position right:", robot.servoRight.getPosition());
            telemetry.addData("Glisiera ticks: ", robot.bratz.getCurrentPosition());
            telemetry.addData("Glisiera target: ", robot.bratz.getTargetPosition());
            telemetry.addData("Trigger value: ", joystic_r_trigger);
            telemetry.update();
        }
    }
    /** Varibile pentru facut automatizare la teleop */

    double PI = 3.1415;
    double GEAR_MOTOR_40_TICKS = 1120;
    double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

    public void moveBrat(String direction){
        if(Objects.equals(direction, "up")){
            try {
                // Limit movement so that it doesn't go above max height

                if (liftCountZ < 4) {
                    runToPosition(liftCountZ, "up");
                }
                else liftCountZ=4;

                TimeUnit.MILLISECONDS.sleep(500);
            } catch(InterruptedException e){
                Thread.currentThread().interrupt();
            }
        } else if(Objects.equals(direction, "down")){
            try {
                // Limit movement so that it doesn't go below min height
                if (liftCountZ > 1) {
                    runToPosition(liftCountZ, "down");
                }
                else {
                    liftCountZ=1;
                }
                TimeUnit.MILLISECONDS.sleep(500);
            } catch (InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
    }
    public void brateCleste() {
        try {
            isOpen=!isOpen;
            if(isOpen){ //pt deschis
                robot.servoLeft.setPosition(0);
                robot.servoRight.setPosition(0.4);
            }
            else{ //pt inchis
                robot.servoLeft.setPosition(0.07);
                robot.servoRight.setPosition(0.32);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void runToPosition(int level, String rotation) {
        int ticks;
        if(rotation == "up"){
            liftCountZ++;
            level = liftCountZ;
            switch(level){
                case 2:
                    ticks = (int)(24 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
                case 3:
                    ticks = (int)(18 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
                case 4:
                    ticks = (int)(16 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() - ticks);
                    break;
            }
        } else if(rotation == "down"){
            liftCountZ--;
            level = liftCountZ;
            switch(level){
                case 1:
                    robot.bratz.setTargetPosition(0);
                    break;
                case 2:
                    ticks = (int)(18 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() + ticks);
                    break;
                case 3:
                    ticks = (int)(16 * TICKS_PER_CM_Z);
                    robot.bratz.setTargetPosition(robot.bratz.getCurrentPosition() + ticks);
                    break;
            }
        }
        robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bratz.setPower(0.6);
    }
    boolean generatePoints(double x, double y){
        for(int i = 0; i < NUMPOINTS; ++i){
            final double angle = Math.toRadians(((double) i / NUMPOINTS) * 360d);
            CirclePoints[i] = new ICirclePoints(x, y);
            if(!checkMat(CirclePoints[i].x, CirclePoints[i].y)){
                if(!isMoving){
                    isInBounds=true;
                    isOverridden =true;
                }
                else isInBounds=false;
            } else{
                isInBounds=true;
                isOverridden =false;
            }
        }
        return true;
    }
    boolean checkMat(double x, double y){
        double mat = 23.5;
        if(x <=3.5 && x>=-3.5 && y<=3.5 && y>=-3.5) return false; //ground mid
        else if(x <=(mat+3.5) && x>=(mat-3.5) && y<=3.5 && y>=3.5) return false; //high right
        else if(x <=(2*mat+3.5) && x>=(2*mat-3.5) && y<=3.5 && y>=-3.5) return false; // ground right
        else if(x <=(-mat+3.5) && x>=(-mat-3.5) && y<=3.5 && y>=-3.5) return false; // high left
        else if(x <=(-2*mat+3.5) && x>=(-2*mat-3.5) && y<=3.5 && y>=-3.5) return false; // ground left
        else if(x <=3.5 && x>=-3.5 && y<=(mat+3.5) && y>=(mat-3.5)) return false; // high top
        else if(x <=3.5 && x>=-3.5 && y<=(2*mat+3.5) && y>=(2*mat-3.5)) return false; // ground top
        else if(x <=3.5 && x>=-3.5 && y<=(-mat+3.5) && y>=(-mat-3.5)) return false; // high bottom
        else if(x <=3.5 && x>=-3.5 && y<=(-2*mat+3.5) && y>=(-2*mat-3.5)) return false; // ground bottom
        //todo: continue here

        return true;
    }
}
