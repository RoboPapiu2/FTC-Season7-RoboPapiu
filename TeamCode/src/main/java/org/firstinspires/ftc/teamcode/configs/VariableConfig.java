package org.firstinspires.ftc.teamcode.configs;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.hardwarePapiu;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Config
public class VariableConfig{
    //TODO: INVOKING ON NULL OBJECT REFERENCE, NEED INIT BUT CANT. solutie where

    hardwarePapiu robot = new hardwarePapiu();

    public void init(HardwareMap ahwmap){
        robot.init(ahwmap);
    }

    public VariableConfig(){}

    //Global values.
    boolean isOpen=false;

    public static VariableStorage.ClesteConfig cleste = new VariableStorage.ClesteConfig(VariableStorage.cOpen1 ,VariableStorage.cOpen2 ,
                                                        VariableStorage.cClosed1 ,VariableStorage.cClosed2);

    public static VariableStorage.SlideConfig slide = new VariableStorage.SlideConfig(VariableStorage.down, VariableStorage.low,
                                                        VariableStorage.middle,VariableStorage.up);

    public void brateCleste() {
        try {
            isOpen=!isOpen;
            if(isOpen){ //pt deschis
                robot.servoLeft.setPosition(cleste.cOpen1);
                robot.servoRight.setPosition(cleste.cOpen2);
            }
            else{ //pt inchis
                robot.servoLeft.setPosition(cleste.cClosed1);
                robot.servoRight.setPosition(cleste.cClosed2);
            }
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e){
            Thread.currentThread().interrupt();
        }
    }
    public void moveBratSus(String direction, int a){
        if(Objects.equals(direction, "up")){
            int ticks = (int)(slide.up * VariableStorage.TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks+ a);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "down")){
            robot.bratz.setTargetPosition((int)(slide.down * VariableStorage.TICKS_PER_CM_Z)+a);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "middle")){
            int ticks = (int)(slide.middle * VariableStorage.TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks+a);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(1);
        } else if(Objects.equals(direction, "low")){
            int ticks = (int)(slide.low * VariableStorage.TICKS_PER_CM_Z);
            robot.bratz.setTargetPosition(-ticks+a);
            robot.bratz.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bratz.setPower(0.6);
        }
    }
}
