package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VariableStorage {

    /** Varibile pentru facut automatizare la teleop */

    public static double PI = 3.1415;
    public double GEAR_MOTOR_40_TICKS = 1120;
    public double GEAR_MOTOR_ORBITAL20_TICKS = 537.6;
    public static double GEAR_MOTOR_GOBILDA5202_TICKS = 537.7;
    public static double WHEEL_DIAMETER_CM = 4;
    //double TICKS_PER_CM_Z = GEAR_MOTOR_40_TICKS / (WHEEL_DIAMETER_CM * PI);
    public static double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA5202_TICKS / (WHEEL_DIAMETER_CM * PI);

    public static double cOpen1=0,cOpen2=0.4,cClosed1=0.13,cClosed2=0.28;
    public static double down=0,low=25,middle=43,up=60;

    public static class ClesteConfig {
        public double cOpen1;
        public double cOpen2;
        public double cClosed1;
        public double cClosed2;
        public ClesteConfig(double cOpen1, double cOpen2, double cClosed1, double cClosed2){
            this.cOpen1 = cOpen1;
            this.cOpen2 = cOpen2;
            this.cClosed1 = cClosed1;
            this.cClosed2 = cClosed2;
        }
    }

    public static class SlideConfig {
        public double down;
        public double low;
        public double middle;
        public double up;
        public SlideConfig(double down, double low, double middle, double up){
            this.down = down;
            this.low = low;
            this.middle = middle;
            this.up = up;
        }
    }

    public VariableStorage(){}
}
