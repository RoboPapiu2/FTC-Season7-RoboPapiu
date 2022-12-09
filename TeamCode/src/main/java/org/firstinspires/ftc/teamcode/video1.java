package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class video1 extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    String locationString;
    public String location;

    static final Rect ROI = new Rect(
            new Point(0, 90),
            new Point(40, 180));

    static double PERCENT_COLOR_TRESH = 0.2;

    public video1(Telemetry t){
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input){

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //CULUARE 1 albastru

        Scalar lowHSV1 = new Scalar(100,150,0);
        Scalar highHSV1 = new Scalar(140,255,255);

        Core.inRange(mat,lowHSV1,highHSV1,mat);

        Mat submat1 = mat.submat(ROI);
        double cevaValue1 = Core.sumElems(submat1).val[0] / ROI.area()/255;
        submat1.release();
        telemetry.addData("valoare1", (int) Core.sumElems(submat1).val[0]);
        telemetry.addData("procent ceva1", Math.round(cevaValue1 * 100)+"%");
        boolean ceva1 = cevaValue1 > PERCENT_COLOR_TRESH;


        //CULUARE 2 sper ca ii verde CA NU POT TESTA
        Scalar lowHSV2 = new Scalar(36, 25, 25);
        Scalar highHSV2 = new Scalar(70, 255,255);

        Core.inRange(mat,lowHSV2,highHSV2,mat);

        Mat submat2 = mat.submat(ROI);
        double cevaValue2 = Core.sumElems(submat1).val[0] / ROI.area()/255;
        telemetry.addData("valoare2", (int) Core.sumElems(submat1).val[0]);
        telemetry.addData("procent ceva2", Math.round(cevaValue2 * 100)+"%");
        submat1.release();
        boolean ceva2 = cevaValue2> PERCENT_COLOR_TRESH;


        //CULUARE 3 sper ca ii rosu TE IMPLOR CA NU MAI VREAU SA CAUT
        Scalar lowHSV3 = new Scalar(161,155,84);
        Scalar highHSV3 = new Scalar(179,255,255);

        Core.inRange(mat,lowHSV3,highHSV3,mat);//1

        Mat submat3 = mat.submat(ROI);
        double cevaValue3= Core.sumElems(submat3).val[0] / ROI.area()/255;
        submat1.release();
        telemetry.addData("valoare3", (int) Core.sumElems(submat3).val[0]);
        telemetry.addData("procent ceva3", Math.round(cevaValue3 * 100)+"%");
        boolean ceva3 = cevaValue3 > PERCENT_COLOR_TRESH;

        telemetry.update();
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, ROI, new Scalar(0, 255, 0));

        return mat;

    }

}
