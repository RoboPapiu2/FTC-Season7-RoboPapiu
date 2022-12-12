package org.firstinspires.ftc.teamcode;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class detectionpipline extends OpenCvPipeline
{
    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;
    //marimi
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public detectionpipline(double tagsize, double fx, double fy, double cx, double cy)
    {
        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize()
    {
        if(nativeApriltagPtr != 0)
        {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("am terminat he he he haw");
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // pune in greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // dai run la apriltag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        //marker pe imagine
        for(AprilTagDetection detection : detections)
        {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //constructia matriciei

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * MArker 3D
     *
     * buf bufferul pe care desenam markerul
     *  length lungimea markerului
     *  rvec vectorul de rotire pentru deterctare
     *  tvec vectorul de traducere pentru detectare
     *  cameraMatrix  matricia care este folosita sa gaseasca detectorul
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // punctele din 3D care le vrem proiectate pe o imagine 2D
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // pune punctele
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // deseneaza markerul
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //Punctele 3D pe care le vrem in 2D
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // proiecteaza puncetele acestea
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // linie de sus(amogus)
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     *  points punctul in care desenam trapezoidul
     *  cameraMatrix matricia camerei
     *  tagsizeX lungimea tagului
     *  tagsizeY inaltimea tagului
     *  pozitia 6DOF a camerei relativa cu tagul
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        //punctele 2d a tagului
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        //punctele 3D ale tagului
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        //pozitia cu punctele 3D si 2D
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}