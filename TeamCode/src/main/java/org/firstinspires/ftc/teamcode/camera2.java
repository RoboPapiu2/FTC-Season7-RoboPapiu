//package org.firstinspires.ftc.teamcode;
//
//import org.opencv.calib3d.Calib3d;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfDouble;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.MatOfPoint3f;
//import org.opencv.core.Point;
//import org.opencv.core.Point3;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.apriltag.AprilTagDetectorJNI;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//
//class camera2 extends OpenCvPipeline
//{
//    private long nativeApriltagPtr;
//    private Mat grey = new Mat();
//    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
//
//    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
//    private final Object detectionsUpdateSync = new Object();
//
//    Mat cameraMatrix;
//
//    Scalar blue = new Scalar(7,197,235,255);
//    Scalar red = new Scalar(255,0,0,255);
//    Scalar green = new Scalar(0,255,0,255);
//    Scalar white = new Scalar(255,255,255,255);
//
//    double fx;
//    double fy;
//    double cx;
//    double cy;
//
//    // UNITS ARE METERS
//    double tagsize;
//    double tagsizeX;
//    double tagsizeY;
//
//    private float decimation;
//    private boolean needToSetDecimation;
//    private final Object decimationSync = new Object();
//
//    public camera2(double tagsize, double fx, double fy, double cx, double cy)
//    {
//        this.tagsize = tagsize;
//        this.tagsizeX = tagsize;
//        this.tagsizeY = tagsize;
//        this.fx = fx;
//        this.fy = fy;
//        this.cx = cx;
//        this.cy = cy;
//
//        constructMatrix();
//
//        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
//    }
//
//    @Override
//    public void finalize()
//    {
//        if(nativeApriltagPtr != 0)
//        {
//            // Delete the native context we created in the constructor
//            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
//            nativeApriltagPtr = 0;
//        }
//        else
//        {
//            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
//        }
//    }
//
//    @Override
//    public Mat processFrame(Mat input)
//    {
//        // Convert to greyscale
//        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
//
//        synchronized (decimationSync)
//        {
//            if(needToSetDecimation)
//            {
//                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
//                needToSetDecimation = false;
//            }
//        }
//
//        // Run AprilTag
//        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
//
//        synchronized (detectionsUpdateSync)
//        {
//            detectionsUpdate = detections;
//        }
//
//        for(AprilTagDetection detection : detections)
//        {
//            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
//            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
//            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
//        }
//
//        return input;
//    }
//
//    public void setDecimation(float decimation)
//    {
//        synchronized (decimationSync)
//        {
//            this.decimation = decimation;
//            needToSetDecimation = true;
//        }
//    }
//
//    public ArrayList<AprilTagDetection> getLatestDetections()
//    {
//        return detections;
//    }
//
//    public ArrayList<AprilTagDetection> getDetectionsUpdate()
//    {
//        synchronized (detectionsUpdateSync)
//        {
//            ArrayList<AprilTagDetection> ret = detectionsUpdate;
//            detectionsUpdate = null;
//            return ret;
//        }
//    }
//
//    void constructMatrix()
//    {
//
//        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);
//
//        cameraMatrix.put(0,0, fx);
//        cameraMatrix.put(0,1,0);
//        cameraMatrix.put(0,2, cx);
//
//        cameraMatrix.put(1,0,0);
//        cameraMatrix.put(1,1,fy);
//        cameraMatrix.put(1,2,cy);
//
//        cameraMatrix.put(2, 0, 0);
//        cameraMatrix.put(2,1,0);
//        cameraMatrix.put(2,2,1);
//    }
//