package com.example.ys.orbtest;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.PixelFormat;
import android.hardware.SensorEvent;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.SeekBar;
import android.widget.TextView;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.nio.charset.StandardCharsets;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Semaphore;

public class OrbTest extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String TAG = "OrbTest::Activity@@JAVA";
    private GLSurfaceView glSurfaceView;
    private CameraBridgeViewBase mOpenCvCameraView;
    private Timer mTimer;
    private MyTimerTask mTimerTask;
    private NumberFormat formatter;
    private TextView myTextView;
    public static double DISTANCE = 1;
    private static long count = 0;
    private imuController sensor;
    private Semaphore sync;


    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };


    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        sync = new Semaphore(1, true);
        MatrixState.set_projection_matrix(445f, 445f, 319.5f, 239.500000f, 850, 480, 0.01f, 100f);
        super.onCreate(savedInstanceState);
        //hide the status bar
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        //hide the title bar
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_orb_test);

        String filepath = "/storage/emulated/0/SLAM/Calibration/mi6.yaml";

        try {
            readFileOnLine(filepath);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            requestPermissionsForCamera();
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        SensorManager manager = (SensorManager)getSystemService(SENSOR_SERVICE);
        sensor = new imuController(this, manager);


        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);
        mOpenCvCameraView.setMaxFrameSize(640, 480);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        //OpenGL layer
        glSurfaceView = (GLSurfaceView) findViewById(R.id.glSurfaceView);
        //OpenGL ES 2.0
        glSurfaceView.setEGLContextClientVersion(2);
        //Set transparent background
        glSurfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        glSurfaceView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        final MyRender earthRender = new MyRender(this);
        glSurfaceView.setRenderer(earthRender);
        // active rendering
        glSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        glSurfaceView.setZOrderOnTop(true);
        glSurfaceView.setOnTouchListener(new View.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (event != null) {
                    // Convert touch coordinates into normalized device
                    // coordinates, keeping in mind that Android's Y
                    // coordinates are inverted.
//                    final float normalizedX = ((event.getX() / (float) v.getWidth()) * 2 - 1) * 4f;
//                    final float normalizedY = (-((event.getY() / (float) v.getHeight()) * 2 - 1)) * 1.5f;
//
//                    if (event.getAction() == MotionEvent.ACTION_DOWN) {
//                        glSurfaceView.queueEvent(new Runnable() {
//                            @Override
//                            public void run() {
//                                earthRender.handleTouchPress(
//                                        normalizedX, normalizedY);
//                            }
//                        });
//                    } else if (event.getAction() == MotionEvent.ACTION_MOVE) {
//                        glSurfaceView.queueEvent(new Runnable() {
//                            @Override
//                            public void run() {
//                                earthRender.handleTouchDrag(
//                                        normalizedX, normalizedY);
//                            }
//                        });
//                    } else if (event.getAction() == MotionEvent.ACTION_UP) {
//                        glSurfaceView.queueEvent(new Runnable() {
//                            @Override
//                            public void run() {
//                                earthRender.handleTouchUp(
//                                        normalizedX, normalizedY);
//                            }
//                        });
//                    }

                    return true;
                } else {
                    return false;
                }
            }
        });



        myTextView = (TextView) findViewById(R.id.myTextView);
        myTextView.setText("DIST: --");

        mTimer = new Timer();
        mTimerTask = new MyTimerTask();
        mTimer.schedule(mTimerTask, 1000, 1000);
        formatter = new DecimalFormat("0.00");
//        sensor.Reset();
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        glSurfaceView.onPause();
    }

    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        glSurfaceView.onResume();
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

    }

    public void onCameraViewStarted(int width, int height) {
    }

    public void onCameraViewStopped() {
    }

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    public void requestPermissionsForCamera() {
        String [] needPermissions = { Manifest.permission.CAMERA };
        int permission = ActivityCompat.checkSelfPermission(OrbTest.this, Manifest.permission.CAMERA);
        if (permission != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(OrbTest.this, needPermissions, 1);
        } else {
            System.out.println("Camera is enable");
        }
        return;
    }

//    private native float[] CVTest(long matAddr); //Calling c++ code
    private native float[] CVTest(long matAddr, long rtAddr);

    /**
     * The function that processes the image.
     * This function is called once every time the camera refreshes,
     * and each input parameter is the current camera view information.
     * * @param inputFrame
     * @return
     */
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        Mat rgb = inputFrame.rgba();
        try {
            sync.acquire();


            Mat RT = sensor.GetSensorData();

            float[] poseMatrix = CVTest(rgb.getNativeObjAddr(), RT.getNativeObjAddr()); //Slam system obtains camera pose matrix
            if (poseMatrix.length > 1) {
                double[][] pose = new double[4][4];

                for (int i = 0; i < poseMatrix.length / 4; i++) {
                    for (int j = 0; j < 4; j++) {

                        if (j == 3 && i != 3) {
                            pose[i][j] = poseMatrix[i * 4 + j];
                        } else {
                            pose[i][j] = poseMatrix[i * 4 + j];
                        }
                    }
                }

                DISTANCE = poseMatrix[16];
//                myTextView.setText("DIST: " + DISTANCE + "m");
            }

//            if (poseMatrix.length != 0) {
//                double[][] pose = new double[4][4];
//                System.out.println("one posematrix is below========");
//                for (int i = 0; i < poseMatrix.length / 4; i++) {
//                    for (int j = 0; j < 4; j++) {
//
//                        if (j == 3 && i != 3) {
//                            pose[i][j] = poseMatrix[i * 4 + j] * SCALE;
//                        } else {
//                            pose[i][j] = poseMatrix[i * 4 + j];
//                        }
//                        System.out.print(pose[i][j] + "\t ");
//                    }
//
//                    System.out.print("\n");
//                }
//
//                System.out.println("Total number: " + count + "frame, the scaling factor is " + SCALE);
//                double[][] R = new double[3][3];
//                double[] T = new double[3];
//
//                for (int i = 0; i < 3; i++) {
//                    for (int j = 0; j < 3; j++) {
//                        R[i][j] = pose[i][j];
//                    }
//                }
//                for (int i = 0; i < 3; i++) {
//                    T[i] = pose[i][3];
//                }
//                RealMatrix rotation = new Array2DRowRealMatrix(R);
//                RealMatrix translation = new Array2DRowRealMatrix(T);
//                MatrixState.set_model_view_matrix(rotation, translation);
//
//                MyRender.flag = true;
//                count++;
//
//            } else {
//                //If you don't get the pose matrix of the camera, don't draw the cube
//                MyRender.flag = false;
//            }
            sync.release();

        } catch (InterruptedException err) {
            System.out.println(err.getMessage());
        }
        return rgb;
    }


    void printMatrix(RealMatrix input) {
        double matrixtoarray[][] = input.getData();
        for (int i = 0; i < matrixtoarray.length; i++) {
            for (int j = 0; j < matrixtoarray[0].length; j++) {
                System.out.print(matrixtoarray[i][j] + "\t");
            }
            System.out.print("\n");
        }
    }


    /**
     * read file content with request permission
     **/
    void readFileOnLine(String strFileName) throws Exception {

        int REQUEST_EXTERNAL_STORAGE = 1;
        String[] PERMISSIONS_STORAGE = {
                Manifest.permission.READ_EXTERNAL_STORAGE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
        };
        int permission = ActivityCompat.checkSelfPermission(OrbTest.this, Manifest.permission.WRITE_EXTERNAL_STORAGE);

        if (permission != PackageManager.PERMISSION_GRANTED) {
            // We don't have permission so prompt the user
            ActivityCompat.requestPermissions(
                    OrbTest.this,
                    PERMISSIONS_STORAGE,
                    REQUEST_EXTERNAL_STORAGE
            );
        }
        FileInputStream fis = new FileInputStream(new File(strFileName));
        BufferedReader reader;
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.KITKAT) {
            reader = new BufferedReader(new InputStreamReader(fis, StandardCharsets.UTF_8));
        } else {
            reader = new BufferedReader(new InputStreamReader(fis));
        }

        String strLine = null;
        while ((strLine = reader.readLine()) != null) {
            Log.i(TAG, strLine + "||");
        }
        reader.close();
        fis.close();
    }

    class MyTimerTask extends TimerTask {

        @Override
        public void run() {
            runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    boolean state = sensor.Ready();
                    String str = "State: ";
                    if (state) {
                        str += "OK";
                    } else {
                        str += "Calibr";
                    }
                    str += ";";
                    if (DISTANCE != 1.0)
                        myTextView.setText(str + "DIST: " + formatter.format(DISTANCE) + "m");
                    else
                        myTextView.setText(str + "DIST: --");
                }
            });
        }
    }
}
