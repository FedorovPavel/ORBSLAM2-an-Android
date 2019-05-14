package com.example.ys.orbtest;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import android.widget.FrameLayout;
import android.content.Context;

import org.opencv.core.Mat;

import static org.opencv.core.CvType.CV_32F;


public class imuController extends FrameLayout implements SensorEventListener {

    private SensorManager manager;
    private Sensor mGyroscope;
    private Sensor mAccelerometer;
    private Sensor mMagnetrometer;

    private AHRS Quaternion;

    private boolean AccelUpdate;
    private float[] AccelData;
    private boolean GyroUpdate;
    private float[] GyroData;
    private boolean MagnetUpdate;
    private float[] MagnetData;

    public imuController(Context context, SensorManager extendsManager) {
        super(context);

        manager = extendsManager;

        mGyroscope = manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAccelerometer = manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetrometer = manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);


        AccelData = new float[3];
        GyroData = new float[3];
        MagnetData = new float[3];

        Start();
        return;
    }

    public Mat GetSensorData() {
        Mat result = new Mat(3,4, CV_32F);

        Mat r = getRotationMatrix();
        Mat t = getTranslationMatrix();

        r.copyTo(result.rowRange(0,3).colRange(0,3));
        t.copyTo(result.rowRange(0,3).col(3));

        return result;
    }

    private Mat getRotationMatrix() {
        Mat res = new Mat(3,3,CV_32F);
        return res;
    }

    private Mat getTranslationMatrix() {
        Mat res = new Mat(3,1,CV_32F);
        return res;
    }


    public void Start() {
        manager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mMagnetrometer, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void Stop() {
        manager.unregisterListener(this);
    }

    private void flushSensorData() {
        AccelUpdate = false;
        GyroUpdate = false;
        MagnetUpdate = false;
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            AccelData[0] = event.values[0];
            AccelData[1] = event.values[1];
            AccelData[2] = event.values[2];
            AccelUpdate = true;
        }
        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            GyroData[0] = event.values[0];
            GyroData[1] = event.values[1];
            GyroData[2] = event.values[2];
            GyroUpdate = true;
        }
        else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            MagnetData[0] = event.values[0];
            MagnetData[1] = event.values[1];
            MagnetData[2] = event.values[2];
            MagnetUpdate = true;
        }

        if (AccelUpdate && GyroUpdate && MagnetUpdate) {
            Quaternion.Update(AccelData, GyroData, MagnetData);
            flushSensorData();
        }
        return;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public class GyroData {

        private float _x;
        private float _y;
        private float _z;
        private float _w;
        private float _time;

        public float getX(){
            return this._x;
        }

        public float setX(float x) {
            this._x = x;
            return this._x;
        }

        public float getY() {
            return this._y;
        }

        public float setY(float y) {
            this._y = y;
            return this._y;
        }

        public float getZ() {
            return this._z;
        }

        public float setZ(float z) {
            this._z = z;
            return this._z;
        }

        public float getW() {
            return this._z;
        }

        public float setW(float w) {
            this._w = w;
            return this._w;
        }

        public float getTime() {
            return this._time;
        }

        public GyroData(float x, float y, float z, float w, float time) {
            this._x = x;
            this._y = y;
            this._z = z;
            this._w = w;
            this._time = time;
        }
    }

    public class AHRS {
        public float SamplePeriod;
        public float Beta;
        public float[] Quaternion;

        public AHRS(float period, float beta) {
            this.Beta = beta;
            this.SamplePeriod = period;
            this.Quaternion = new float[4];
            Reset();
        }

        public boolean Reset() {
            this.Quaternion[0] = 1;
            this.Quaternion[1] = 0;
            this.Quaternion[2] = 0;
            this.Quaternion[3] = 0;
            return true;
        }

        public void Update(float[] a, float[] g, float[] m) {

        }
    }
}
