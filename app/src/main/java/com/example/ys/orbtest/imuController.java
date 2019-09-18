package com.example.ys.orbtest;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import android.os.Environment;
import android.widget.FrameLayout;
import android.content.Context;

import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.io.IOException;
import com.example.ys.orbtest.FiltFilt;

import java.util.Arrays;
import java.util.concurrent.Semaphore;

import edu.mines.jtk.dsp.ButterworthFilter;
import edu.mines.jtk.sgl.Axis;
import uk.me.berndporr.iirj.*;
import edu.mines.jtk.*;

import static org.opencv.core.CvType.CV_32F;

public class imuController extends FrameLayout implements SensorEventListener {

    private double MaxAcc;
    private double MaxGyr;
    private double MaxMag;
    private double MaxAccLin;
    private double AccNoise;
    private double GyrNoise;
    private double MagNoise;
    private double AccLinNoise;
    private SensorManager manager;
    private Sensor mGyroscope;
    private Sensor mAccelerometer;
    private Sensor mMagnetrometer;
    private Sensor mLinearAccelerometer;
    private Semaphore lockAcc;
    private Semaphore lockGyr;
    private Semaphore lockMag;
    private Semaphore lockAccLin;
    private static int AXIS = 3;

    private AHRS Quaternion;
    private MotionSensor Position;

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
        mLinearAccelerometer = manager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        Quaternion = new AHRS(0.1f);
        Position = new MotionSensor();

        AccelData = new float[3];
        GyroData = new float[3];
        MagnetData = new float[3];

        lockAcc = new Semaphore(1, true);
        lockGyr = new Semaphore(1, true);
        lockMag = new Semaphore(1, true);
        lockAccLin = new Semaphore(1, true);

        //  Set phone param
        MaxAcc = 40.0;
        MaxGyr = 60.0;
        MaxMag = 360;
        MaxAccLin = 20;


        flushSensorData();

        Start();
        return;
    }

    public Matrix GetTranslate() {
        Matrix res = new Matrix(3,1, getTranslationMatrix());
        return res;
    }

    public Mat GetSensorData() {
        Mat result = new Mat(3,4, CV_32F);

        float[][] r = getRotationMatrix();
        float[] t = getTranslationMatrix();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.put(i,j, r[i][j]);
            }
        }

        for (int i = 0; i < 3; i++) {
            result.put(i, 3, t[i]);
        }

        return result;
    }

    private float[][] getRotationMatrix() {
        float[][] res = new float[3][3];

        float [] matrix = Quaternion.GetRotation();
        int i, j;
        for (int k = 0; k < matrix.length; k++) {
            j = k % 3;
            i = k / 3;
            res[i][j] =  matrix[k];
        }
        return res;
    }

    private float[] getTranslationMatrix() {
        float[] res = new float[3];
        float[] data = Position.GetPosition();
        for (int i = 0; i < data.length; i++) {
            res[i] = data[i];
        }

        return res;
    }


    public void Start() {
        manager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mMagnetrometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mLinearAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public boolean Ready() {
        return Position.ready;
    }

    public void Stop() {
        manager.unregisterListener(this);
    }

    private void flushSensorData() {
        AccelUpdate = false;
        GyroUpdate = false;
        MagnetUpdate = false;
    }

    public boolean checkCorrectnessSensorData(float[] data, double max) {
        for (int i = 0; i < AXIS; i++) {
            if (Math.abs(data[i]) > max) {
                return false;
            }
        }
        return true;
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        try {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER: {
                    lockAcc.acquire();
                    if (!checkCorrectnessSensorData(event.values, MaxAcc)) {
                        lockAcc.release();
                        return;
                    }
                    AccelData[0] = SensorDataHandler.GetAccuracy(-event.values[1],8);
                    AccelData[1] = SensorDataHandler.GetAccuracy(event.values[0],8);
                    AccelData[2] = SensorDataHandler.GetAccuracy(event.values[2],8);
                    AccelUpdate = true;
                    lockAcc.release();
                    return;
                }
                case Sensor.TYPE_GYROSCOPE: {
                    lockGyr.acquire();
                    if (!checkCorrectnessSensorData(event.values, MaxGyr)) {
                        lockGyr.release();
                        return;
                    }
                    GyroData[0] = SensorDataHandler.GetAccuracy(-event.values[1], 8);
                    GyroData[1] = SensorDataHandler.GetAccuracy(event.values[0], 8);
                    GyroData[2] = SensorDataHandler.GetAccuracy(-event.values[2], 8);
                    GyroUpdate = true;
                    lockGyr.release();
                    return;
                }
                case Sensor.TYPE_MAGNETIC_FIELD: {
                    lockMag.acquire();
                    if (!checkCorrectnessSensorData(event.values, MaxMag)) {
                        lockMag.release();
                        return;
                    }
                    MagnetData[0] = SensorDataHandler.GetAccuracy(-event.values[1],8);
                    MagnetData[1] = SensorDataHandler.GetAccuracy(event.values[0], 8);
                    MagnetData[2] = SensorDataHandler.GetAccuracy(event.values[2], 8);
                    MagnetUpdate = true;
                    lockMag.release();
                    return;
                }
                case Sensor.TYPE_LINEAR_ACCELERATION: {
                    lockAccLin.acquire();
                    if (!checkCorrectnessSensorData(event.values, MaxAccLin)) {
                        lockAccLin.release();
                        return;
                    }
                    lockAcc.acquire();
                    lockGyr.acquire();
                    lockMag.acquire();
                    float[] al = new float[AXIS];
                    al[0] = SensorDataHandler.GetAccuracy(-event.values[1], 8) + 0.000129f;
                    al[1] = SensorDataHandler.GetAccuracy(event.values[0], 8) + 0.000111f;
                    al[2] = SensorDataHandler.GetAccuracy(event.values[2], 8) + 0.01395f;

                    Position.Update(al, AccelData, GyroData, MagnetData, event.timestamp);
                    Quaternion.Update(AccelData, GyroData, event.timestamp);
                    lockMag.release();
                    lockGyr.release();
                    lockAcc.release();
                    lockAccLin.release();
                    return;
                }
            }
        }catch (Exception err) {
            System.out.println(err.getMessage());
        }
        return;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public static class SensorDataHandler {

        static float GetAccuracy(float val, int deg) {
            float res;
            float fraction;
            if (val < 0.0f) {
                res = (float)Math.ceil(val);
                fraction = val - res;
                fraction *= (Math.pow(10, deg));
                fraction = (float) Math.ceil(fraction);
                fraction /= Math.pow(10, deg);
                res += fraction;
            }
            else {
                res = (float)Math.floor(val);
                fraction = val - res;
                fraction *= Math.pow(10, deg);
                fraction = (float) Math.floor(fraction);
                fraction /= Math.pow(10, deg);
                res += fraction;
            }
            return res;
        }

        static double Mean(double[] m) {
            float sum = 0;
            for (int i = 0; i < m.length; i++) {
                sum += m[i];
            }
            return sum / m.length;
        }

        static float Mean(ArrayList<float[]> m, int index) {
            float sum = 0;
            for (int i = 0; i < m.size(); i++) {
                sum += m.get(i)[index];
            }
            return sum / m.size();
        }
    }

    public class AHRS {
        public long prevTime;
        public float Beta;
        public float[] Quaternion;
        private Semaphore lock;
        private float[] accelLastData;
        private float[] initError;
        private float Ki;

        public AHRS(float beta) {
            this.Beta = beta;
            this.Quaternion = new float[4];
            accelLastData = new float[4];
            initError = new float[3];
            Ki = 1;
            lock = new Semaphore(1);
            Reset();
        }

        private boolean Reset() {
            this.Quaternion[0] = 1;
            this.Quaternion[1] = 0;
            this.Quaternion[2] = 0;
            this.Quaternion[3] = 0;

            for (int i = 0; i < initError.length; i++) {
                initError[i] = 0;
            }
            return true;
        }

        public float[] GetRotation() {
            float [] rmatrix = new float[9];
            try {
                lock.acquire();

                float w = Quaternion[0];
                float x = Quaternion[1];
                float y = Quaternion[2];
                float z = Quaternion[3];

                rmatrix[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
                rmatrix[1] = 2.0f * x * y - 2.0f * z * w;
                rmatrix[2] = 2.0f * x * z + 2.0f * y * w;

                rmatrix[3] = 2.0f * x * y + 2.0f * z * w;
                rmatrix[4] = 1.0f - 2.0f * x * x - 2.0f * z * z;
                rmatrix[5] = 2.0f * y * z - 2.0f * x * w;

                rmatrix[6] = 2.0f * x * z - 2.0f * y * w;
                rmatrix[7] = 2.0f * y * z + 2.0f * x * w;
                rmatrix[8] = 1.0f - 2.0f * x * x - 2.0f * y * y;
//                Reset();
                lock.release();
            } catch (InterruptedException err) {}


            return rmatrix;
        }

        public void Update(float[] a, float[] g, float[] m, long time) {
            if (prevTime == 0) {
                for (int i = 0; i < a.length;i++)
                    accelLastData[i] = a[i];
                prevTime = time;
                return;
            }
            try {
                lock.acquire();
                if (time - prevTime < 5000) {
                    lock.release();
                    return;
                }
                float ax,ay,az;
                float gx,gy,gz;
                float mx,my,mz;
                ax = a[0]; ay = a[1]; az = a[2];
                gx = g[0]; gy = g[1]; gz = g[2];
                mx = m[0]; my = m[1]; mz = m[2];
                if (Math.abs(accelLastData[0] + accelLastData[1] + accelLastData[2] - ax -ay -az) > 8.0f) {
                    //  Data ejection
                    prevTime = time;
                    lock.release();
                    return;
                }

                float q1 = Quaternion[0];
                float q2 = Quaternion[1];
                float q3 = Quaternion[2];
                float q4 = Quaternion[3];

                float SamplePeriod = (time - prevTime) / 1000000000.0f;     // ns into s
                if (SamplePeriod > 1.0f) {
                    prevTime = time;
                    lock.release();
                    return;
                }
                SamplePeriod += 0.0000000001f;
                float norm;
                float hx, hy, _2bx, _2bz;
                float s1, s2, s3, s4;
                float qDot1, qDot2, qDot3, qDot4;

                // Auxiliary variables to avoid repeated arithmetic
                float _2q1mx;
                float _2q1my;
                float _2q1mz;
                float _2q2mx;
                float _4bx;
                float _4bz;
                float _2q1 = 2f * q1;
                float _2q2 = 2f * q2;
                float _2q3 = 2f * q3;
                float _2q4 = 2f * q4;
                float _2q1q3 = 2f * q1 * q3;
                float _2q3q4 = 2f * q3 * q4;
                float q1q1 = q1 * q1;
                float q1q2 = q1 * q2;
                float q1q3 = q1 * q3;
                float q1q4 = q1 * q4;
                float q2q2 = q2 * q2;
                float q2q3 = q2 * q3;
                float q2q4 = q2 * q4;
                float q3q3 = q3 * q3;
                float q3q4 = q3 * q4;
                float q4q4 = q4 * q4;

                // Normalise accelerometer measurement
                norm = (float)Math.sqrt(ax * ax + ay * ay + az * az);
                if (norm == 0f) {
                    prevTime = time;
                    lock.release();
                    return;
                }
                norm = 1 / norm;        // use reciprocal for division
                a[0] *= norm;
                a[1] *= norm;
                a[2] *= norm;

                // Normalise magnetometer measurement
                norm = (float)Math.sqrt(mx * mx + my * my + mz * mz);
                if (norm == 0f) {
                    prevTime = time;
                    lock.release();
                    return;
                }
                norm = 1 / norm;        // use reciprocal for division
                m[0] *= norm;
                m[1] *= norm;
                m[2] *= norm;

                // Reference direction of Earth's magnetic field
                _2q1mx = 2f * q1 * mx;
                _2q1my = 2f * q1 * my;
                _2q1mz = 2f * q1 * mz;
                _2q2mx = 2f * q2 * mx;
                hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
                        + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
                hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
                        + my * q3q3 + _2q3 * mz * q4 - my * q4q4;

                _2bx = (float) Math.sqrt(hx * hx + hy * hy);
                _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
                        + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
                _4bx = 2f * _2bx;
                _4bz = 2f * _2bz;

                // Gradient decent algorithm corrective step
                s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay)
                        - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay)
                        - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az)
                        + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay)
                        - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az)
                        + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay)
                        + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                norm = (float)Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude

                if (norm == 0f) {
                    prevTime = time;
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                s1 *= norm;
                s2 *= norm;
                s3 *= norm;
                s4 *= norm;

                // Compute rate of change of quaternion
                qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
                qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
                qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
                qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

                // Integrate to yield quaternion
                q1 += qDot1 * SamplePeriod;
                q2 += qDot2 * SamplePeriod;
                q3 += qDot3 * SamplePeriod;
                q4 += qDot4 * SamplePeriod;
                norm = (float)Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
                if (norm == 0f) {
                    prevTime = time;
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                Quaternion[0] = q1 * norm;
                Quaternion[1] = q2 * norm;
                Quaternion[2] = q3 * norm;
                Quaternion[3] = q4 * norm;

                prevTime = time;
                lock.release();
            } catch (InterruptedException err) {}
            return;
        }

        public void Update(float[] a, float[] g, float[] m, double dt, double Kp) {

            //  Normalise accelerometer measurement
            float ax,ay,az;
            float gx,gy,gz;
            float mx,my,mz;
            ax = a[0]; ay = a[1]; az = a[2];
            gx = g[0]; gy = g[1]; gz = g[2];
            mx = m[0]; my = m[1]; mz = m[2];

            float q1 = Quaternion[0];
            float q2 = Quaternion[1];
            float q3 = Quaternion[2];
            float q4 = Quaternion[3];

            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2f * q1;
            float _2q2 = 2f * q2;
            float _2q3 = 2f * q3;
            float _2q4 = 2f * q4;
            float _2q1q3 = 2f * q1 * q3;
            float _2q3q4 = 2f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = (float)Math.sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0f) {
                return;
            }

            norm = 1 / norm;        // use reciprocal for division
            a[0] *= norm;
            a[1] *= norm;
            a[2] *= norm;

            // Normalise magnetometer measurement
            norm = (float)Math.sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0f) {
                return;
            }

            norm = 1 / norm;        // use reciprocal for division
            m[0] *= norm;
            m[1] *= norm;
            m[2] *= norm;

            //  Compute error between estimated and measured direction of gravity
            float[] v = new float[3];
            v[0] = 2*q2*q4 - q1*q3;
            v[1] = 2*q1*q2 + q3*q4;
            v[2] = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;               	// estimated direction of gravity
            float[] error = cross(v, a);

            // compute integral feedback terms (only outside of init period)
            for (int i = 0; i < initError.length; i++) {
                initError[i] += error[i];
            }

            // Apply feedback terms
            float[] ref = new float[3];
            for (int i = 0; i < ref.length; i++) {
                ref[i] = (float)(g[i] - (Kp*error[i] +  Ki*initError[i]));
            }

            // Compute rate of change of quaternion
            float[] vectorOfRef = new float[4];
            vectorOfRef[0] = 0;
            for (int i = 0; i < ref.length; i++) {
                vectorOfRef[i+1] = ref[i];
            }
            float[] s = MultipleQuaternion(Quaternion.clone(), vectorOfRef);
            for (int i = 0; i < s.length; i++) {
                s[i] *= 0.5f;
            }
            q1 += s[0] * dt;
            q2 += s[1] * dt;
            q3 += s[2] * dt;
            q4 += s[3] * dt;

            norm = (float)Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            if (norm == 0f) {
                prevTime = (long)dt;
                lock.release();
                return;
            }
            norm = 1f / norm;
            Quaternion[0] = q1 * norm;
            Quaternion[1] = q2 * norm;
            Quaternion[2] = q3 * norm;
            Quaternion[3] = q4 * norm;

            try {
                lock.acquire();
                if (dt < 5000) {
                    lock.release();
                    return;
                }
                ax = a[0]; ay = a[1]; az = a[2];
                gx = g[0]; gy = g[1]; gz = g[2];
                mx = m[0]; my = m[1]; mz = m[2];
                if (Math.abs(accelLastData[0] + accelLastData[1] + accelLastData[2] - ax -ay -az) > 8.0f) {
                    //  Data ejection
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }


                float SamplePeriod = (float)dt;     // ns into s
                if (SamplePeriod > 1.0f) {
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }
                SamplePeriod += 0.0000000001f;

                // Normalise accelerometer measurement
                norm = (float)Math.sqrt(ax * ax + ay * ay + az * az);
                if (norm == 0f) {
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }
                norm = 1 / norm;        // use reciprocal for division
                a[0] *= norm;
                a[1] *= norm;
                a[2] *= norm;

                // Normalise magnetometer measurement
                norm = (float)Math.sqrt(mx * mx + my * my + mz * mz);
                if (norm == 0f) {
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }
                norm = 1 / norm;        // use reciprocal for division
                m[0] *= norm;
                m[1] *= norm;
                m[2] *= norm;

                // Reference direction of Earth's magnetic field
                _2q1mx = 2f * q1 * mx;
                _2q1my = 2f * q1 * my;
                _2q1mz = 2f * q1 * mz;
                _2q2mx = 2f * q2 * mx;
                hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
                        + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
                hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
                        + my * q3q3 + _2q3 * mz * q4 - my * q4q4;

                _2bx = (float) Math.sqrt(hx * hx + hy * hy);
                _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
                        + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
                _4bx = 2f * _2bx;
                _4bz = 2f * _2bz;

                // Gradient decent algorithm corrective step
                s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay)
                        - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay)
                        - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az)
                        + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay)
                        - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az)
                        + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay)
                        + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
                        + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
                        + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
                norm = (float)Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude

                if (norm == 0f) {
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                s1 *= norm;
                s2 *= norm;
                s3 *= norm;
                s4 *= norm;

                // Compute rate of change of quaternion
                qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
                qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
                qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
                qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

                // Integrate to yield quaternion
                q1 += qDot1 * SamplePeriod;
                q2 += qDot2 * SamplePeriod;
                q3 += qDot3 * SamplePeriod;
                q4 += qDot4 * SamplePeriod;
                norm = (float)Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
                if (norm == 0f) {
                    prevTime = (long)dt;
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                Quaternion[0] = q1 * norm;
                Quaternion[1] = q2 * norm;
                Quaternion[2] = q3 * norm;
                Quaternion[3] = q4 * norm;

                prevTime = (long)dt;
                lock.release();
            } catch (InterruptedException err) {}
            return;
        }

        public void Update(float[] a, float[] g, long time) {
            if (prevTime == 0) {
                for (int i = 0; i < a.length;i++)
                    accelLastData[i] = a[i];
                prevTime = time;
                return;
            }
            try {
                lock.acquire();
                if (time - prevTime < 5000) {
                    lock.release();
                    return;
                }
                float ax,ay,az;
                float gx,gy,gz;
                ax = a[0]; ay = a[1]; az = a[2];
                gx = g[0]; gy = g[1]; gz = g[2];

                float q1 = Quaternion[0], q2 = Quaternion[1], q3 = Quaternion[2], q4 = Quaternion[3];   // short name local variable for readability
                float norm;
                float s1, s2, s3, s4;
                float qDot1, qDot2, qDot3, qDot4;

                // Auxiliary variables to avoid repeated arithmetic
                float _2q1 = 2f * q1;
                float _2q2 = 2f * q2;
                float _2q3 = 2f * q3;
                float _2q4 = 2f * q4;
                float _4q1 = 4f * q1;
                float _4q2 = 4f * q2;
                float _4q3 = 4f * q3;
                float _8q2 = 8f * q2;
                float _8q3 = 8f * q3;
                float q1q1 = q1 * q1;
                float q2q2 = q2 * q2;
                float q3q3 = q3 * q3;
                float q4q4 = q4 * q4;
                float SamplePeriod = (time - prevTime) / 1000000000.0f;     // ns into s

                // Normalise accelerometer measurement
                norm = (float)Math.sqrt(ax * ax + ay * ay + az * az);
                if (norm == 0f) {
                    lock.release();
                    return;
                }
                norm = 1 / norm;        // use reciprocal for division
                ax *= norm;
                ay *= norm;
                az *= norm;

                // Gradient decent algorithm corrective step
                s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
                s2 = _4q2 * q4q4 - _2q4 * ax + 4f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
                s3 = 4f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
                s4 = 4f * q2q2 * q4 - _2q2 * ax + 4f * q3q3 * q4 - _2q3 * ay;
                norm = (float)Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude

                if (norm == 0f) {
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                s1 *= norm;
                s2 *= norm;
                s3 *= norm;
                s4 *= norm;

                // Compute rate of change of quaternion
                qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
                qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
                qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
                qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

                // Integrate to yield quaternion
                q1 += qDot1 * SamplePeriod;
                q2 += qDot2 * SamplePeriod;
                q3 += qDot3 * SamplePeriod;
                q4 += qDot4 * SamplePeriod;
                norm = (float)Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
                if (norm == 0f) {
                    lock.release();
                    return;
                }
                norm = 1f / norm;
                Quaternion[0] = q1 * norm;
                Quaternion[1] = q2 * norm;
                Quaternion[2] = q3 * norm;
                Quaternion[3] = q4 * norm;

                prevTime = time;
                lock.release();
            } catch (InterruptedException err) {}
        }

        public float[] Rotate(float[] vector, float[] quaternion) {
            float [] temp;
            float [] transformVector = new float[4];
            transformVector[0] = 0;
            for (int i = 1; i < transformVector.length; i++) {
                transformVector[i] = vector[i-1];
            }
            temp = MultipleQuaternion(MultipleQuaternion(quaternion, transformVector), quaternion);
            float [] result = new float[3];
            for(int i = 0; i < AXIS; i++) {
                result[i] = temp[i+1];
            }
            return result;
        }

        public float[] MultipleQuaternion(float[] q1, float[] q2) {
            float[] result = new float[4];
            result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
            result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
            result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
            result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
            return result;
        }

        private float[] cross(float[] a, float[] b) {
            float[] result = new float[3];
            result[0] = a[1] * b[2] - a[2] * b[1];
            result[1] = a[2] * b[0] - a[0] * b[2];
            result[2] = a[0] * b[1] - a[1] * b[0];
            return result;
        }

        public float[] GetVector() {
            float[] result = Quaternion.clone();
            return result;
        }
    }

    public class MotionSensor {
        private long Time;
        private long prevT;
        private float[] AccLinear;
        public boolean ready = false;

        //  noise parametrs
        private int countDisp = 1000;
        private int currentDisp;
        private float[] minInStay;
        private float[] maxInStay;

        //  Simple kalman
        private float K = 0.5f;
        private float[] prevA;
        private float[] prevPrevA;

        //  Kalman
        private ArrayList<float[]> CovAcc;
        private Kalman Kfilter;

        //  Sync
        private final Semaphore lock;   //  sync lock

        //  Output
        private float[] prevPositions;
        private float[] prevVelocity;

        //  Mean
        private int countSample = 30;
        private ArrayList<float[]> meanAcceleration;
        private float[] prevMeanAcceleration;
        private float[] prevMeanVelocity;
        private float[] prevMeanPosition;
        private int currentSample = 0;
        private ArrayList<float[]> Acc;
        private ArrayList<float[]> Gyro;
        private ArrayList<float[]> Magnetic;

        //  Old
        private Butterworth buttord;
        private ButterworthFilter butter;
        private AHRS Quaternion;
        private boolean first;
        private int pointer;
        private float[] prevAvgA;
        private float[] meanPos;
        private float[] meanVel;

        public MotionSensor(){
            lock = new Semaphore(1, false);

            AccLinear = new float[3];
            prevPositions = new float[3];
            prevVelocity = new float[3];
            currentSample = 0;

            Kfilter = new Kalman(6, 3, 6);
            CovAcc = new ArrayList<>();

            meanAcceleration = new ArrayList<>();
            prevMeanAcceleration = new float[AXIS];
            prevMeanVelocity = new float[AXIS];
            prevMeanPosition = new float[AXIS];

            prevA = new float[3];
            prevPrevA = new float[3];
            minInStay = new float[3];
            maxInStay = new float[3];
            for (int i = 0; i < AXIS; i++) {
                minInStay[i] = (float)MaxAccLin;
                maxInStay[i] = (float)-MaxAccLin;
            }

            Acc = new ArrayList<>();
            Gyro = new ArrayList<>();
            Magnetic = new ArrayList<>();
            prevPositions = new float[3];
            prevVelocity = new float[3];
            currentSample = 0;
            Quaternion = new AHRS(0.1f);


            prevAvgA = new float[3];
            meanPos = new float[3];
            meanVel = new float[3];
            prevA = new float[3];

        }

        //  Butterworth release
        private void periodUpdate() {
            //  compute accelerometer magnitude
//            float[] accelerometerMagnitudes = new float[countSample];
//            double ax;
//            double ay;
//            double az;
//            double samplePeriod = 0;
//            for (int i = 0; i < countSample; i++) {
//                ax = AccLinear.get(i)[0];
//                ay = AccLinear.get(i)[1];
//                az = AccLinear.get(i)[2];
//
//                accelerometerMagnitudes[i] = (float)(Math.sqrt(ax * ax + ay * ay * az * az));
//                if (i == 0) {
//                    samplePeriod += (double)(Time.get(i+1) - Time.get(i)) / 1000000000.0;
//                } else
//                    samplePeriod += (double)(Time.get(i) - Time.get(i-1)) / 1000000000.0;
//            }
//
//            samplePeriod /= countSample;
//            //  High-pass filter
//            double filtCutOff = 0.001;
//            butter = new ButterworthFilter((2*filtCutOff)/(1/samplePeriod), 1 ,ButterworthFilter.Type.HIGH_PASS);
//            float [] accelerometerMFiltered = new float[countSample];
//            butter.applyForward(accelerometerMagnitudes, accelerometerMFiltered);
//
//            for (int i = 0; i < countSample; i++) {
//                accelerometerMagnitudes[i] = (float)Math.abs(accelerometerMFiltered[i]);
//            }
//
//            //  Low-pass filter
//            filtCutOff = 19.6;
//            butter = new ButterworthFilter((2*filtCutOff)/(1/samplePeriod), 1, ButterworthFilter.Type.LOW_PASS);
//            butter.applyForward(accelerometerMagnitudes, accelerometerMFiltered);
//
//            //  stationary states
//            boolean[] stationary = new boolean[countSample];
//            for (int i = 0; i < countSample; i++) {
//                if (accelerometerMFiltered[i] < 0.05) {
//                    stationary[i] = true;
//                    Quaternion.Update(Acc.get(i), Gyro.get(i),Time.get(i));
//                } else {
//                    stationary[i] = false;
//                    Quaternion.Update(Acc.get(i), Gyro.get(i), Time.get(i));
//                }
//
//                float[] matrix = Quaternion.GetRotation();
//                Matrix RM = new Matrix(3,3,matrix);
//                Matrix AccM = new Matrix(1, 3, AccLinear.get(i));
//                RM = Matrix.Invert(RM);
//                RM = Matrix.Multiply(RM, AccM);
//                AccLinear.set(i, RM.GetMatrixAsVector());
//            }
//
//            ArrayList<float[]> velocity = new ArrayList<>();
//
//            //  save velocity by past calculating
//            float[] item = new float[AXIS];
//            for (int i = 0; i < AXIS; i++) {
//                item[i] = prevVelocity[i];
//            }
//            velocity.add(item);
//
//            double dt;
//            for(int i = 1; i < countSample; i++) {
//                item = new float[AXIS];
//                dt = ((double)(Time.get(i) - Time.get(i-1))) / 1000000000.0;
//
//                if (stationary[i]) {
//                    item[0] = 0;
//                    item[1] = 0;
//                    item[2] = 0;
//                } else {
//                    item[0] = (float) (velocity.get(i - 1)[0] + ((AccLinear.get(i)[0] + AccLinear.get(i - 1)[0]) / 2) * dt);
//                    item[1] = (float) (velocity.get(i - 1)[1] + ((AccLinear.get(i)[1] + AccLinear.get(i - 1)[1]) / 2) * dt);
//                    item[2] = (float) (velocity.get(i - 1)[2] + ((AccLinear.get(i)[2] + AccLinear.get(i - 1)[2]) / 2) * dt);
//                }
//
//                velocity.add(item);
//
//            }
//
//            ArrayList<float[]> positions = new ArrayList<>();
//            item = new float[AXIS];
//            for (int i = 0; i < AXIS; i++) {
//                item[i] = prevPositions[i];
//            }
//            positions.add(item);
//
//            for (int i = 1; i < countSample; i++) {
//                item = new float[AXIS];
//                dt = ((double)(Time.get(i) - Time.get(i-1))) / 1000000000.0;
//                item[0] = (float)(positions.get(i - 1)[0] + ((velocity.get(i)[0] + velocity.get(i-1)[0])/2) * dt);
//                item[1] = (float)(positions.get(i - 1)[1] + ((velocity.get(i)[1] + velocity.get(i-1)[1])/2) * dt);
//                item[2] = (float)(positions.get(i - 1)[2] + ((velocity.get(i)[2] + velocity.get(i-1)[2])/2) * dt);
//                positions.add(item);
//            }
//
//            prevPositions = positions.get(countSample - 1).clone();
//            prevVelocity = velocity.get(countSample - 1).clone();
            return;
        }

        //  Kalman release
        private void periodUpdate2() {

            try {
                float dt = (float) ((Time - prevT) / 1000000000.0);
                if (dt > 1.0f) {
                    prevT = Time;
                    return;
                }
                Quaternion.Update(Acc.get(0), Gyro.get(0), Time);
                Matrix R = new Matrix(3,3, Quaternion.GetRotation());
                Matrix A = new Matrix(3, 1, AccLinear);
                Matrix AR = Matrix.Multiply(R,A);
                for (int i = 0; i < AXIS; i++) {
                    AccLinear[i] = AR.data[i][0];
                }

                //  accumulate error of acceleration
                if (currentDisp < countDisp) {

                    //  find min and max noise in stay state
                    for (int i = 0; i < AXIS; i++) {
                        if (AccLinear[i] > maxInStay[i]) {
                            maxInStay[i] = AccLinear[i];
                        }
                        if (AccLinear[i] < minInStay[i]) {
                            minInStay[i] = AccLinear[i];
                        }
                    }


                    CovAcc.add(AccLinear);
                    prevT = Time;
                    currentDisp++;
                    prevPrevA = prevA.clone();
                    prevA = AccLinear.clone();
                    return;
                }

                //  Mean initing
                if (currentSample < countSample) {
                    meanAcceleration.add(AccLinear.clone());
                    currentSample++;
                    prevPrevA = prevA.clone();
                    prevA.clone();
                    return;
                }

                meanAcceleration.add(AccLinear.clone());
                float[] currentMeanAcceleration = new float[3];
                for (int i = 0; i < AXIS; i++) {
                    currentMeanAcceleration[i] = SensorDataHandler.Mean(meanAcceleration, i);
                }
                meanAcceleration.remove(0);

                if (currentSample == countSample) {
                    prevMeanAcceleration = currentMeanAcceleration.clone();
                    currentSample++;
                    prevPrevA = prevA.clone();
                    prevA = AccLinear.clone();
                    return;
                }

                float[] currentMeanVelocity = new float[3];
                for (int i = 0; i < AXIS; i++) {
                    if (AccLinear[i] < maxInStay[i] && AccLinear[i] > minInStay[i]) {
                        currentMeanVelocity[i] = 0;
                    } else {
                        currentMeanVelocity[i] = prevMeanVelocity[i] + (prevMeanAcceleration[i] + currentMeanAcceleration[i]) / 2 * dt;
                    }
                }

                if (currentSample < countSample + 1) {
                    prevMeanVelocity = currentMeanVelocity.clone();
                    prevMeanAcceleration = currentMeanAcceleration.clone();
                    currentSample++;
                    prevPrevA = prevA.clone();
                    prevA = AccLinear.clone();
                    return;
                }

                float[] currentMeanPosition = new float[3];
                for (int i = 0; i < AXIS; i++) {
                    currentMeanPosition[i] = prevMeanPosition[i] + (prevMeanVelocity[i] + currentMeanVelocity[i]) / 2 * dt;
                }
                prevMeanAcceleration = currentMeanAcceleration.clone();
                prevMeanVelocity = currentMeanVelocity.clone();

                //  deviation
                CovAcc.add(AccLinear);
                float[] avg = new float[3];
                for (int i = 0; i < CovAcc.size(); i++) {
                    for (int j = 0; j < AXIS; j++) {
                        avg[j] += CovAcc.get(i)[j];
                    }
                }

                for (int i = 0; i < AXIS; i++) {
                    avg[i] /= CovAcc.size();
                }


                float[] dev = new float[3];
                for (int i = 0; i < AXIS; i++) {
                    float s = 0;
                    for (int j = 0; j < CovAcc.size(); j++) {
                        s += (float) Math.pow((CovAcc.get(j)[i] - avg[i]), 2);
                    }

                    dev[i] = (float) Math.sqrt(s / CovAcc.size());
                }

                CovAcc.remove(0);

                //  first predict
                Kfilter.rebuildF(dt);
                Kfilter.rebuildB(dt);
                float[] u = new float[AccLinear.length];
                for (int i = 0; i < u.length; i++) {
                    if ((prevPrevA[i] - prevA[i] > 0) && (prevA[i] - AccLinear[i]) > 0 )
                        u[i] = AccLinear[i] + prevA[i];
                    else
                        u[i] = AccLinear[i] + prevA[i];
                }

                Kfilter.rebuildU(u);
                Kfilter.rebuildQ(dev, dt);

                Kfilter.Predict();

                Kfilter.Xk = Kfilter.Xk_k1.clone();
                //  Predict Pos;
                Matrix pPos = Kfilter.Xk_k1.clone();

                //  Update with Mean method
                Matrix measure = pPos.clone();
                float vector = (float)Math.sqrt(Math.pow(AccLinear[0], 2) + Math.pow(AccLinear[1], 2) + Math.pow(AccLinear[2], 2));
                for (int i = 0; i < AXIS; i++) {
                    if (vector < maxInStay[i] && vector > minInStay[i]) {
                        measure.data[i][0] = prevPositions[i];
                        measure.data[i+AXIS][0] = 0.00f;
                    } else {
                        measure.data[i][0] = pPos.data[i][0];
                        measure.data[i+AXIS][0] = pPos.data[i+AXIS][0];
                    }
                }

                float posDev = (float) Math.sqrt(dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2]);
                Kfilter.rebuildZ(measure);
                Kfilter.rebuildR(posDev);

                Kfilter.Update();

                prevPositions[0] = Kfilter.Xk.data[0][0];
                prevPositions[1] = Kfilter.Xk.data[1][0];
                prevPositions[2] = Kfilter.Xk.data[2][0];


                prevMeanPosition = prevPositions.clone();
                prevT = Time;
                prevPrevA = prevA.clone();
                prevA = AccLinear.clone();
                ready = true;
                // v1.0
//                float dt = (float) ((Time.get(0) - prevT) / 1000000000.0);
//                if (dt > 1.0f) {
//                    prevT = Time.get(0);
//                    return;
//                }
//                Quaternion.Update(Acc.get(0), Gyro.get(0), Time.get(0));
//
//                //              Get A_North, A_East, A_Altitude
////                float[] matrix = Quaternion.GetRotation();
////                Matrix RM = new Matrix(3, 3, matrix);
//                Matrix AccM = new Matrix(3, 1, AccLinear.get(0));
////                RM = Matrix.Invert(RM);
////                AccM = Matrix.Multiply(RM, AccM);
////                AccLinear.set(0, AccM.GetMatrixAsVector());
//                float [] accK = new float[3];
//                for (int i = 0; i < AXIS; i++) {
//                    accK[i] = K * AccLinear.get(0)[i] + (1 - K) * prevA[i];
//                }
//                prevA = accK.clone();
//
//                if (currentDisp < countDisp) {
//                    float[] item = new float[3];
////                    for (int i = 0; i < item.length; i++) {
////                        item[i] = AccLinear.get(0)[i];
////                    }
//
//                    CovAcc.add(accK);
//                    prevT = Time.get(0);
//                    currentDisp++;
//                    if (currentDisp == countDisp) {
//                        for (int i = 0; i < CovAcc.size(); i++) {
//                            for (int j = 0; j < AXIS; j++) {
//                                prevAvgA[j] += CovAcc.get(i)[j];
//                            }
//                        }
//
//                        for (int i = 0; i < AXIS; i++) {
//                            prevAvgA[i] /= CovAcc.size();
//                        }
//
//                    }
//                    return;
//                }
//
//                //  deviation
//                CovAcc.add(accK);
//                float[] avg = new float[3];
//                for (int i = 0; i < CovAcc.size(); i++) {
//                    for (int j = 0; j < AXIS; j++) {
//                        avg[j] += CovAcc.get(i)[j];
//                    }
//                }
//
//                for (int i = 0; i < AXIS; i++) {
//                    avg[i] /= CovAcc.size();
//                }
//
//
//                float[] dev = new float[3];
//                for (int i = 0; i < AXIS; i++) {
//                    float s = 0;
//                    for (int j = 0; j < CovAcc.size(); j++) {
//                        s += (float) Math.pow((CovAcc.get(j)[i] - avg[i]), 2);
//                    }
//
//                    dev[i] = -(float) Math.sqrt(s / CovAcc.size());
//                }
//
//                CovAcc.remove(0);
//                //            SS.add(dev);
//
//                //  first predict
//                Kfilter.rebuildF(dt);
//                Kfilter.rebuildB(dt);
//                Kfilter.rebuildU(accK);
//                Kfilter.rebuildQ(dev, dt);
//
//                Kfilter.Predict();
//
//                Kfilter.Xk = Kfilter.Xk_k1.clone();
//                //  Predict Pos;
//                Matrix pPos = Kfilter.Xk_k1.clone();
//
//                //  Update
//                float posDev = (float) Math.sqrt(dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2]);
//                Matrix measure = pPos.clone();
//                for (int i = 0; i < AXIS; i++) {
//                    measure.data[i+AXIS][0] = (float)(meanVel[i] + (prevAvgA[i] + avg[i])/2.0 * dt);
//                    measure.data[i][0] = (float)(meanPos[i] + (meanVel[i] + measure.data[i+AXIS][0])/ 2.0 * dt);
//                }
//
//                if (Math.sqrt(Math.pow(AccLinear.get(0)[0], 2) + Math.pow(AccLinear.get(0)[1], 2) + Math.pow(AccLinear.get(0)[2], 2)) < 0.005) {
//                    measure.data[3][0] = 0.0f;
//                    measure.data[4][0] = 0.0f;
//                    measure.data[5][0] = 0.0f;
//                }
//
//
//                Kfilter.rebuildZ(measure);
//                Kfilter.rebuildR(posDev);
//
//                Kfilter.Update();
//                prevPositions[0] = Kfilter.Xk.data[0][0];
//                prevPositions[1] = Kfilter.Xk.data[1][0];
//                prevPositions[2] = Kfilter.Xk.data[2][0];
//                for (int i = 0; i < AXIS; i++) {
//                    meanVel[i] = measure.data[i+AXIS][0];
//                    meanPos[i] = prevPositions[i];
//                }
//                prevAvgA = avg.clone();
//                prevT = Time.get(0);
            } catch (Exception err) {
                System.out.print(err.getMessage());
            }
        }

        public void Update(float[] al, float[] a, float[] g, float[] m, long time) {
            try {
                lock.acquire();
                //  copy AL, A, G, M to SampleArray
                float[] temp = new float[3];
                AccLinear = al.clone();

                temp = new float[3];
                for (int i = 0; i < a.length; i++) {
                    temp[i] = a[i];
                }
                Acc.add(temp);

                temp = new float[3];
                for (int i = 0; i < g.length; i++) {
                    temp[i] = g[i];
                }
                Gyro.add(temp);

                temp = new float[3];
                for (int i = 0; i < m.length; i++) {
                    temp[i] = m[i];
                }
                Magnetic.add(temp);

                Time = time;

                periodUpdate2();

                Acc.clear();
                Gyro.clear();
                Magnetic.clear();
                lock.release();

                return;

//                pointer++;
//                if (pointer > 1000 && pointer < 10000) {
//                    try {
//                        TS.add(Time.get(0));
//                        ALS.add(AccLinear.get(0));
//                        QS.add(Quaternion.Quaternion);
//                        float [] item = new float[3];
//                        item[0] = Kfilter.Xk.data[0][0];
//                        item[1] = Kfilter.Xk.data[1][0];
//                        item[2] = Kfilter.Xk.data[2][0];
//                        PKS.add(item);
//
//                        item = new float[3];
//                        item[0] = Kfilter.Xk.data[3][0];
//                        item[1] = Kfilter.Xk.data[4][0];
//                        item[2] = Kfilter.Xk.data[5][0];
//                        VKS.add(item);
//
//                        item = new float[3];
//                        item[0] = Kfilter.Xk_k1.data[0][0];
//                        item[1] = Kfilter.Xk_k1.data[1][0];
//                        item[2] = Kfilter.Xk_k1.data[2][0];
//                        PK1S.add(item);
//
//                        item = new float[3];
//                        item[0] = Kfilter.Xk_k1.data[3][0];
//                        item[1] = Kfilter.Xk_k1.data[4][0];
//                        item[2] = Kfilter.Xk_k1.data[5][0];
//                        VK1S.add(item);
//
//                    }catch (Exception err) {
//                        System.out.print("OK");
//                    }
//                } else if (pointer == 10000) {
//                    File file;
//                    try {
//                        file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "T.txt");
//                        FS = new FileOutputStream(file, true);
//                        String string;
//                        for (int j = 0; j < TS.size(); j++) {
//                            string = TS.get(j) + ",";
//                            string += ALS.get(j)[0] + "," + ALS.get(j)[1] + "," + ALS.get(j)[2] + ",";
//                            string += QS.get(j)[0] + "," + QS.get(j)[1] + "," + QS.get(j)[2] + "," + QS.get(j)[3] + ",";
//                            string += PKS.get(j)[0] + "," + PKS.get(j)[1] + "," + PKS.get(j)[2] + ",";
//                            string += VKS.get(j)[0] + "," + VKS.get(j)[1] + "," + VKS.get(j)[2] + ",";
//                            string += PK1S.get(j)[0] + "," + PK1S.get(j)[1] + "," + PK1S.get(j)[2] + ",";
//                            string += VK1S.get(j)[0] + "," + VK1S.get(j)[1] + "," + VK1S.get(j)[2] + ",";
//                            string += SS.get(j)[0] + "," + SS.get(j)[1] + "," + SS.get(j)[2] + "\n";
//
//                            FS.write(string.getBytes());
//                        }
//
//                        FS.close();
//                    }catch (Exception err) {
//                        System.out.print(err.getMessage());
//                    }
//                } else if (pointer > 10000) {
//                    prevPositions[0] = 10000;
//                }
//                AccLinear.clear();
//                Acc.clear();
//                Gyro.clear();
//                Magnetic.clear();
//                Time.clear();
//                lock.release();
//                return;
//                System.out.println("|A|[" + ax + "," + ay + "," + az + ","+ Double.toString(dt) + "]");
//                if (dt > 1.0) {
//                    prevTime = time;
//                    lock.release();
//                    return;
//                }
//
//                prevAX[pointer] = ax;
//                prevAY[pointer] = ay;
//                prevAZ[pointer] = az;
//
//                //  Compute accelerometer magnitude
//                ArrayList<Double> accMagnitudes = new ArrayList<Double>();
//                for (int i = 0; i < pointer; i++) {
//                    accMagnitudes.add(Math.sqrt(prevAX[i] * prevAX[i] + prevAY[i]* prevAY[i] + prevAZ[i] * prevAZ[i]));
//                }
//                float freq = 0.1f;
//
                //  HP filter accelerometer data
//                butter.highPass(1, dt, (2 * freq/ (1/dt)));
//                Biquad coef = butter.getBiquad(0);
//                ArrayList<Double> B = new ArrayList<>(3);
//                B.add(coef.getB0());
//                B.add(coef.getB1());
//                B.add(coef.getB2());
//
//                ArrayList<Double> A = new ArrayList<>(3);
//                A.add(coef.getA0());
//                A.add(coef.getA1());
//                A.add(coef.getA2());
//
//                ArrayList<Double> accMagnitudesFiltered = FiltFilt.doFiltfilt(B, A, accMagnitudes);
//
//                //  Get absolute value
//                for (int i = 0; i < accMagnitudesFiltered.size(); i++) {
//                    accMagnitudesFiltered.set(i, Math.abs(accMagnitudesFiltered.get(i)));
//                }
//
//                //  LP
//                freq = 5.5f;
//                butter.lowPass(1, dt, (2* freq /(1/dt)));
//                coef = butter.getBiquad(0);
//                B.clear();
//                B.add(coef.getB0());
//                B.add(coef.getB1());
//                B.add(coef.getB2());
//
//                A.clear();
//                A.add(coef.getA0());
//                A.add(coef.getA1());
//                A.add(coef.getA2());
//
//                accMagnitudes.clear();
//                accMagnitudes.addAll(accMagnitudesFiltered);
//                accMagnitudesFiltered = FiltFilt.doFiltfilt(B, A, accMagnitudes);

//                int[] stationary = new int[sample - 1];
//                for (int i = 0; i < sample - 1; i++) {
//                    if (accMagnitudes.get(i) < 0.05) {
//                        stationary[i] = 1;
//                    } else {
//                        stationary[i] = 0;
//                    }
//                }

//                if (cur < sample) {
//                    prevAvgA[0] = SensorDataHandler.Mean(prevAX);
//                    prevAvgA[1] = SensorDataHandler.Mean(prevAY);
//                    prevAvgA[2] = SensorDataHandler.Mean(prevAZ);
//                    cur++;
//                    Swap();
//                    prevTime = time;
//                    lock.release();
//                    return;
//                }


//                double[] cA = new double[3];
//                cA[0] = SensorDataHandler.Mean(prevAX);
//                cA[1] = SensorDataHandler.Mean(prevAY);
//                cA[2] = SensorDataHandler.Mean(prevAZ);
//                Swap();

//                double[] velocity = new double[3];
//                if (stationary[sample - 2] == 1) {
//                    for (int i = 0; i < 3; i++) {
//                        velocity[i] = 0;
//                    }
//                } else {
//                    velocity[0] = prevV[0] + (prevAX[sample-2] + prevAX[sample-3])/2*dt;
//                    velocity[1] = prevV[1] + (prevAY[sample-2] + prevAY[sample-3])/2*dt;
//                    velocity[2] = prevV[2] + (prevAZ[sample-2] + prevAZ[sample-3])/2*dt;
//                }
//
//

//                if (cur == sample) {
//                    for (int i = 0; i < 3; i ++) {
//                        prevV[i] = velocity[i];
//                    }
//                    cur++;
//                    prevTime = time;
//                    lock.release();
//                    return;
//                }

//                prevAvgA = cA;

//                for (int i = 0; i < velocity.length; i++) {
//                    position[i] += (velocity[i] + prevV[i]) / 2 * dt;
//                }
//                prevV = velocity;
//                prevTime = time;
//                lock.release();
            }catch (Exception err) {
                System.out.print(err.getMessage());
            }
        }

        public float[] GetAccelerationByWorldCoordinateSystem(float[] RM, float[] acceleration) {
            float [] result = new float[AXIS];
            result[0] = RM[0] * acceleration[0] + RM[1] * acceleration[1] + RM[2] * acceleration[2];
            result[1] = RM[3] * acceleration[0] + RM[4] * acceleration[1] + RM[5] * acceleration[2];
            result[2] = RM[6] * acceleration[0] + RM[7] * acceleration[1] + RM[8] * acceleration[2];

            return result;
        }

        public float[] GetPosition() {
            float[] res = new float[3];
            try {
                lock.acquire();
                for(int i = 0; i < 3; i++)
                    res[i] = prevPositions[i];
                lock.release();

//                Reset();
            } catch (Exception err) {
                System.out.print(err.getMessage());
            }
            return res;
        }
    }
}
