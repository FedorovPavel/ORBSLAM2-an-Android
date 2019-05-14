package com.example.ys.orbtest;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import android.widget.FrameLayout;
import android.content.Context;

import org.opencv.core.Mat;

import java.util.concurrent.Semaphore;

import static org.opencv.core.CvType.CV_32F;


public class imuController extends FrameLayout implements SensorEventListener {

    private SensorManager manager;
    private Sensor mGyroscope;
    private Sensor mAccelerometer;
    private Sensor mMagnetrometer;
    private Sensor mLinearAccelerometer;

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

        Quaternion = new AHRS(1f / 256f, 0.1f);
        Position = new MotionSensor();

        AccelData = new float[3];
        GyroData = new float[3];
        MagnetData = new float[3];

        flushSensorData();

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

        float [] matrix = Quaternion.GetRotation();
        int i, j;
        for (int k = 0; k < matrix.length; k++) {
            j = k % 3;
            i = k / 3;
            res.put(i, j, matrix[k]);
        }
        return res;
    }

    private Mat getTranslationMatrix() {
        Mat res = new Mat(3,1,CV_32F);
        float[] data = Position.GetPosition();
        for (int i = 0; i < data.length; i++) {
            res.put(i,0,data[i]);
        }

        return res;
    }


    public void Start() {
        manager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mMagnetrometer, SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this, mLinearAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
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
            //AccelUpdate = true;
            Quaternion.Update(AccelData, GyroData, MagnetData);
            return;
        }
        else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            GyroData[0] = event.values[0];
            GyroData[1] = event.values[1];
            GyroData[2] = event.values[2];
            //GyroUpdate = true;
            Quaternion.Update(AccelData, GyroData, MagnetData);
            return;
        }
        else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            MagnetData[0] = event.values[0];
            MagnetData[1] = event.values[1];
            MagnetData[2] = event.values[2];
//            MagnetUpdate = true;
            Quaternion.Update(AccelData, GyroData, MagnetData);
            return;
        } else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            Position.Update(event.values[0], event.values[1], event.values[2], event.timestamp);
            return;
        }

//        if (AccelUpdate && GyroUpdate && MagnetUpdate) {
//            Quaternion.Update(AccelData, GyroData, MagnetData);
//            flushSensorData();
//        }
        return;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public class MotionSensor {
        private float[] postion;
        private float[] velocity;
        private long prevTime;
        private final Semaphore lock = new Semaphore(1, true);
        private int firstStep;
        private float[] firstData;

        public MotionSensor(){
            postion = new float[3];
            velocity = new float[3];
            firstData = new float[3];
            firstStep = -1;
            prevTime = 0;
        }

        public void Reset() {
//            try {
//                lock.acquire();
                for (int i = 0; i < 3; i++) {
                    postion[i] = 0;
                    velocity[i] = 0;
                }
//                lock.release();
//            } catch (InterruptedException err) {

//            }
        }

        public void Update(float ax, float ay, float az, long time) {
            if (firstStep == 1) {
//                try {
//                    lock.acquire();
                    float dt = (time - prevTime) / (float)Math.pow(10, 9);
                    velocity[0] += ax * dt;
                    velocity[1] += ay * dt;
                    velocity[2] += az * dt;
                    for (int i = 0; i < velocity.length; i++) {
                        postion[i] += velocity[i] * dt;
                    }

                    prevTime = time;
//                    lock.release();
//                }catch (InterruptedException err) {}
            } else if (firstStep == 0) {
//                try {
//                    lock.acquire();

                    float dt = (time - prevTime) / (float)Math.pow(10, 9);
                    for (int i = 0; i < velocity.length; i++) {
                        velocity[i] = firstData[i] * dt;
                        postion[i] = velocity[i] * dt;
                    }

                    velocity[0] += ax * dt;
                    velocity[1] += ay * dt;
                    velocity[2] += az * dt;
                    for (int i = 0; i < velocity.length; i++) {
                        postion[i] += velocity[i] * dt;
                    }

                    prevTime = time;
                    firstStep = 1;
//                    lock.release();
//                } catch (InterruptedException err) {}
            } else {
                firstData[0] = ax;
                firstData[1] = ay;
                firstData[2] = az;
                prevTime = time;
                firstStep = 0;
            }
        }

        public float[] GetPosition() {
            float[] res = new float[3];
//            try {

//                lock.acquire();
                for(int i = 0; i < 3; i++)
                    res[i] = postion[i];
//                lock.release();

//                Reset();
//            } catch (InterruptedException err) {

//            }
            return res;
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

        public float[] GetRotation() {
            float [] matrix = new float[9];
            SensorManager.getRotationMatrixFromVector(matrix, Quaternion);
//            Reset();
            return matrix;
        }

        public void Update(float[] a, float[] g, float[] m) {
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
            norm = (float)Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if (norm == 0f)
                return;
            norm = 1 / norm;        // use reciprocal for division
            a[0] *= norm;
            a[1] *= norm;
            a[2] *= norm;

            // Normalise magnetometer measurement
            norm = (float)Math.sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
            if (norm == 0f)
                return;
            norm = 1 / norm;        // use reciprocal for division
            m[0] *= norm;
            m[1] *= norm;
            m[2] *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2f * q1 * m[0];
            _2q1my = 2f * q1 * m[1];
            _2q1mz = 2f * q1 * m[2];
            _2q2mx = 2f * q2 * m[0];
            hx = m[0] * q1q1 - _2q1my * q4 + _2q1mz * q3 + m[0] * q2q2 + _2q2 * m[2] * q3 + _2q2 * m[2] * q4 - m[0] * q3q3 - m[0] * q4q4;
            hy = _2q1mx * q4 + m[1] * q1q1 - _2q1mz * q2 + _2q2mx * q3 - m[1] * q2q2 + m[1] * q3q3 + _2q3 * m[2] * q4 - m[1] * q4q4;
            _2bx = (float)Math.sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + m[2] * q1q1 + _2q2mx * q4 - m[2] * q2q2 + _2q3 * m[1] * q4 - m[2] * q3q3 + m[2] * q4q4;
            _4bx = 2f * _2bx;
            _4bz = 2f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2f * q2q4 - _2q1q3 - a[0]) + _2q2 * (2f * q1q2 + _2q3q4 - a[1]) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m[1]) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m[1]) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m[2]);
            s2 = _2q4 * (2f * q2q4 - _2q1q3 - a[0]) + _2q1 * (2f * q1q2 + _2q3q4 - a[1]) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - a[2]) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m[0]) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m[1]) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m[2]);
            s3 = -_2q1 * (2f * q2q4 - _2q1q3 - a[0]) + _2q4 * (2f * q1q2 + _2q3q4 - a[1]) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - a[2]) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m[0]) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m[1]) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m[2]);
            s4 = _2q2 * (2f * q2q4 - _2q1q3 - a[0]) + _2q3 * (2f * q1q2 + _2q3q4 - a[1]) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - m[0]) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - m[1]) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - m[2]);
            norm = (float)Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            if (norm == 0f)
                return;
            norm = 1f / norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * g[0] - q3 * g[1] - q4 * g[2]) - Beta * s1;
            qDot2 = 0.5f * (q1 * g[0] + q3 * g[2] - q4 * g[1]) - Beta * s2;
            qDot3 = 0.5f * (q1 * g[1] - q2 * g[2] + q4 * g[0]) - Beta * s3;
            qDot4 = 0.5f * (q1 * g[2] + q2 * g[1] - q3 * g[0]) - Beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * SamplePeriod;
            q2 += qDot2 * SamplePeriod;
            q3 += qDot3 * SamplePeriod;
            q4 += qDot4 * SamplePeriod;
            norm = (float)Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            if (norm == 0f)
                return;
            norm = 1f / norm;
            Quaternion[0] = q1 * norm;
            Quaternion[1] = q2 * norm;
            Quaternion[2] = q3 * norm;
            Quaternion[3] = q4 * norm;
        }
    }
}
