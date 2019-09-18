package com.example.ys.orbtest;

public class Kalman {

    public Matrix Xk_k1;
    public Matrix Xk;
    private Matrix Pk_k1;
    private Matrix Pk;
    private Matrix B;
    private Matrix u;
    private Matrix F;
    private Matrix H;
    private Matrix Q;
    private Matrix R;
    private Matrix Zk;
    private Matrix Yk;

    private int States;
    private int Controls;
    private int Measurements;

    public Kalman(int countState, int countControl, int countMeasurement) {
        States = countState;
        Controls = countControl;
        Measurements = countMeasurement;

        Xk_k1 = new Matrix(countState,1);
        Xk = new Matrix(countState,1);

        Pk_k1 = new Matrix(countState, countState);
        Pk = new Matrix(countState, countState);

        F = new Matrix(countState, countState);
        Q = new Matrix(countState, countState);
        u = new Matrix(countControl, 1);
        B = new Matrix(countState, countControl);
        H = new Matrix(countMeasurement, countState);
        R = new Matrix(countMeasurement, countMeasurement);
        Zk = new Matrix(countMeasurement, 1);
        Yk = new Matrix(countMeasurement, 1);

        Matrix.SetIdentity(H);
        Matrix.SetIdentity(Pk);

    }

    public void Free() {

    }

    public void Predict() {
        //Xk|k-1 = Fk*Xk-1|k-1 + Bk*Uk
        Xk_k1 = Matrix.Multiply(F, Xk);
        Matrix temp = Matrix.Multiply(B, u);
        Xk_k1 = Matrix.Add(Xk_k1, temp);

        //Pk|k-1 = Fk*Pk-1|k-1*Fk(t) + Qk
        Matrix temp2 = Matrix.Transpose(F);
        temp = Matrix.Multiply(F, Pk);
        Pk_k1 = Matrix.Multiply(temp, temp2);
        Pk_k1 = Matrix.Add(Pk_k1, Q);
    }

    public void Update() {
        //Yk = Zk - Hk*Xk|k-1
        Yk = Matrix.Multiply(H,Xk_k1);
        Yk = Matrix.Sub(Zk, Yk);

        //Sk = Rk + Hk*Pk|k-1*Hk(t)

        Matrix temp = Matrix.Transpose(H);
        temp = Matrix.Multiply(Pk_k1, temp);
        Matrix Sk = Matrix.Multiply(H, temp);
        Sk = Matrix.Add(R, Sk);

        //Kk = Pk|k-1*Hk(t)*Sk(inv)
        Matrix SkInv = Matrix.Invert(Sk);
        if (SkInv.state == false) {
            return;
        }
        Matrix K = Matrix.Multiply(temp, SkInv);

        //xk|k = xk|k-1 + Kk*Yk
        Xk = Matrix.Multiply(K, Yk);
        Xk = Matrix.Add(Xk_k1,Xk);

        //Pk|k = (I - Kk*Hk) * Pk|k-1 - SEE WIKI!!!
        Matrix temp2 = Matrix.Multiply(K, H);
        temp2 = Matrix.SubtractFromIdentity(temp2);
        Pk = Matrix.Multiply(temp2, Pk_k1);

        //we don't use this
        //Yk|k = Zk - Hk*Xk|k
        Yk = Matrix.Multiply(H, Xk);
        Yk = Matrix.Sub(Zk, Yk);
    }


    public void rebuildF(float dt) {
        for (int i = 0; i < States; i++) {
            for (int j = 0; j < States; j++)
                this.F.data[i][j] = 0;
            this.F.data[i][i] = 1;
        }

        for (int i = 3; i < States; i++) {
            this.F.data[i-3][i] = dt;
        }

        return;
    }

    public void rebuildB(float dt) {
        for (int i = 0; i < B.Cols; i++) {
            this.B.data[i][i] = dt * dt / 4;
        }

        for (int i = B.Cols; i < B.Rows; i++) {
            this.B.data[i][i - B.Cols] = dt / 2;
        }
    }

    public void rebuildU(float[] a) {
        for (int i = 0; i < a.length; i++) {
            this.u.data[i][0] = a[i];
        }
    }

    public void rebuildQ(float[] dev, float dt) {
        float[] velDev = new float[3];
        float[] posDev = new float[3];
        float[] covDev = new float[3];
        for (int i = 0; i < 3; i++) {
            velDev[i] = dev[i] * dt;
            posDev[i] = velDev[i] * dt / 2;
            covDev[i] = velDev[i] * posDev[i];
        }

        for (int i = 0; i < Q.Rows; i++) {
            for(int j = 0; j < Q.Cols; j++) {
                Q.data[i][j] = 0;
            }
        }

        Q.data[0][0] = posDev[0] * posDev[0];
        Q.data[1][1] = posDev[1] * posDev[1];
        Q.data[2][2] = posDev[2] * posDev[2];

        Q.data[0][3] = covDev[0];
        Q.data[1][4] = covDev[1];
        Q.data[2][5] = covDev[2];

        Q.data[3][0] = covDev[0];
        Q.data[4][1] = covDev[1];
        Q.data[5][2] = covDev[2];

        Q.data[3][3] = velDev[0] * velDev[0];
        Q.data[4][4] = velDev[1] * velDev[1];
        Q.data[5][5] = velDev[2] * velDev[2];

        return;
    }

    public void rebuildR(float dev) {
        for (int i = 0; i < R.Rows; i++) {
            for (int j = 0; j < R.Cols; j++) {
                R.data[i][j] = 0.0f;
            }

            if (i < 3)
                R.data[i][i] = dev;
            else
                R.data[i][i] = dev/10.0f;
        }

        return;
    }

    public void rebuildZ(Matrix measure) {
        for (int i = 0; i < measure.Rows; i++) {
            this.Zk.data[i][0] = measure.data[i][0];
        }

        return;
    }
}
