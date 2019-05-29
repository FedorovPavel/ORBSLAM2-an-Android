package com.example.ys.orbtest;

public class Matrix {
    public int Cols;
    public int Rows;
    public float[][] data;

    public Matrix(int col, int row) {
        Cols = col;
        Rows = row;
        data = new float[row][col];
    }

    public Matrix(int col, int row, float[] dataVector) {
        Cols = col;
        Rows = row;
        data = new float[Rows][Cols];
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                data[i][j] = dataVector[(i*(col)) + j];
            }
        }
    }

    public void Multiple(Matrix b) {
        float[][] a = this.data.clone();

        if (this.Cols != b.Rows) {
            return;
        }

        int cols = this.Cols;
        this.Cols = b.Cols;
        this.data = new float[this.Rows][this.Cols];

        for (int i = 0; i < this.Rows; i++) {
            for (int j = 0;j < this.Cols; j++) {
                this.data[i][j] = 0;
                for (int r = 0; r < cols; r++) {
                    this.data[i][j] += a[i][r] * b.data[r][j];
                }
            }
        }

        return;
    }

    public void Inverse() {
        float[][] inverse = new float[Rows][Cols];

        // minors and cofactors
        for (int i = 0; i < Rows; i++)
            for (int j = 0; j < Cols; j++)
                inverse[i][j] = (float)(Math.pow(-1, i + j) * determinant(minor(this.data, i, j)));

        // adjugate and determinant
        double det = 1.0 / determinant(this.data);
        for (int i = 0; i < inverse.length; i++) {
            for (int j = 0; j <= i; j++) {
                float temp = inverse[i][j];
                inverse[i][j] = (float)(inverse[j][i] * det);
                inverse[j][i] = (float)(temp * det);
            }
        }

        this.data = inverse;
    }

    public float[] GetMatrixAsVector() {
        float[] res = new float[Cols * Rows];
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                res[j + (i*(Cols))] = data[i][j];
            }
        }

        return res;
    }

    private double determinant(float[][] matrix) {
        if (matrix.length != matrix[0].length)
            throw new IllegalStateException("invalid dimensions");

        if (matrix.length == 2)
            return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];

        double det = 0;
        for (int i = 0; i < matrix[0].length; i++)
            det += Math.pow(-1, i) * matrix[0][i] * determinant(minor(matrix, 0, i));
        return det;
    }

    private float[][] minor(float[][] matrix, int row, int column) {
        float[][] minor = new float[matrix.length - 1][matrix.length - 1];

        for (int i = 0; i < matrix.length; i++)
            for (int j = 0; i != row && j < matrix[i].length; j++)
                if (j != column)
                    minor[i < row ? i : i - 1][j < column ? j : j - 1] = matrix[i][j];
        return minor;
    }

}
