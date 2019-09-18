package com.example.ys.orbtest;

import com.example.ys.orbtest.util.Mat;

public class Matrix {
    public int Cols;
    public int Rows;
    public float[][] data;
    public boolean state;

    public Matrix(int row, int col) {
        Cols = col;
        Rows = row;
        data = new float[row][col];
        state = true;
    }

    public Matrix(int row, int col, float[] dataVector) {
        Cols = col;
        Rows = row;
        data = new float[Rows][Cols];
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                data[i][j] = dataVector[(i*(col)) + j];
            }
        }
        state = true;
    }

    public Matrix clone() {
        Matrix result = new Matrix(this.Rows, this.Cols);
        for (int i = 0; i < result.Rows; i++) {
            for(int j = 0; j < result.Cols; j++) {
                result.data[i][j] = this.data[i][j];
            }
        }
        return result;
    }


    public static Matrix Transpose(Matrix a) {
        Matrix c = new Matrix(a.Cols, a.Rows);
        for (int i = 0; i < a.Rows; i++) {
            for (int j = 0; j < a.Cols; j++) {
                c.data[j][i] = a.data[i][j];
            }
        }

        return c;
    }

    public static Matrix Multiply(Matrix a, Matrix b) {
        if (a.Cols != b.Rows) {
            throw new Error("A.Cols != B.Rows");
        }

        Matrix c = new Matrix(a.Rows, b.Cols);

        for (int i = 0; i < c.Rows; i++) {
            for (int j = 0;j < c.Cols; j++) {
                c.data[i][j] = 0;
                for (int r = 0; r < a.Cols; r++) {
                    c.data[i][j] += a.data[i][r] * b.data[r][j];
                }
            }
        }

        return c;
    }

    public static Matrix Add(Matrix a, Matrix b) {
        if (a.Rows != b.Rows || a.Cols != b.Cols) {
            throw new Error("A.Rows != B.Rows");
        }

        Matrix c = new Matrix(a.Rows, a.Cols);
        for (int i = 0; i < a.Rows; i++) {
            for (int j = 0; j < a.Cols; j++) {
                c.data[i][j] = a.data[i][j] + b.data[i][j];
            }
        }

        return c;
    }

    public static Matrix Sub(Matrix a, Matrix b) {
        if (a.Rows != b.Rows || a.Cols != b.Cols) {
            throw new Error("A.Rows != B.Rows");
        }

        Matrix c = new Matrix(a.Rows, a.Cols);

        for (int i = 0; i < a.Rows; i++) {
            for(int j = 0; j < a.Cols; j++) {
                c.data[i][j] = a.data[i][j] - b.data[i][j];
            }
        }

        return c;
    }

    public static Matrix Invert2(Matrix a) {
        Matrix c = new Matrix(a.Rows, a.Cols);

        for (int i = 0; i < a.Rows; i++)
            for (int j = 0; j < a.Cols; j++)
                c.data[i][j] = (float)(Math.pow(-1, i + j) * determinant(minor(c.data, i, j)));

        // adjugate and determinant
        double det = determinant(c.data);
        if (Math.abs(det) <= 0.000000000000001) {
            c.state = false;
            return c;
        }
        det = 1.0 / det;
        for (int i = 0; i < c.Rows; i++) {
            for (int j = 0; j <= i; j++) {
                float temp = c.data[i][j];
                c.data[i][j] = (float)(c.data[j][i] * det);
                c.data[j][i] = (float)(temp * det);
            }
        }

        return c;
    }

    public static Matrix Invert(Matrix a) {
        Matrix c = new Matrix(a.Rows, a.Cols);

        Matrix.SetIdentity(c);

        int j;
        float scalar;
        for (int i = 0; i < a.Rows; i++) {
            if (Math.abs(a.data[i][i]) == 0.0) { //we have to swap rows here to make nonzero diagonal
                for (j = i; j < a.Rows; j++) {
                    if (a.data[j][j] != 0.0)
                        break;
                }

                if (j == a.Rows) {
                    c.state = false;
                    return c;  //can't get inverse matrix
                }
                swapRow(a, i, j);
                swapRow(c, i, j);
            } //if mtxin->data[r][r] == 0.0

            scalar = 1.0f / a.data[i][i];

            scaleRow(a, i, scalar);
            scaleRow(c, i, scalar);

            for (j = 0; j < i; j++) {
                scalar = -a.data[j][i];
                shearRow(a, j, i, scalar);
                shearRow(c, j, i, scalar);
            }

            for (j = i + 1; j < a.Rows; j++) {
                scalar = -a.data[j][i];
                shearRow(a, j, i, scalar);
                shearRow(c, j, i, scalar);
            }
        } //for r < mtxin->rows
        return c;
    }

    public static Matrix SubtractFromIdentity(Matrix a) {
        Matrix c = new Matrix(a.Rows, a.Cols);

        for (int i = 0; i < a.Rows; i++) {
            for (int j = 0; j < i; j++) {
                c.data[i][j] = -a.data[i][j];
            }
            c.data[i][i] = 1.0f - a.data[i][i];
            for (int j = i+1; j < a.Cols; j++) {
                c.data[i][j] = -a.data[i][j];
            }
        }

        return c;
    }

    public static boolean SetIdentity(Matrix a) {
        for (int i = 0; i < a.Rows; i++) {
            for (int j = 0; j < a.Cols; j++) {
                a.data[i][j] = 0.0f;
            }
            a.data[i][i] = 1.0f;
        }

        return true;
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

    private static double determinant(float[][] matrix) {
        if (matrix.length != matrix[0].length)
            throw new IllegalStateException("invalid dimensions");

        if (matrix.length == 2)
            return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];

        double det = 0;
        for (int i = 0; i < matrix[0].length; i++)
            det += Math.pow(-1, i) * matrix[0][i] * determinant(minor(matrix, 0, i));
        return det;
    }

    private static float[][] minor(float[][] matrix, int row, int column) {
        float[][] minor = new float[matrix.length - 1][matrix.length - 1];

        for (int i = 0; i < matrix.length; i++)
            for (int j = 0; i != row && j < matrix[i].length; j++)
                if (j != column)
                    minor[i < row ? i : i - 1][j < column ? j : j - 1] = matrix[i][j];
        return minor;
    }

    private static void swapRow(Matrix a, int i, int j) {
        float[] temp = a.data[i].clone();
        a.data[i] = a.data[j].clone();
        a.data[j] = temp.clone();
        return;
    }

    private static  void scaleRow(Matrix a, int row, float scalar) {
        for (int i = 0; i < a.Cols; i++)
            a.data[row][i] *= scalar;
    }

    private static void shearRow(Matrix a, int r1, int r2, float scalar) {
        for (int c = 0; c < a.Cols; c++)
            a.data[r1][c] += a.data[r2][c] * scalar;
    }
}
