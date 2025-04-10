using System;
using System.Collections.Generic;
using UnityEditor.Rendering;
using UnityEngine;
using Unity.Mathematics;

[Serializable]
public class Matrix
{
    public int Rows;
    public int Cols;
    public float[,] Data {get;}

    //constructors
    public Matrix(float[,] data)
    {
        Rows = data.GetLength(0);
        Cols = data.GetLength(1);
        Data = data;
    }

    public Matrix(int rows, int cols) {
        Rows = rows;
        Cols = cols;
        Data = new float[rows,cols];
    }

    //indexer
    public float this[int row, int col] {
        get {
            return Data[row,col];
        }
        set {
            Data[row,col] = value;
        }
    }

    public void fill(float value) {
        for (int row = 0; row < Rows; row++) {
            for (int col = 0; col < Cols; col++) {
                Data[row,col] = value;
            }
        }
    }

    public Matrix MatMul(Matrix B) {
        Matrix result = new Matrix(Rows, B.Cols);
        for (int rowA = 0; rowA < Rows; rowA++) {
            for (int colB = 0; colB < B.Cols; colB++) {
                float current = 0f;
                for (int index = 0; index < Cols; index++) {
                    current = current + Data[rowA, index] * B[index, colB];
                }
                result[rowA, colB] = current;
            }
        }
        return result;
    }

    public static Matrix FromFloat4x4(float4x4 mat) {
        Matrix result = new Matrix(4,4);
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                result[row,col] = mat[row][col];
            }
        }
        return result;
    }

    public static Matrix FromFloat3x3(float3x3 mat) {
        Matrix result = new Matrix(3,3);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result[row,col] = mat[row][col];
            }
        }
        return result;
    }
}