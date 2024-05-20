using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class SpatialHash
{
    (int offsetX, int offsetY)[] cellOffsets =
    {
        (-1, 1),
        (0, 1),
        (1, 1),
        (-1, 0),
        (0, 0),
        (1, 0),
        (-1, -1),
        (0, -1),
        (1, -1),
    };
    class Entry : IComparable
    {
        public int index;
        public uint cellKey;

        public Entry(int index, uint cellKey)
        {
            this.index = index;
            this.cellKey = cellKey;
        }

        public int CompareTo(object obj)
        {
            Entry e = obj as Entry;
            if(!cellKey.Equals(e.cellKey))
            {
                return cellKey.CompareTo(e.cellKey);
            }
            else
            {
                return index.CompareTo(e.index);
            }
        }
    }

    Entry[] spatialLookup;
    int[] startIndices;

    Vector2[] points;
    float radius;
    Simulation2D simulation;

    public SpatialHash(int numPartical, Simulation2D sim)
    {
        spatialLookup = new Entry[numPartical];
        startIndices = new int[numPartical];
        points = new Vector2[numPartical];
        simulation = sim;
    }

    //��һ��positionת��Ϊcell��x��y
    public (int x, int y) PositionToCellCoord(Vector2 point, float radius)
    {
        int cellX = (int)(point.x / radius);
        int cellY = (int)(point.y / radius);
        return (cellX, cellY);
    }
    //���ϣֵ
    public uint HashCell(int cellX, int cellY)
    {
        uint a = (uint)cellX * 15823;
        uint b = (uint)cellY * 9737333;
        return a + b;
    }
    //�ɹ�ϣֵ�õ�cellKey
    public uint GetKeyFromHash(uint hash)
    {
        return hash % (uint)spatialLookup.Length;
    }
    //���¿ռ��������
    public void UpdateSpatialLookup(Vector2[] points, float radius)
    {
        this.points = points;
        this.radius = radius;

        //���¿ռ��cellKey
        Parallel.For(0, points.Length, i =>
        {
            (int cellX, int cellY) = PositionToCellCoord(points[i], radius);
            uint cellKey = GetKeyFromHash(HashCell(cellX, cellY));
            spatialLookup[i] = new Entry(i, cellKey);
            startIndices[i] = int.MaxValue;
        });

        //����cellKey����
        Array.Sort(spatialLookup);

        //����ռ���ÿһ����Ԫ��ʼλ�õ�����
        Parallel.For(0, points.Length, i =>
        {
            uint key = spatialLookup[i].cellKey;
            uint keyPre = i == 0 ? uint.MaxValue : spatialLookup[i - 1].cellKey;

            if(key != keyPre)
            {
                startIndices[key] = i;
            }
        });
    }
    //����ʾ��
    public void ForeachPointWithinRadius(Vector2 samplePoint)
    {
        //������ת��Ϊ��������
        (int centerX, int centerY) = PositionToCellCoord(samplePoint, radius);
        float sqrRadius = radius * radius;

        //�����������ڵ�9������
        foreach((int offsetX, int offsetY) in cellOffsets)
        {
            //����������ת��ΪcellKey����ȡ����ʼ����
            uint cellKey = GetKeyFromHash(HashCell(centerX + offsetX, centerY + offsetY));
            int cellStartIndex = startIndices[cellKey];

            for(int i = cellStartIndex; i < spatialLookup.Length; i++)
            {
                //��cellKey�����������
                if (spatialLookup[i].cellKey != cellKey) break;

                //ȡ�����ӵ�����
                int particalIndex = spatialLookup[i].index;
                float sqrDst = (samplePoint - points[particalIndex]).sqrMagnitude;
                //û������ⷶΧ
                if(sqrDst <= sqrRadius)
                {
                    //�����ӽ��в���
                }
            }
        }
    }

    public (float, float) CalculateDensity(Vector2 samplePoint)
    {
        //������ת��Ϊ��������
        (int centerX, int centerY) = PositionToCellCoord(samplePoint, radius);
        float sqrRadius = radius * radius;

        (float, float) density = (0, 0);

        //�����������ڵ�9������
        foreach ((int offsetX, int offsetY) in cellOffsets)
        {
            //����������ת��ΪcellKey����ȡ����ʼ����
            uint cellKey = GetKeyFromHash(HashCell(centerX + offsetX, centerY + offsetY));
            int cellStartIndex = startIndices[cellKey];

            for (int i = cellStartIndex; i < spatialLookup.Length; i++)
            {
                //��cellKey�����������
                if (spatialLookup[i].cellKey != cellKey) break;

                //ȡ�����ӵ�����
                int particalIndex = spatialLookup[i].index;
                float sqrDst = (samplePoint - points[particalIndex]).sqrMagnitude;
                //û������ⷶΧ
                if (sqrDst <= sqrRadius)
                {
                    (float, float) densityStep = simulation.CalculateDensityStep(samplePoint, points[particalIndex]);
                    //�����ӽ��в���
                    density.Item1 += densityStep.Item1;
                    density.Item2 += densityStep.Item2;
                }
            }
        }
        return density;
    }

    Vector2 RandomDir()
    {
        var rng = new Unity.Mathematics.Random(42);
        float randomAngle = (float)rng.NextDouble() * 3.14f * 2;
        return new Vector2(Mathf.Cos(randomAngle), Mathf.Sin(randomAngle));
    }

    public Vector2 CalculatePressureForce(int particalIndex)
    {
        Vector2 samplePoint = points[particalIndex];
        //������ת��Ϊ��������
        (int centerX, int centerY) = PositionToCellCoord(samplePoint, radius);
        float sqrRadius = radius * radius;

        Vector2 pressureForce = Vector2.zero;

        //�����������ڵ�9������
        foreach ((int offsetX, int offsetY) in cellOffsets)
        {
            //����������ת��ΪcellKey����ȡ����ʼ����
            uint cellKey = GetKeyFromHash(HashCell(centerX + offsetX, centerY + offsetY));
            int cellStartIndex = startIndices[cellKey];

            for (int i = cellStartIndex; i < spatialLookup.Length; i++)
            {
                //��cellKey�����������
                if (spatialLookup[i].cellKey != cellKey) break;

                //ȡ�����ӵ�����
                int otherParticalIndex = spatialLookup[i].index;
                float sqrDst = (samplePoint - points[otherParticalIndex]).sqrMagnitude;
                //û������ⷶΧ
                if (sqrDst <= sqrRadius)
                {
                    //�����ӽ��в���
                    pressureForce += simulation.CalculatePressureForceStep(particalIndex, otherParticalIndex);
                }
            }
        }

        return pressureForce;
    }

    public Vector2 CalculateViscosityForce(int particalIndex)
    {
        Vector2 samplePoint = points[particalIndex];
        //������ת��Ϊ��������
        (int centerX, int centerY) = PositionToCellCoord(samplePoint, radius);
        float sqrRadius = radius * radius;

        Vector2 viscosityForce = Vector2.zero;

        //�����������ڵ�9������
        foreach ((int offsetX, int offsetY) in cellOffsets)
        {
            //����������ת��ΪcellKey����ȡ����ʼ����
            uint cellKey = GetKeyFromHash(HashCell(centerX + offsetX, centerY + offsetY));
            int cellStartIndex = startIndices[cellKey];

            for (int i = cellStartIndex; i < spatialLookup.Length; i++)
            {
                //��cellKey�����������
                if (spatialLookup[i].cellKey != cellKey) break;

                //ȡ�����ӵ�����
                int otherParticalIndex = spatialLookup[i].index;
                float sqrDst = (samplePoint - points[otherParticalIndex]).sqrMagnitude;
                //û������ⷶΧ
                if (sqrDst <= sqrRadius)
                {
                    //�����ӽ��в���
                    viscosityForce += simulation.CalculateViscosityForceStep(particalIndex, otherParticalIndex);
                }
            }
        }

        return viscosityForce;
    }
}
