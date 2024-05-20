using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;
using System.Text;

public class Simulation2D : MonoBehaviour
{
    [Header("Simulation Settings")]
    public int numPartical;
    public float gravity = 9.8f;
    [Range(0, 1)] public float collisionDamping = 0.95f;
    public Vector2 boundsSize;
    public bool fixedTimeStep = false;
    public float particalSpace = 0.2f;
    public float smoothRadius = 0.35f;
    public float mass = 1f;
    public float targetDensity;
    public float pressureMultipler;
    public float nearPressureMultipler;
    public float viscosityStrength;

    [Header("Interaction Settings")]
    public float interactionRadius;
    public float interactionStrength;

    float particalSize;
    Transform[] particals;
    Vector2[] positions;
    Vector2[] velocitys;
    Vector2[] predictPositions;

    public ComputeBuffer positionBuffer { get; private set; }
    public ComputeBuffer velocityBuffer { get; private set; }
    public ComputeBuffer densityBuffer { get; private set; }

    //density和nearDensity
    (float, float)[] densities;
    float[] particalPropertys;

    ParticalDisplay2D display;
    SpatialHash fixedRadiusNeighborSearch;

    //鼠标交互属性
    Vector2 mousePos;
    bool isPullInteraction;
    bool isPushInteraction;
    float currInteractStrength;

    void Init()
    {
        positions = new Vector2[numPartical];
        velocitys = new Vector2[numPartical];
        predictPositions = new Vector2[numPartical];
        particals = new Transform[numPartical];
        particalPropertys = new float[numPartical];
        densities = new (float, float)[numPartical];

        positionBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numPartical);
        velocityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numPartical);
        densityBuffer = ComputeHelper.CreateStructuredBuffer<float2>(numPartical);

        display = GetComponent<ParticalDisplay2D>();
        fixedRadiusNeighborSearch = new SpatialHash(numPartical, this);

        particalSize = display.scale;

        //prefab.transform.localScale = new Vector3(2*particalSize, 2*particalSize, 2*particalSize);

        int particalsPerRaw = (int)Mathf.Sqrt(numPartical);
        int particalsPerCol = (numPartical - 1) / particalsPerRaw + 1;
        float space =  particalSize + particalSpace;

        for(int i = 0; i < numPartical; i++)
        {
            float x = (i % particalsPerRaw - particalsPerRaw / 2f + 0.5f) * space;
            float y = (i / particalsPerRaw - particalsPerCol / 2f + 0.5f) * space;
            positions[i] = new Vector2(x, y);
            predictPositions[i] = new Vector2(x, y);
            //particals[i] = Instantiate(prefab, new Vector3(x, y, 0), Quaternion.identity);
        }


        positionBuffer.SetData(positions);
        velocityBuffer.SetData(velocitys);

        //display initial
        display.Init(this);
    }
    
    void Start()
    {
        Init();
    }

    void FixedUpdate()
    {
        if (fixedTimeStep)
        {
            RunSimulationFrame(Time.fixedDeltaTime);
        }
    }

    void Update()
    {
        // Run simulation if not in fixed timestep mode
        // (skip running for first few frames as deltaTime can be disproportionaly large)
        if (!fixedTimeStep && Time.frameCount > 20)
        {
            RunSimulationFrame(Time.deltaTime);
        }
    }

    void RunSimulationFrame(float frameTime)
    {
        //处理鼠标输入交互
        HandleMouseInput();

        //设置shader的buffer
        positionBuffer.SetData(positions);
        velocityBuffer.SetData(velocitys);
        //densityBuffer.SetData(densities);

        //计算物理运动
        SimulationStep(Time.deltaTime);
    }

    void ResolveCollisions(ref Vector2 position, ref Vector2 velocity)
    {
        Vector2 halfBoundSize = boundsSize / 2 - Vector2.one * particalSize;
        if(Mathf.Abs(position.x) > halfBoundSize.x)
        {
            position.x = halfBoundSize.x * Mathf.Sign(position.x);
            velocity.x *= -1 * collisionDamping;
        }
        if (Mathf.Abs(position.y) > halfBoundSize.y)
        {
            position.y = halfBoundSize.y * Mathf.Sign(position.y);
            velocity.y *= -1 * collisionDamping;
        }
    }

    float ViscositySmoothingKernel(float radius, float dst)
    {
        float volume = Mathf.PI * Mathf.Pow(radius, 8) / 4f;
        float value = Mathf.Max(0, radius * radius - dst * dst);
        return value * value * value / volume;
    }

    //float SmoothingKernelDerivative(float radius, float dst)
    //{
    //    if (dst >= radius) return 0;
    //    float f = radius * radius - dst * dst;
    //    float scale = -24 / (Mathf.PI * Mathf.Pow(radius, 8));
    //    return scale * dst * f * f;
    //}

    float SmoothingKernel(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float volume = Mathf.PI * Mathf.Pow(radius, 4) / 6f;
        float value = (radius - dst) * (radius - dst);
        return value / volume;
    }

    float SmoothingKernelDerivative(float radius, float dst)
    {
        if (dst >= radius) return 0;

        float scale = 12 / (Mathf.PI * Mathf.Pow(radius, 4));
        return -(radius - dst) * scale;
    }

    float NearDensityKernel(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float volume = (Mathf.PI * Mathf.Pow(radius, 5)) / 10f;
        float value = (radius - dst) * (radius - dst) * (radius - dst);
        return value / volume;
    }

    float NearDensityKernelDerivative(float radius, float dst)
    {
        if (dst >= radius) return 0;
        float v = radius - dst;
        float volume = (Mathf.Pow(radius, 5) * Mathf.PI) / 30f;
        float value = -v * v;
        return value / volume;
    }



    (float, float) CalculateDensity(Vector2 samplePoint)
    {
        (float, float) density = fixedRadiusNeighborSearch.CalculateDensity(samplePoint);
        return density;
    }

    public (float, float) CalculateDensityStep(Vector2 samplePoint, Vector2 otherPoint)
    {
        (float, float) density;
        float dst = (otherPoint - samplePoint).magnitude;
        density.Item1 = SmoothingKernel(smoothRadius, dst) * mass;
        density.Item2 = NearDensityKernel(smoothRadius, dst) * mass;
        return density;
    }

    Vector2 RandomDir()
    {
        var rng = new Unity.Mathematics.Random(42);
        float randomAngle = (float)rng.NextDouble() * 3.14f * 2;
        return new Vector2(Mathf.Cos(randomAngle), Mathf.Sin(randomAngle));
    }

    Vector2 CalculatePressureForce(int particalIndex)
    {
        Vector2 pressureForce = fixedRadiusNeighborSearch.CalculatePressureForce(particalIndex);
        return pressureForce;
    }

    public Vector2 CalculatePressureForceStep(int particalIndex, int otherIndex)
    {
        Vector2 pressureForce = Vector2.zero;
        float density = densities[particalIndex].Item1;
        float densityNear = densities[particalIndex].Item2;
        float pressure = ConvertDensityToPress(density);
        float nearPressure = ConvertNearDensityToPress(densityNear);

        if (particalIndex == otherIndex)
        {
            return pressureForce;
        }

        Vector2 offset = predictPositions[otherIndex] - predictPositions[particalIndex];
        float dst = offset.magnitude;
        Vector2 dir = dst == 0 ? RandomDir() : offset / dst;
        //Vector2 dir = offset.normalized;
        float slope1 = SmoothingKernelDerivative(smoothRadius, dst);
        float slope2 = NearDensityKernelDerivative(smoothRadius, dst);
        (float, float) otherD = densities[otherIndex];

        float otherDensity = otherD.Item1;
        float otherNearDensity = otherD.Item2;

        float otherPressure = ConvertDensityToPress(otherDensity);
        float otherNearPressure = ConvertNearDensityToPress(otherNearDensity);

        float sharedPressure = (pressure + otherPressure) * 0.5f;
        float sharedNearPressure = (nearPressure + otherNearPressure) * 0.5f;

        pressureForce += sharedPressure * slope1 * dir / otherDensity;
        pressureForce += sharedNearPressure * slope2 * dir / otherNearDensity;

        return pressureForce;
    }

    Vector2 CalculateViscosityForce(int particalIndex)
    {
        Vector2 viscosityForce = fixedRadiusNeighborSearch.CalculateViscosityForce(particalIndex);
        return viscosityForce * viscosityStrength;
    }

    public Vector2 CalculateViscosityForceStep(int particalIndex, int otherIndex)
    {
        Vector2 viscosityForce;
        Vector2 samplePoint = predictPositions[particalIndex];
        Vector2 otherPoint = predictPositions[otherIndex];

        float dst = (samplePoint - otherPoint).magnitude;
        float influence = ViscositySmoothingKernel(smoothRadius, dst);
        viscosityForce = (velocitys[otherIndex] - velocitys[particalIndex]) * influence;

        return viscosityForce;
    }

    Vector2 CalculateExternalForces(int particalIndex)
    {
        // Gravity
        Vector2 gravityAccel = gravity * Vector2.down;
        //默认外力加速度为重力
        Vector2 ExternalAcceleration = gravityAccel;

        if (currInteractStrength != 0)
        {
            Vector2 offset = (mousePos - positions[particalIndex]);
            float sqrDst = Vector2.Dot(offset, offset);
            //在交互范围内
            if (sqrDst < interactionRadius * interactionRadius)
            {
                float dst = Mathf.Sqrt(sqrDst);
                Vector2 dir = dst <= float.Epsilon ? Vector2.zero : offset / dst;
                float centOffset = 1 - dst / interactionRadius;
                float gravityWeight = 1 - (centOffset * Mathf.Clamp01(currInteractStrength / 10f));

                ExternalAcceleration = (dir * currInteractStrength - velocitys[particalIndex]) * centOffset;
                ExternalAcceleration += gravityAccel * gravityWeight;
            }
        }
        
        return ExternalAcceleration;
    }

    float ConvertDensityToPress(float density)
    {
        float densityDif = density - targetDensity;
        float pressure = densityDif * pressureMultipler;
        return pressure;
    }

    float ConvertNearDensityToPress(float nearDensity)
    {
        float pressure = nearDensity * nearPressureMultipler;
        return pressure;
    }

    float CalculateSharedPressure(float densityA, float densityB)
    {
        float pressureA = ConvertDensityToPress(densityA);
        float pressureB = ConvertDensityToPress(densityB);
        return (pressureA + pressureB) / 2f;
    }
    void UpdateDensities()
    {
        Parallel.For(0, numPartical, i =>
        {
            densities[i] = CalculateDensity(positions[i]);
        });
    }

    void SimulationStep(float deltaTime)
    {
        //计算重力,外力，预测位置
        Parallel.For(0, numPartical, i =>
        {
            velocitys[i] += CalculateExternalForces(i) * deltaTime;

            predictPositions[i] = positions[i] + velocitys[i] * 1 / 120f;
        });

        fixedRadiusNeighborSearch.UpdateSpatialLookup(predictPositions, smoothRadius);

        //计算密度
        Parallel.For(0, numPartical, i =>
        {
            densities[i] = CalculateDensity(predictPositions[i]);
        });

        //计算压力
        Parallel.For(0, numPartical, i =>
        {
            Vector2 pressureForce = CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / densities[i].Item1;
            velocitys[i] += pressureAcceleration * deltaTime;
        });

        //计算粘度
        Parallel.For(0, numPartical, i =>
        {
            Vector2 viscosityForce = CalculateViscosityForce(i);
            velocitys[i] += viscosityForce * deltaTime;
        });


        //更新位置和碰撞
        Parallel.For(0, numPartical, i =>
        {
            positions[i] += velocitys[i] * deltaTime;
            ResolveCollisions(ref positions[i], ref velocitys[i]);
        });
    }

    void HandleMouseInput()
    {
        mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        isPullInteraction = Input.GetMouseButton(0);
        isPushInteraction = Input.GetMouseButton(1);
        currInteractStrength = 0;
        if (isPushInteraction || isPullInteraction)
        {
            currInteractStrength = isPushInteraction ? -interactionStrength : interactionStrength;
        }
    }

    void OnDestroy()
    {
        ComputeHelper.Release(positionBuffer, velocityBuffer, densityBuffer);
    }
    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0, 1, 0, 0.4f);
        Gizmos.DrawWireCube(Vector2.zero, boundsSize);

        if (Application.isPlaying)
        {
            Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            bool isPullInteraction = Input.GetMouseButton(0);
            bool isPushInteraction = Input.GetMouseButton(1);
            bool isInteracting = isPullInteraction || isPushInteraction;
            if (isInteracting)
            {
                Gizmos.color = isPullInteraction ? Color.green : Color.red;
                Gizmos.DrawWireSphere(mousePos, interactionRadius);
            }
        }
    }
}
