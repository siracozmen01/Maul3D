// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Maul3D C# starter kit: P/Invoke over the core surface (world,
// body, shape, joint, step, queries, snapshot). Single file, no
// dependencies, blittable structs only. Build the native library
// with -DMAUL3D_BUILD_SHARED=ON and drop it next to your binary
// (maul3d.dll / libmaul3d.so / libmaul3d.dylib).
//
// The layout law: every struct here mirrors the C header byte for
// byte. Call Maul3D.Native.LayoutCheck() once at startup; it
// compares Marshal.SizeOf against sizes probed from the C compiler
// and throws before a mismatch can corrupt anything. Bools in defs
// are bytes here (C _Bool is one byte; C# bool marshals as four).
// The rest of the API follows the same pattern; extend as needed.

using System;
using System.Runtime.InteropServices;

namespace Maul3D
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Vec3
    {
        public float X, Y, Z;
        public Vec3(float x, float y, float z) { X = x; Y = y; Z = z; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Quat
    {
        public float X, Y, Z, W;
        public static Quat Identity => new Quat { X = 0, Y = 0, Z = 0, W = 1 };
    }

    /// World positions are double precision (the hybrid precision law:
    /// positions in double, everything else in float).
    [StructLayout(LayoutKind.Sequential)]
    public struct Pos3
    {
        public double X, Y, Z;
        public Pos3(double x, double y, double z) { X = x; Y = y; Z = z; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct WorldId { public int Index1; public ushort Generation; }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyId { public int Index1; public ushort World0; public ushort Generation; }

    [StructLayout(LayoutKind.Sequential)]
    public struct ShapeId { public int Index1; public ushort World0; public ushort Generation; }

    [StructLayout(LayoutKind.Sequential)]
    public struct JointId { public int Index1; public ushort World0; public ushort Generation; }

    public enum BodyType : int { Static = 0, Kinematic = 1, Dynamic = 2 }

    public enum JointType : int
    {
        Spherical = 0, Revolute = 1, Prismatic = 2, Fixed = 3, Distance = 4,
        Generic = 5, Wheel = 6, Filter = 7, Parallel = 8, Motor = 9,
        Gear = 10, Pulley = 11
    }

    /// Build with Native.DefaultWorldDef(); the cookie in InternalValue
    /// is demanded by CreateWorld, so a hand-rolled def refuses loudly.
    [StructLayout(LayoutKind.Sequential)]
    public struct WorldDef
    {
        public Vec3 Gravity;
        public int BodyCapacity;
        public int ShapeCapacity;
        public int MeshCapacity;
        public int JointCapacity;
        public int VoxelCapacity;
        public int CharacterCapacity;
        public int VehicleCapacity;
        public int SoftBodyCapacity;
        public int WorkerCount;
        public IntPtr EnqueueTask;   // both null = serial (the default)
        public IntPtr FinishTask;
        public IntPtr UserTaskContext;
        public float ContactHertz;
        public float ContactDampingRatio;
        public float ContactPushMaxSpeed;
        public float RestitutionThreshold;
        public float MaximumLinearSpeed;
        public int EnableSleeping;
        public int EnableContinuous;
        public float HitEventThreshold;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BodyDef
    {
        public int Type; // BodyType
        public Pos3 Position;
        public Quat Rotation;
        public Vec3 LinearVelocity;
        public Vec3 AngularVelocity;
        public float GravityScale;
        public float LinearDamping;
        public float AngularDamping;
        public ulong UserData;
        public byte IsBullet; // C _Bool: one byte, 0 or 1
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ShapeDef
    {
        public float Density;
        public float Friction;
        public float Restitution;
        public ulong CategoryBits;
        public ulong MaskBits;
        public int GroupIndex;
        public float RollingResistance;
        public ulong UserData;
        public byte IsSensor;
        public byte EnableHitEvents;
        public byte EnablePreSolveEvents;
        public Vec3 LocalPosition;
        public Quat LocalRotation;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JointDef
    {
        public int Type; // JointType
        public BodyId BodyA;
        public BodyId BodyB;
        public Vec3 LocalAnchorA;
        public Vec3 LocalAnchorB;
        public Vec3 LocalAxisA;
        public Vec3 LocalAxisB;
        public byte EnableLimit;
        public float LowerLimit;
        public float UpperLimit;
        public byte EnableMotor;
        public float MotorSpeed;
        public float MaxMotorEffort;
        public byte EnableCone;
        public float ConeAngle;
        public byte CollideConnected;
        public byte GenericLinear0, GenericLinear1, GenericLinear2;
        public byte GenericAngular0, GenericAngular1, GenericAngular2;
        public byte GenericMotorAxis;
        public byte GenericReserved;
        public float GenericLinearLower0, GenericLinearLower1, GenericLinearLower2;
        public float GenericLinearUpper0, GenericLinearUpper1, GenericLinearUpper2;
        public float GenericAngularLower0, GenericAngularLower1, GenericAngularLower2;
        public float GenericAngularUpper0, GenericAngularUpper1, GenericAngularUpper2;
        public float Ratio;
        public Pos3 GroundAnchorA;
        public Pos3 GroundAnchorB;
        public int InternalValue;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Sphere { public Vec3 Center; public float Radius; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Capsule { public Vec3 Point1; public Vec3 Point2; public float Radius; }

    [StructLayout(LayoutKind.Sequential)]
    public struct Plane { public Vec3 Normal; public float Offset; }

    [StructLayout(LayoutKind.Sequential)]
    public struct RayHit
    {
        public ShapeId Shape;
        public Pos3 Point;
        public Vec3 Normal;
        public float Fraction;
        public byte Hit;
    }

    public static class Native
    {
        const string Lib = "maul3d";

        [DllImport(Lib, EntryPoint = "m3GetVersion")]
        public static extern int GetVersion();

        // World
        [DllImport(Lib, EntryPoint = "m3DefaultWorldDef")]
        public static extern WorldDef DefaultWorldDef();
        [DllImport(Lib, EntryPoint = "m3CreateWorld")]
        public static extern WorldId CreateWorld(ref WorldDef def);
        [DllImport(Lib, EntryPoint = "m3DestroyWorld")]
        public static extern void DestroyWorld(WorldId world);
        [DllImport(Lib, EntryPoint = "m3World_Step")]
        public static extern void Step(WorldId world, float dt, int substeps);
        [DllImport(Lib, EntryPoint = "m3World_Hash")]
        public static extern ulong Hash(WorldId world);

        // Snapshot and rollback: the buffer is opaque bytes; restore
        // returns false on any config or version mismatch (loudly, by
        // contract, never a partial restore).
        [DllImport(Lib, EntryPoint = "m3World_SnapshotSize")]
        public static extern int SnapshotSize(WorldId world);
        [DllImport(Lib, EntryPoint = "m3World_Snapshot")]
        public static extern int Snapshot(WorldId world, byte[] buffer, int capacity);
        [DllImport(Lib, EntryPoint = "m3World_Restore")]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool Restore(WorldId world, byte[] data, int size);

        // Body
        [DllImport(Lib, EntryPoint = "m3DefaultBodyDef")]
        public static extern BodyDef DefaultBodyDef();
        [DllImport(Lib, EntryPoint = "m3CreateBody")]
        public static extern BodyId CreateBody(WorldId world, ref BodyDef def);
        [DllImport(Lib, EntryPoint = "m3DestroyBody")]
        public static extern void DestroyBody(BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetPosition")]
        public static extern Pos3 GetPosition(BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetRotation")]
        public static extern Quat GetRotation(BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetLinearVelocity")]
        public static extern Vec3 GetLinearVelocity(BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_GetAngularVelocity")]
        public static extern Vec3 GetAngularVelocity(BodyId body);
        [DllImport(Lib, EntryPoint = "m3Body_SetLinearVelocity")]
        public static extern void SetLinearVelocity(BodyId body, Vec3 velocity);
        [DllImport(Lib, EntryPoint = "m3Body_SetAngularVelocity")]
        public static extern void SetAngularVelocity(BodyId body, Vec3 velocity);
        [DllImport(Lib, EntryPoint = "m3Body_SetTransform")]
        public static extern void SetTransform(BodyId body, Pos3 position, Quat rotation);

        // Shapes
        [DllImport(Lib, EntryPoint = "m3DefaultShapeDef")]
        public static extern ShapeDef DefaultShapeDef();
        [DllImport(Lib, EntryPoint = "m3CreateSphereShape")]
        public static extern ShapeId CreateSphereShape(BodyId body, ref ShapeDef def,
                                                       ref Sphere sphere);
        [DllImport(Lib, EntryPoint = "m3CreateBoxShape")]
        public static extern ShapeId CreateBoxShape(BodyId body, ref ShapeDef def,
                                                    Vec3 halfExtents);
        [DllImport(Lib, EntryPoint = "m3CreateCapsuleShape")]
        public static extern ShapeId CreateCapsuleShape(BodyId body, ref ShapeDef def,
                                                        ref Capsule capsule);
        [DllImport(Lib, EntryPoint = "m3CreatePlaneShape")]
        public static extern ShapeId CreatePlaneShape(BodyId body, ref ShapeDef def,
                                                      ref Plane plane);

        // Joints
        [DllImport(Lib, EntryPoint = "m3DefaultJointDef")]
        public static extern JointDef DefaultJointDef();
        [DllImport(Lib, EntryPoint = "m3CreateJoint")]
        public static extern JointId CreateJoint(ref JointDef def);
        [DllImport(Lib, EntryPoint = "m3DestroyJoint")]
        public static extern void DestroyJoint(JointId joint);

        // Queries
        [DllImport(Lib, EntryPoint = "m3World_CastRayClosest")]
        public static extern RayHit CastRayClosest(WorldId world, Pos3 origin, Vec3 translation);

        /// Sizes probed from the C compiler (x86-64 System V and
        /// Windows x64 agree on all of these). A mismatch here means
        /// a header drifted; fail before anything corrupts.
        public static void LayoutCheck()
        {
            Check<WorldDef>(112);
            Check<BodyDef>(104);
            Check<ShapeDef>(88);
            Check<JointDef>(224);
            Check<RayHit>(56);
            Check<WorldId>(8);
            Check<BodyId>(8);
            Check<Sphere>(16);
            Check<Capsule>(28);
            Check<Plane>(16);
        }

        static void Check<T>(int expected)
        {
            int actual = Marshal.SizeOf<T>();
            if (actual != expected)
            {
                throw new InvalidOperationException(
                    $"Maul3D layout drift: {typeof(T).Name} is {actual} bytes, expected {expected}");
            }
        }
    }
}
