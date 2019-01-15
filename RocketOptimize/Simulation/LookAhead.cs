using OpenTK;
using System;

namespace RocketOptimize.Simulation
{
    public struct LookAheadState
    {
        public Vector3d[] FuturePositions;
        public double Apoapsis;
        public double Periapsis;
        public double SemiMajorAxis;
        public double SemiMinorAxis;
        public double SemiLatusRectum;
        public double Theta;
        public double ArgumentOfPeriapsis;

        public LookAheadState(int length)
        {
            FuturePositions = new Vector3d[length];
            Apoapsis = 0;
            Periapsis = 0;
            SemiMajorAxis = 0;
            SemiMinorAxis = 0;
            SemiLatusRectum = 0;
            Theta = 0;
            ArgumentOfPeriapsis = 0;
        }
    }

    public static class LookAhead
    {
        public static bool Intersect(Vector3d[] vectors, int currentLast, out Vector3d intersection)
        {
            if(currentLast < 1)
            {
                intersection = vectors[0];
                return false;
            }

            Vector3d d = (vectors[currentLast] - vectors[currentLast - 1]);
            if(d.LengthSquared < 1e-5)
            {
                intersection = vectors[currentLast];
                return false;
            }
            d.Normalize();

            Vector3d m = vectors[currentLast];

            double b = Vector3d.Dot(m, d);
            double c = m.LengthSquared - Constants.EarthRadius * Constants.EarthRadius;

            if(c > 0 && b > 0)
            {
                intersection = vectors[currentLast];
                return false;
            }

            double discriminant = b * b - c;
            if(discriminant < 0)
            {
                intersection = vectors[currentLast];
                return false;
            }

            double t = -b - Math.Sqrt(discriminant);
            if(t < 0)
            {
                return Intersect(vectors, currentLast - 1, out intersection);
            }
            intersection = vectors[currentLast] + t * d;
            return true;
        }

        static bool SampleOrbit(ref Vector3d[] placeInto, ref Matrix4d rotationMatrix, double semiLatusRectum, double eccentricity, double offsetAngle, double angle, int offset, int steps)
        {
            double step = angle / (steps-1);

            for (int i = 0; i < steps; i++)
            {
                var x = offsetAngle - step * i;
                var currentRadius = semiLatusRectum / (1 + eccentricity * Math.Cos(x));

                var untransformedVector = new Vector3d(Math.Sin(x) * currentRadius, -Math.Cos(x) * currentRadius, 0);
                Vector3d.TransformVector(ref untransformedVector, ref rotationMatrix, out placeInto[i + offset]);

                if (currentRadius < Constants.EarthRadius)
                {
                    Intersect(placeInto, i + offset, out placeInto[i + offset]);
                    for (int j = offset + i + 1; j < placeInto.Length; j++)
                    {
                        placeInto[j] = placeInto[i + offset];
                    }
                    return false;
                }
            }
            return true;
        }

        public static void CalculateOrbit(ref LookAheadState lookAheadState, State state)
        {
            var mu = Constants.EarthGravitationalConstant;

            var radiusVector = state.Position;
            var velocityVector = state.Velocity;

            var h = Vector3d.Cross(
                radiusVector,
                velocityVector
            );

            var r = radiusVector.Length;

            var eccentricityVector = (
                (velocityVector.LengthSquared - mu / r) * radiusVector - Vector3d.Dot(radiusVector, velocityVector) * velocityVector
               ) / mu;

            var eccentricity = eccentricityVector.Length;
            var specificOrbitalEnergy = velocityVector.LengthSquared / 2 - mu / r;

            var semiMajorAxis = -mu / (2 * specificOrbitalEnergy);
            var semiMinorAxis = semiMajorAxis * Math.Sqrt(1 - eccentricity * eccentricity);

            var apoapsis = (1 + eccentricity) * semiMajorAxis;
            var periapsis = (1 - eccentricity) * semiMajorAxis;

            var semiLatusRectum = periapsis * (1 + eccentricity);

            var thetaFromPeriapsis = Math.Acos(((semiLatusRectum - r) / (eccentricity * r)) % (Math.PI * 2));

            var actualRotation = Math.Atan2(radiusVector.Y, radiusVector.X) + Math.PI / 2;

            double argumentOfPeriapsis;
            if (h.Z < 0)
            {
                thetaFromPeriapsis = Math.PI * 2 - thetaFromPeriapsis;
                argumentOfPeriapsis = actualRotation - thetaFromPeriapsis;
                //rotationMatrix = Matrix4d.Mult(rotationMatrix, Matrix4d.RotateY(Math.PI));
            }
            else
            {
                argumentOfPeriapsis = actualRotation - thetaFromPeriapsis;
            }
            var rotationMatrix = Matrix4d.CreateRotationZ(argumentOfPeriapsis);


            int highAccuracyPoints = lookAheadState.FuturePositions.Length / 2;
            double highAccuracyAngle = Math.PI / 20;

			if(SampleOrbit(ref lookAheadState.FuturePositions, ref rotationMatrix, semiLatusRectum, eccentricity, thetaFromPeriapsis, highAccuracyAngle, 0, highAccuracyPoints))
            {
                SampleOrbit(ref lookAheadState.FuturePositions, ref rotationMatrix, semiLatusRectum, eccentricity, thetaFromPeriapsis - highAccuracyAngle, Math.PI*2 - highAccuracyAngle, highAccuracyPoints, lookAheadState.FuturePositions.Length - highAccuracyPoints);
            }
            //double approximatePerimeter = Math.PI * (3 * (semiMajorAxis + semiMinorAxis) - Math.Sqrt((3*semiMajorAxis + semiMinorAxis) * (semiMajorAxis + 3*semiMinorAxis)));

            lookAheadState.Apoapsis = apoapsis - Constants.EarthRadius;
            lookAheadState.Periapsis = periapsis - Constants.EarthRadius;
            lookAheadState.SemiMajorAxis = semiMajorAxis;
            lookAheadState.SemiMinorAxis = semiMinorAxis;
            lookAheadState.SemiLatusRectum = semiLatusRectum;
            lookAheadState.ArgumentOfPeriapsis = argumentOfPeriapsis;
            lookAheadState.Theta = thetaFromPeriapsis;

            //Title = string.Format("e{0,2:F}, {1,2:F} km {2,2:F} km, r {3,2:F}, v {4,2:F} {5,2:F} {6,2:F} {7,2:F}, a{8,2:F} b{9,2:F}", e, (apoapsis - Constants.EarthRadius)/1000, (periapsis - Constants.EarthRadius) / 1000, (radiusVector.Length - Constants.EarthRadius) /1000, velocityVector.Length / 1000, theta2, actualRotation, offset, (a - Constants.EarthRadius)/1000, (b - Constants.EarthRadius)/1000);
            /*
                        GL.PushMatrix();
                        //GL.Rotate(offset / Math.PI * 180, Vector3d.UnitZ);
                        GL.LineWidth(2f);
                        GL.Color3(1f, 0f, 0f);
                        GL.Begin(PrimitiveType.LineLoop);
                        for (var theta = 0.0; theta < Math.PI * 2.0; theta += 0.01)
                        {
                            GL.Vertex3(Math.Sin(theta) * b, Math.Cos(theta) * a + (apoapsis - a), 0);
                        }
                        GL.End();
                        GL.PopMatrix();
            */
        }
    }
}
