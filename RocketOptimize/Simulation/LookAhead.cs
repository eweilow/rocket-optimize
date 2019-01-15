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
    }

    public static class LookAhead
    {

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
                    for (int j = offset + i + 1; j < placeInto.Length; j++)
                    {
                        placeInto[j] = placeInto[i + offset];
                    }
                    return false;
                }
            }
            return true;
        }

        public static LookAheadState CalculateOrbit(State state)
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


            const int points = 15000;
            const int highAccuracyPoints = 10000;
            const double highAccuracyAngle = Math.PI / 16;
            var futurePositions = new Vector3d[points];

			if(SampleOrbit(ref futurePositions, ref rotationMatrix, semiLatusRectum, eccentricity, thetaFromPeriapsis, highAccuracyAngle, 0, highAccuracyPoints))
            {
                SampleOrbit(ref futurePositions, ref rotationMatrix, semiLatusRectum, eccentricity, thetaFromPeriapsis - highAccuracyAngle, Math.PI*2 - highAccuracyAngle, highAccuracyPoints, points - highAccuracyPoints);
            }
            //double approximatePerimeter = Math.PI * (3 * (semiMajorAxis + semiMinorAxis) - Math.Sqrt((3*semiMajorAxis + semiMinorAxis) * (semiMajorAxis + 3*semiMinorAxis)));
            

            return new LookAheadState()
            {
                FuturePositions = futurePositions,
                Apoapsis = apoapsis - Constants.EarthRadius,
                Periapsis = periapsis - Constants.EarthRadius,
                SemiMajorAxis = semiMajorAxis,
                SemiMinorAxis = semiMinorAxis,
                SemiLatusRectum = semiLatusRectum,
                ArgumentOfPeriapsis = argumentOfPeriapsis,
                Theta = thetaFromPeriapsis
            };

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
