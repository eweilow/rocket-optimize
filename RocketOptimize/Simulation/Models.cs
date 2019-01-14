using OpenTK;
using System;

namespace RocketOptimize.Simulation
{
    static class Models
    {
        static double GravityIntensity(double gravitationalConstant, double radiusSquared)
        {
            return gravitationalConstant / radiusSquared;
        }

        public static Vector3d Gravity(double gravitationalConstant, Vector3d target, Vector3d origin)
        {
            Vector3d relative = origin - target;
            double radiusSquared = relative.LengthSquared;
            relative.Normalize();

            return relative * GravityIntensity(gravitationalConstant, radiusSquared);
        }
    }
}
