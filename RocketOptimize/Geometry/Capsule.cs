using System;
using System.Collections.Generic;
using System.Text;

namespace RocketOptimize.Geometry
{

    public class Capsule : IVolume
    {
        public readonly double Radius;
        public readonly double Height;

        public Capsule(double radius, double height)
        {
            Radius = radius;
            Height = height;
        }

        public double GetShellArea()
        {
            throw new NotImplementedException();
        }

        public double GetShellVolume(double thickness)
        {
            throw new NotImplementedException();
        }

        public double GetVolume()
        {
            throw new NotImplementedException();
        }
    }
}
