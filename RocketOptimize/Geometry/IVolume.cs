using System;
using System.Collections.Generic;
using System.Text;

namespace RocketOptimize.Geometry
{
    public interface IVolume
    {
        double GetVolume();
        double GetShellArea();
        double GetShellVolume(double thickness);
    }
}
