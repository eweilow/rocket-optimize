﻿using System;

namespace RocketOptimize.Simulation
{
    public static class Constants
    {
        public static double Scaling = 1.0;
        public static double EarthRadius = 6374000.0;
        public static double EarthGravitationalConstant = 3.986004418e14;
        public static double GasConstant = 287.053;
        public static double G0 = 9.0866;
        public static double SecondsInDay = 60 * 60 * 24;
        public static double EarthSurfaceVelocity = EarthRadius * 2 * Math.PI / SecondsInDay;
        public static double EarthSurfacePressure = Models.StandardAtmosphere(0.0).Pressure;
    }
}
