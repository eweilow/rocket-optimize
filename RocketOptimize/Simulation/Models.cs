using OpenTK;
using System;

namespace RocketOptimize.Simulation
{
    public struct Atmosphere
    {
        public double GeometricAltitude;
        public double GeopotentialAltitude;
        public double Temperature;
        public double Pressure;
        public double Density;

        private static double Lerp(double a, double b, double factor)
        {
            return a + (b - a) * factor;
        }

        public static Atmosphere Interpolate(Atmosphere a, Atmosphere b, double factor)
        {
            return new Atmosphere()
            {
                GeometricAltitude = Lerp(a.GeometricAltitude, b.GeometricAltitude, factor),
                GeopotentialAltitude = Lerp(a.GeopotentialAltitude, b.GeopotentialAltitude, factor),
                Temperature = Lerp(a.Temperature, b.Temperature, factor),
                Pressure = Lerp(a.Pressure, b.Pressure, factor),
                Density = Lerp(a.Density, b.Density, factor),
            };
        }
    }

    public class Lerp<T>
    {
        public static Lerp<T> LinSpace(double from, double to, int points, Func<double, T> generator, Func<T, T, double, T> interpolate)
        {
            double step = (to - from) / (points-1);

            (double, T)[] lerpPoints = new (double, T)[points];
            for (int i = 0; i < points; i++)
            {
                double value = from + step * i;
                lerpPoints[i] = (value, generator(value));
            }

            return new Lerp<T>(lerpPoints, from, to, step, interpolate);
        }

        private readonly (double, T)[] Points;

        private readonly double From;
        private readonly double To;
        private readonly double Step;
        private readonly Func<T, T, double, T> Interpolate;

        protected Lerp((double, T)[] points, double from, double to, double step, Func<T, T, double, T> interpolate)
        {
            Points = points;
            From = from;
            To = to;
            Step = step;
            Interpolate = interpolate;
        }

        public T Get(double value)
        {
            if(value < From)
            {
                return Points[0].Item2;
            }
            else if (value > To || double.IsNaN(value) || double.IsInfinity(value))
            {
                return Points[Points.Length-1].Item2;
            }
            double offset = value - From;
            int lowIndex = (int)Math.Floor(offset / Step);

            if(lowIndex + 1 >= Points.Length)
            {
                return Points[Points.Length - 1].Item2;
            }

            double factor = (offset - lowIndex * Step) / Step;

            var low = Points[lowIndex].Item2;
            var high = Points[lowIndex + 1].Item2;

            return Interpolate(low, high, factor);
        }
    }

    public static class Models
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

        static double GeopotentialAltitude(double geometricAltitude)
        {
            return Constants.EarthRadius * (geometricAltitude * 1000.0) / (Constants.EarthRadius + (geometricAltitude * 1000.0)) / 1000.0;
        }

        public readonly static Lerp<Atmosphere> AtmosphereLerp = Lerp<Atmosphere>.LinSpace(0.0, 1000000.0, 1000, StandardAtmosphere, Atmosphere.Interpolate);

        public static Atmosphere StandardAtmosphere(double geometricAltitude)
        {
            var z = geometricAltitude / 1000.0; // geometric altitude
            var h = GeopotentialAltitude(z); // geopotential altitude

            if (h < 0.1)
            {
                h = 0.1;
            }

            if (z < 0.0)
            {
                z = 0.0;
            }

            if (z > 995.0)
            {
                z = 995.0;
            }

            double temperature;
            double pressure;
            double density;

            if (h < 11.0)
            {
                temperature = 288.15 - 6.5 * h;
                pressure = 101325.0 * Math.Pow(288.15 / (288.15 - 6.5 * h), 34.1632 / -6.5);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h < 20.0)
            {
                temperature = 216.65;
                pressure = 22632.06 * Math.Exp(-34.1632 * (h - 11.0) / 216.65);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h < 32.0)
            {
                temperature = 196.65 + h;
                pressure = 5474.889 * Math.Pow(216.65 / (216.65 + (h - 20.0)), 34.1632);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h < 47.0)
            {
                temperature = 139.05 + 2.8 * h;
                pressure = 868.0187 * Math.Pow(228.65 / (228.65 + 2.8 * (h - 32.0)), 34.1632 / 2.8);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h < 51.0)
            {
                temperature = 270.65;
                pressure = 110.9063 * Math.Exp(-34.1632 * (h - 47.0) / 270.65);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h < 71.0)
            {
                temperature = 413.45 - 2.8 * h;
                pressure = 66.93887 * Math.Pow(270.65 / (270.65 - 2.8 * (h - 51.0)), 34.1632 / -2.8);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else if (h <= 84.852)
            {
                temperature = 356.65 - 2.0 * h;
                pressure = 3.956420 * Math.Pow(214.65 / (214.65 - 2.0 * (h - 71.0)), 34.1632 / -2.0);
                density = pressure / (Constants.GasConstant * temperature);
            }
            else
            {
                const double MAX = 84.852;
                const double FALLOFF_ZERO = 90.0;

                var falloff = 1.0 - (h - MAX) / (FALLOFF_ZERO - MAX);
                temperature = 356.65 - 2.0 * MAX;
                if (h > FALLOFF_ZERO)
                {
                    pressure = 0.0;
                    density = 0.0;
                }
                else
                {
                    pressure = falloff * 3.956420 * Math.Pow(214.65 / (214.65 - 2.0 * (MAX - 71.0)), 34.1632 / -2.0);
                    density = pressure / (Constants.GasConstant * temperature);
                }
            }

            return new Atmosphere()
            {
                GeopotentialAltitude = h,
                GeometricAltitude = z,
                Temperature = temperature,
                Pressure = pressure,
                Density = density
            };
        }
    }
}
