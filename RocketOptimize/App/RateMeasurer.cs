using System.Diagnostics;

namespace RocketOptimize.App
{
    public class RateMeasurer
    {
        public double CurrentRate
        {
            get;
            private set;
        }

        private Timer _timer;
        private double _seconds;
        public RateMeasurer(double seconds)
        {
            _seconds = seconds;
            var time = (int)(seconds * 1000);
            _timer = new Timer(time);
            Collect();
        }

        private double _sampleTime;
        private int _samples;
        private void Collect()
        {
            var currentTime = _sampleTime / _samples;
            CurrentRate = 1000.0 / currentTime;
            _samples = 0;
            _sampleTime = 0.0;
            _timer.Restart();
        }

        public Stopwatch Measure()
        {
            var watch = new Stopwatch();
            watch.Start();
            return watch;
        }

        public void Sample(Stopwatch watch)
        {
            watch.Stop();
            double ticks = watch.ElapsedTicks;
            double elapsedMilliseconds = 1000.0 * ticks / Stopwatch.Frequency;

            _sampleTime += elapsedMilliseconds;
            _samples++;

            if (_timer.IsDone())
            {
                Collect();
            }
        }
    }
}
