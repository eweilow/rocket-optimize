using System;

namespace RocketOptimize.App
{
    public class Timer
    {
        public readonly int Duration;
        private bool _isDone;
        private DateTime _start;

        public Timer(int duration)
        {
            Duration = duration;
            Restart();
        }

        public void Restart()
        {
            _isDone = false;
            _start = DateTime.UtcNow;
        }

        public bool IsDone()
        {
            return (DateTime.UtcNow - _start).TotalMilliseconds > Duration;
        }
    }
}
