using OpenTK;

namespace RocketOptimize.Simulation.Integrators
{
    public delegate Vector3d AccelerationCalculator(double dt, ref State state);

    public interface IIntegrator
    {
        void Integrate(double timeStep, ref State currentState, out State newState, AccelerationCalculator calculateAcceleration);
    }
}
