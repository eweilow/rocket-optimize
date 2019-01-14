using System;
using OpenTK;

namespace RocketOptimize.Simulation.Integrators
{
    public interface IIntegrator
    {
        void Integrate(float timeStep, ref State currentState, out State newState, Func<State, Vector3d> computeCurrentForce);
    }
}
