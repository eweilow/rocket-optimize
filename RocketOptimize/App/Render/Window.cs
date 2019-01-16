using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;

namespace RocketOptimize.App.Render
{
    public abstract class Window<T> : GameWindow where T : Camera
    {
        protected T Camera;

        public Window(int width = 1200, int height = 1200, string title = "Rocket Optimize") : base(width, height)
        {
            Title = title;
            VSync = VSyncMode.On;
        }

        public void Start(double updateRate)
        {
            Run(updateRate);
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            Camera = CreateCamera();
            DidResize(Width, Height);
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            base.OnUpdateFrame(e);
            UpdateTick((float)e.Time);
        }

        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height);
            DidResize(Width, Height);

            base.OnResize(e);
        }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);
            GL.Clear(ClearBufferMask.ColorBufferBit);

            if(Camera.UpdateMatrices())
            {
                var viewMatrix = Camera.ViewMatrix;
                GL.MatrixMode(MatrixMode.Modelview);
                GL.LoadMatrix(ref viewMatrix);

                var projectionMatrix = Camera.ProjectionMatrix;
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadMatrix(ref projectionMatrix);
            }

            RenderTick((float)e.Time);
            SwapBuffers();
        }

        public abstract T CreateCamera();
        public abstract void UpdateTick(float updateTime);
        public abstract void RenderTick(float updateTime);
        public abstract void DidResize(int width, int height);
    }
}
