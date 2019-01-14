using System;
using OpenTK;

namespace RocketOptimize.App.Render
{
    public enum ProjectionType
    {
        Perspective,
        Ortographic
    }

    public class Camera
    {
        public ProjectionType ProjectionType { get; private set; }

        public int Width { get; private set; }
        public int Height { get; private set; }

        public float FovRads
        {
            get
            {
                return (Fov * (float)Math.PI) / 180f;
            }
        }

        public float Fov { get; private set; }

        public float NearZ { get; private set; }
        public float FarZ { get; private set; }

        public Matrix4 ProjectionMatrix { get; private set; }
        public Matrix4 ViewMatrix { get; private set; }

        public Vector3 Position { get; private set; }
        public Vector3 LookAt { get; private set; }

        private void Update()
        {
            ViewMatrix = Matrix4.LookAt(Position, LookAt, Vector3.UnitY);

            switch (ProjectionType)
            {
                case ProjectionType.Perspective:
                    ProjectionMatrix = Matrix4.CreatePerspectiveFieldOfView(FovRads, Width / (float)Height, NearZ, FarZ);
                    break;
                case ProjectionType.Ortographic:
                    var halfWidth = Width / 2f;
                    var halfHeight = Height / 2f;
                    ProjectionMatrix = Matrix4.CreateOrthographicOffCenter(-halfWidth, halfWidth, -halfHeight, halfHeight, NearZ, FarZ);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

        }

        public void Resize(int w, int h)
        {
            Width = w;
            Height = h;
            Update();
        }

        public void SetPosition(Vector3 position)
        {
            Position = position;
            Update();
        }

        public void SetLookat(Vector3 lookat)
        {
            LookAt = lookat;
            Update();
        }

        public void SetProjectionPerspective(float fov, float near, float far)
        {
            ProjectionType = ProjectionType.Perspective;
            Fov = fov;
            NearZ = near;
            FarZ = far;
            Update();
        }

        public void SetProjectionOrthographic(float near, float far)
        {
            ProjectionType = ProjectionType.Ortographic;
            NearZ = near;
            FarZ = far;
            Update();
        }
    }
}
