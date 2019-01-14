using OpenTK;
using RocketOptimize.Simulation;
using System;

namespace RocketOptimize.App.Render
{
    public enum ProjectionType
    {
        Perspective,
        Ortographic
    }

    public class SmoothOrthoCamera : Camera
    {
        public SmoothOrthoCamera() : base()
        {
            SetProjectionOrthographic(-1.0, 1.0);
            SetPosition(Vector3d.Zero);
            SetLookat(-Vector3d.UnitZ);
        }

        private double _centerX;
        private double _centerY;
        private double _size;
        private double _targetCenterX;
        private double _targetCenterY;
        private double _targetSize;

        public double Smoothing = 0.5;

        public SmoothOrthoCamera(double centerX, double centerY, int size)
        {
            _centerX = centerX;
            _centerY = centerY;
            _size = size;
            _targetCenterX = centerX;
            _targetCenterY = centerY;
            _targetSize = size;
        }

        public void CenterOn(double minX, double maxX, double minY, double maxY, double scale, int minSize)
        {
            double width = maxX - minX;
            double height = maxY - minY;

            int size = Math.Max(minSize, Math.Max((int)(width * scale), (int)(height * scale)));

            double centerX = minX + (width / 2);
            double centerY = minY + (height / 2);

            SetExtents(centerX, centerY, size);
        }

        public void SetExtents(double centerX, double centerY, int size)
        {
            _targetCenterX = centerX;
            _targetCenterY = centerY;
            _targetSize = size;
        }

        public void Update()
        {
            _centerX += (_targetCenterX - _centerX) * Smoothing;
            _centerY += (_targetCenterY - _centerY) * Smoothing;
            _size += (_targetSize - _size) * Smoothing;

            SetProjectionOrthographic(-1f, 1f);
            Resize((int)_size, (int)_size);
            SetPosition(new Vector3d() { X = _centerX, Y = _centerY, Z = 0 });
            SetLookat(new Vector3d() { X = _centerX, Y = _centerY, Z = -1 });
        }
    }

    public class Camera
    {
        protected bool hasChanged = false;

        public ProjectionType ProjectionType { get; protected set; }

        public int Width { get; protected set; }
        public int Height { get; protected set; }

        public double FovRads
        {
            get
            {
                return (Fov * Math.PI) / 180.0;
            }
        }

        public double Fov { get; private set; }

        public double NearZ { get; private set; }
        public double FarZ { get; private set; }

        public Matrix4d ProjectionMatrix { get; private set; }
        public Matrix4d ViewMatrix { get; private set; }

        public Vector3d Position { get; private set; }
        public Vector3d LookAt { get; private set; }

        public bool UpdateMatrices()
        {
            if (hasChanged == false)
            {
                return false;
            }
            hasChanged = false;

            ViewMatrix = Matrix4d.LookAt(Position, LookAt, Vector3d.UnitY);

            switch (ProjectionType)
            {
                case ProjectionType.Perspective:
                    ProjectionMatrix = Matrix4d.CreatePerspectiveFieldOfView(FovRads, Width / (float)Height, NearZ, FarZ);
                    break;
                case ProjectionType.Ortographic:
                    var halfWidth = Width / 2f;
                    var halfHeight = Height / 2f;
                    ProjectionMatrix = Matrix4d.CreateOrthographicOffCenter(-halfWidth, halfWidth, -halfHeight, halfHeight, NearZ, FarZ);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
            return true;
        }

        public void Resize(int w, int h)
        {
            Width = w;
            Height = h;
            hasChanged = true;
        }

        public void SetPosition(Vector3d position)
        {
            Position = position;
            hasChanged = true;
        }

        public void SetLookat(Vector3d lookat)
        {
            LookAt = lookat;
            hasChanged = true;
        }

        public void SetProjectionPerspective(double fov, double near, double far)
        {
            ProjectionType = ProjectionType.Perspective;
            Fov = fov;
            NearZ = near;
            FarZ = far;
            hasChanged = true;
        }

        public void SetProjectionOrthographic(double near, double far)
        {
            ProjectionType = ProjectionType.Ortographic;
            NearZ = near;
            FarZ = far;
            hasChanged = true;
        }
    }
}
