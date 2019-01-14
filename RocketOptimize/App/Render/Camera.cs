using OpenTK;
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
            SetProjectionOrthographic(-1f, 1f);
            SetPosition(Vector3.Zero);
            SetLookat(-Vector3.UnitZ);
        }

        private float _centerX;
        private float _centerY;
        private float _size;
        private float _targetCenterX;
        private float _targetCenterY;
        private float _targetSize;

        public float Smoothing = 0.5f;

        public SmoothOrthoCamera(float centerX, float centerY, int size)
        {
            _centerX = centerX;
            _centerY = centerY;
            _size = size;
            _targetCenterX = centerX;
            _targetCenterY = centerY;
            _targetSize = size;
        }

        public void CenterOn(float minX, float maxX, float minY, float maxY, float scale, int minSize)
        {
            float width = maxX - minX;
            float height = maxY - minY;

            int size = Math.Max(minSize, Math.Max((int)(width * scale), (int)(height * scale)));

            float centerX = minX + (width / 2);
            float centerY = minY + (height / 2);

            SetExtents(centerX, centerY, size);
        }

        public void SetExtents(float centerX, float centerY, int size)
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
            SetPosition(new OpenTK.Vector3() { X = _centerX, Y = _centerY, Z = 0 });
            SetLookat(new OpenTK.Vector3() { X = _centerX, Y = _centerY, Z = -1 });
        }
    }

    public class Camera
    {
        protected bool hasChanged = false;

        public ProjectionType ProjectionType { get; protected set; }

        public int Width { get; protected set; }
        public int Height { get; protected set; }

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

        public bool UpdateMatrices()
        {
            if (hasChanged == false)
            {
                return false;
            }
            hasChanged = false;

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
            return true;
        }

        public void Resize(int w, int h)
        {
            Width = w;
            Height = h;
            hasChanged = true;
        }

        public void SetPosition(Vector3 position)
        {
            Position = position;
            hasChanged = true;
        }

        public void SetLookat(Vector3 lookat)
        {
            LookAt = lookat;
            hasChanged = true;
        }

        public void SetProjectionPerspective(float fov, float near, float far)
        {
            ProjectionType = ProjectionType.Perspective;
            Fov = fov;
            NearZ = near;
            FarZ = far;
            hasChanged = true;
        }

        public void SetProjectionOrthographic(float near, float far)
        {
            ProjectionType = ProjectionType.Ortographic;
            NearZ = near;
            FarZ = far;
            hasChanged = true;
        }
    }
}
