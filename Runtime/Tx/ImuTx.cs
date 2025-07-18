using System;
using UnityEngine;
using ProBridge.Tx;
using ProBridge.Utils;
using sensor_msgs.msg;

namespace ProBridgeSenors.Tx
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/Imu")]
    public class ImuTx : ProBridgeTxStamped<Imu>
    {
        
        
        [Header("Noise Parameters")]
        public bool applyNoise = false;
        public float linearAccelerationNoiseStdDev = 0.0f;
        public float angularVelocityNoiseStdDev = 0.0f;
        public float orientationNoiseStdDev = 0.0f;
        

        private Vector3 _lastVel = new(0, 0, 0);

        private Vector3 _lastPosition;
        private Quaternion _lastRotation;
        private Vector3 _velocity;
        protected Vector3 _acceleration;
        protected Vector3 _angularVelocity;
        protected Quaternion _orientation;
        private bool _isGlobal;
        private Vector3 _gravityDirection;
        private float _gravityMagnitude;

        protected override void AfterEnable()
        {
            _gravityDirection = Physics.gravity.normalized;
            _gravityMagnitude = Physics.gravity.magnitude;
            _lastPosition = transform.position;
            _lastRotation = transform.rotation;
        }

        void FixedUpdate()
        {
            _velocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;

            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(_lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            _angularVelocity = Mathf.Deg2Rad * angle * axis / Time.fixedDeltaTime;

            _lastPosition = transform.position;
            _lastRotation = transform.rotation;

            acceleration = (_velocity - _lastVel) / Time.fixedDeltaTime - transform.InverseTransformDirection(_gravityDirection) * _gravityMagnitude;;

            if (!_isGlobal)
            {
                _acceleration = transform.InverseTransformDirection(_acceleration);
                _angularVelocity = transform.InverseTransformDirection(_angularVelocity);
            }

            _lastVel = _velocity;
            _orientation = transform.rotation;
        }
        protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
        {
            if (applyNoise)
            {
                data.angular_velocity.x += GaussianNoise.Generate(angularVelocityNoiseStdDev);
                data.angular_velocity.y += GaussianNoise.Generate(angularVelocityNoiseStdDev);
                data.angular_velocity.z += GaussianNoise.Generate(angularVelocityNoiseStdDev);
                
                data.linear_acceleration.x += GaussianNoise.Generate(linearAccelerationNoiseStdDev);
                data.linear_acceleration.y += GaussianNoise.Generate(linearAccelerationNoiseStdDev);
                data.linear_acceleration.z += GaussianNoise.Generate(linearAccelerationNoiseStdDev);
                
                var qNoise = Quaternion.Euler(
                    GaussianNoise.Generate(orientationNoiseStdDev),
                    GaussianNoise.Generate(orientationNoiseStdDev),
                    GaussianNoise.Generate(orientationNoiseStdDev)
                );

                var orientationWithNoise = data.orientation.FromRos() * qNoise;
                
                data.orientation = orientationWithNoise.ToRos();
            }            
            
            data.angular_velocity = _angularVelocity.ToRosAngular();
            data.linear_acceleration = _acceleration.ToRos();
            data.orientation = _orientation.ToRos();

            return base.GetMsg(ts);
        }
    }
}