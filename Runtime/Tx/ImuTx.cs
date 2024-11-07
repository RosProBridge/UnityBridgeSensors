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
        public Vector3 acceleration { get; private set; }

        private Vector3 _lastVel = new(0, 0, 0);

        private Vector3 _lastPosition;
        private Quaternion _lastRotation;
        private Vector3 _velocity;
        private Vector3 _angularVelocity;
        private bool _isGlobal;
        private Vector3 _gravityDirection;
        private float _gravityMagnitude;

        protected override void OnStart()
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
                acceleration = transform.InverseTransformDirection(acceleration);
                _angularVelocity = transform.InverseTransformDirection(_angularVelocity);
            }

            _lastVel = _velocity;
        }
        protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.angular_velocity = _angularVelocity.ToRos();
            data.linear_acceleration = acceleration.ToRos();
            data.orientation = transform.rotation.ToRos();

            return base.GetMsg(ts);
        }
    }
}