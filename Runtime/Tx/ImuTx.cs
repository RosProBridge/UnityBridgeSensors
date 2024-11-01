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
        public Vector3 Acceleration { get; private set; }

        public bool isNedImu;
        private Vector3 _lastVel = new Vector3(0, 0, 0);
        private Vector3 gravityDirection;
        private float gravityMagnitude;
        private Transform initTransform;

        private Vector3 lastPosition;
        private Quaternion lastRotation;
        private Vector3 velocity;
        private Vector3 angularVelocity;


        protected override void OnStart()
        {
            gravityDirection = Physics.gravity.normalized;
            gravityMagnitude = Physics.gravity.magnitude;
            initTransform = transform;
            lastPosition = transform.position;
            lastRotation = transform.rotation;
        }

        void FixedUpdate()
        {
            velocity = (transform.position - lastPosition) / Time.fixedDeltaTime;

            Quaternion deltaRotation = transform.rotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out var angle, out var axis);
            angularVelocity = Mathf.Deg2Rad * angle * axis / Time.fixedDeltaTime;

            lastPosition = transform.position;
            lastRotation = transform.rotation;


            if (isNedImu)
            {
                Acceleration = (velocity - _lastVel) / Time.deltaTime -
                               initTransform.InverseTransformDirection(gravityDirection) * gravityMagnitude;
            }
            else
            {
                Acceleration = (velocity - _lastVel) / Time.deltaTime;
            }

            _lastVel = velocity;
        }


        protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.angular_velocity = (Quaternion.Inverse(transform.rotation) * angularVelocity).ToRosAngular();
            data.linear_acceleration = Acceleration.ToRos();
            data.orientation = transform.rotation.ToRos();

            return base.GetMsg(ts);
        }
    }
}