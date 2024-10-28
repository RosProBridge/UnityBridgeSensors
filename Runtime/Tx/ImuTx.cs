using System;
using UnityEngine;
using ProBridge.Tx;
using ProBridge.Utils;
using sensor_msgs.msg;

namespace ProBridgeSenors.Tx
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/Imu")]
    [RequireComponent(typeof(Rigidbody))]
    public class ImuTx : ProBridgeTxStamped<Imu>
    {
        public Rigidbody Body { get; private set; }
        public Vector3 Acceleration { get; private set; }

        public bool isNedImu;
        private Vector3 _lastVel = new Vector3(0,0,0);
        private Vector3 gravityDirection;
        private float gravityMagnitude;
        private Transform initTransform;


        protected override void OnStart()
        {
            Body = GetComponent<Rigidbody>();
            gravityDirection = Physics.gravity.normalized;
            gravityMagnitude = Physics.gravity.magnitude;
            initTransform = this.transform;

        }
        void FixedUpdate()
        {
            if (isNedImu)
            {
                Acceleration = (Body.velocity - _lastVel) / Time.deltaTime - initTransform.InverseTransformDirection(gravityDirection) * gravityMagnitude;

            }
            else
            {
                Acceleration = (Body.velocity - _lastVel) / Time.deltaTime;
            }
            _lastVel = Body.velocity;
        }


        protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.angular_velocity = (Quaternion.Inverse(Body.rotation) * Body.angularVelocity).ToRosAngular();
            data.linear_acceleration = Acceleration.ToRos();
            data.orientation = Body.rotation.ToRos();

            return base.GetMsg(ts);
        }
    }
}