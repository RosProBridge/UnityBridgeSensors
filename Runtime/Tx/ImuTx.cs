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
        public Rigidbody Rigidbody { get; private set; }
        public ArticulationBody ArticulationBody { get; private set; }
        public Vector3 Acceleration { get; private set; }

        public bool isNedImu;
        private Vector3 _lastVel = new Vector3(0,0,0);
        private Vector3 gravityDirection;
        private float gravityMagnitude;
        private Transform initTransform;


        protected override void OnStart()
        {
            Rigidbody = GetComponent<Rigidbody>();
            if (Rigidbody == null)
            {
                ArticulationBody = GetComponent<ArticulationBody>();
                if (ArticulationBody == null)
                {
                    Debug.LogWarning("No Articulation Body or Rigidbody found");
                    throw new NullReferenceException("No Articulation Body or Rigidbody found");
                }
            }
            gravityDirection = Physics.gravity.normalized;
            gravityMagnitude = Physics.gravity.magnitude;
            initTransform = this.transform;

        }
        void FixedUpdate()
        {
            var velocity = Rigidbody==null ? ArticulationBody.velocity : Rigidbody.velocity;
            if (isNedImu)
            {
                Acceleration = (velocity - _lastVel) / Time.deltaTime - initTransform.InverseTransformDirection(gravityDirection) * gravityMagnitude;

            }
            else
            {
                Acceleration = (velocity - _lastVel) / Time.deltaTime;
            }
            _lastVel = velocity;
        }


        protected override ProBridge.ProBridge.Msg GetMsg(TimeSpan ts)
        {
            var angularVelocity = Rigidbody==null? ArticulationBody.angularVelocity : Rigidbody.angularVelocity;
            
            data.angular_velocity = (Quaternion.Inverse(transform.rotation) * angularVelocity).ToRosAngular();
            data.linear_acceleration = Acceleration.ToRos();
            data.orientation = transform.rotation.ToRos();

            return base.GetMsg(ts);
        }
    }
}