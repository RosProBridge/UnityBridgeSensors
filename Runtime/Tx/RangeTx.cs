using UnityEngine;
using System.Collections.Generic;
using System;


namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/Range")]
    public class RangeTx : ProBridgeTxStamped<sensor_msgs.msg.Range>
    {
        public enum RadiationType
        {
            ULTRASOUND = sensor_msgs.msg.Range.ULTRASOUND,
            INFRARED = sensor_msgs.msg.Range.INFRARED
        }

        [Space(10)]
        public RadiationType radiationType = RadiationType.ULTRASOUND;

        [Range(0.001f, MathF.PI)]
        public float FOV = Mathf.PI / 6;
        public float minRange = 0.1f;
        public float maxRange = 10.0f;
        public int horizontalResolution = 10;
        public int verticalResolution = 10;

        public bool showGizmos = true;

        protected override ProBridge.Msg GetMsg(System.TimeSpan ts)
        {
            SimulateSensor();
            data.header.stamp = ts;
            return base.GetMsg(ts);
        }

        private void SimulateSensor()
        {
            Vector3 direction = transform.forward;

            float horizontalStep = FOV / Mathf.Max(1, horizontalResolution - 1);
            float verticalStep = FOV / Mathf.Max(1, verticalResolution - 1);
            float min_range = Mathf.Infinity;
            IterateFOV((vAngle, hAngle, rotatedDirection) =>
            {
                Vector3 rayOrigin = transform.position + rotatedDirection * minRange;
                if (Physics.Raycast(rayOrigin, rotatedDirection, out RaycastHit hit, maxRange - minRange))
                {
                    if (hit.distance + minRange < min_range)
                    {
                        min_range = hit.distance + minRange;
                    }
                }
            });

            data.field_of_view = FOV;
            data.max_range = maxRange;
            data.min_range = minRange;
            data.radiation_type = (byte)radiationType;
            data.range = min_range;
        }

        private void OnDrawGizmos()
        {
            if (!showGizmos) return;

            IterateFOV((vAngle, hAngle, rotatedDirection) =>
            {
                Vector3 rayOrigin = transform.position + rotatedDirection * minRange;
                Vector3 endpoint = transform.position + rotatedDirection * maxRange;

                if (Physics.Raycast(rayOrigin, rotatedDirection, out RaycastHit hit, maxRange - minRange))
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(rayOrigin, hit.point);
                }
                else
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawLine(rayOrigin, endpoint);
                }
            });
        }

        private void IterateFOV(System.Action<float, float, Vector3> action)
        {
            Vector3 direction = transform.forward;
            float horizontalStep = FOV / Mathf.Max(1, horizontalResolution - 1);
            float verticalStep = FOV / Mathf.Max(1, verticalResolution - 1);

            for (float vAngle = -FOV / 2; vAngle <= FOV / 2; vAngle += verticalStep)
            {
                for (float hAngle = -FOV / 2; hAngle <= FOV / 2; hAngle += horizontalStep)
                {
                    Quaternion rotation = Quaternion.Euler(vAngle * Mathf.Rad2Deg, hAngle * Mathf.Rad2Deg, 0);
                    Vector3 rotatedDirection = rotation * direction;
                    action(vAngle, hAngle, rotatedDirection);
                }
            }
        }
    }
}