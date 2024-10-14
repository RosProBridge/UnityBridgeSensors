using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/NavSatFix")]
    public class NavSatFixTx : ProBridgeTxStamped<sensor_msgs.msg.NavSatFix>
    {
        [Header("Params")]
        public double startLatitude;
        public double startLongitude;
        public double startAltitude;

        [Header("Values")]
        public double latitude;
        public double longitude;
        public double altitude;


        private Vector3 startPos;

        protected override void OnStart()
        {
            startPos = transform.position;
            UpdateLLA();
        }

        private void Update()
        {
            UpdateLLA();
        }

        private void UpdateLLA()
        {
            double la = startLatitude;
            double lo = startLongitude;
            double al = startAltitude;

            GeoConverter.Local2Global(transform.position - startPos, ref la, ref lo, ref al);

            latitude = la;
            longitude = lo;
            altitude = al;
        }

        protected override ProBridge.Msg GetMsg(TimeSpan ts)
        {
            data.latitude = latitude;
            data.longitude = longitude;
            data.altitude = altitude;

            return base.GetMsg(ts);
        }
    }
}
