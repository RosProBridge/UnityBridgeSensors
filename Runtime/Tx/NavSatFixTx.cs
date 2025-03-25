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

        [Header("Noise Parameters")]
        public bool applyNoise = false;
        public float latitudeNoiseStdDev = 0.00001f;  // 0.00001 degrees, which is about 1.11 meters of noise
        public float longitudeNoiseStdDev = 0.00001f;  // 0.00001 degrees, which is about 1.11 meters of noise
        public float altitudeNoiseStdDev = 5.0f; // 5 meters standard deviation for altitude noise

        [Header("Values")]
        public double latitude;
        public double longitude;
        public double altitude;


        private Vector3 startPos;
        private System.Random random = new System.Random();

        protected override void AfterEnable()
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
            if (applyNoise)
            {
                latitude += GaussianNoise.Generate(latitudeNoiseStdDev);
                longitude += GaussianNoise.Generate(longitudeNoiseStdDev);
                altitude += GaussianNoise.Generate(altitudeNoiseStdDev);
            }
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
