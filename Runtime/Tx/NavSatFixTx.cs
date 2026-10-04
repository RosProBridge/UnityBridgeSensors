using System;
using UnityEngine;
using ProBridge.Utils;

namespace ProBridge.Tx.Sensor
{
    [AddComponentMenu("ProBridge/Tx/sensor_msgs/NavSatFix")]
    public class NavSatFixTx : ProBridgeTxStamped<sensor_msgs.msg.NavSatFix>
    {
        // WGS84 ellipsoid
        private const double WGS84_A = 6378137.0;
        private const double WGS84_F = 1.0 / 298.257223563;
        private const double WGS84_B = WGS84_A * (1.0 - WGS84_F);
        private const double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);
        private const double WGS84_EP2 = WGS84_E2 / (1.0 - WGS84_E2);
        private const double Deg2Rad = Math.PI / 180.0;
        private const double Rad2Deg = 180.0 / Math.PI;

        [Header("Params")]
        [Tooltip("Origin of the local frame: start LLA is placed here, local x = east, y = up, z = north. " +
                 "If empty, the position of this sensor at the first enable is used (world axes).")]
        public Transform initOrigin;
        [Tooltip("Latitude of the origin, degrees")]
        public double startLatitude;
        [Tooltip("Longitude of the origin, degrees")]
        public double startLongitude;
        [Tooltip("Altitude of the origin above the WGS84 ellipsoid, meters")]
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
        private bool startPosSet;

        protected override void AfterEnable()
        {
            if (!startPosSet)
            {
                startPos = transform.position;
                startPosSet = true;
            }
            UpdateLLA();
        }

        private void Update()
        {
            UpdateLLA();
        }

        private void UpdateLLA()
        {
            Vector3 local = initOrigin != null
                ? initOrigin.InverseTransformPoint(transform.position)
                : transform.position - startPos;

            var ecef = UnityToEcef(local, startLatitude, startLongitude, startAltitude);
            var lla = Epsg4978ToEpsg4979(ecef.x, ecef.y, ecef.z);
            latitude = lla.lat;
            longitude = lla.lon;
            altitude = lla.alt;
            if (applyNoise)
            {
                latitude += GaussianNoise.Generate(latitudeNoiseStdDev);
                longitude += GaussianNoise.Generate(longitudeNoiseStdDev);
                altitude += GaussianNoise.Generate(altitudeNoiseStdDev);
            }
        }

        /// <summary>
        /// Unity local position (x = east, y = up, z = north) on the plane tangent to the ellipsoid
        /// at the origin LLA -> ECEF (EPSG:4978).
        /// </summary>
        public static (double x, double y, double z) UnityToEcef(Vector3 local, double lat0, double lon0, double alt0)
        {
            var o = Epsg4979ToEpsg4978(lat0, lon0, alt0);

            double sphi = Math.Sin(lat0 * Deg2Rad), cphi = Math.Cos(lat0 * Deg2Rad);
            double slam = Math.Sin(lon0 * Deg2Rad), clam = Math.Cos(lon0 * Deg2Rad);
            double e = local.x, n = local.z, u = local.y;

            return (
                o.x - slam * e - sphi * clam * n + cphi * clam * u,
                o.y + clam * e - sphi * slam * n + cphi * slam * u,
                o.z + cphi * n + sphi * u
            );
        }

        /// <summary>
        /// Geodetic WGS84 LLA (EPSG:4979, degrees / meters) -> ECEF (EPSG:4978, meters).
        /// </summary>
        public static (double x, double y, double z) Epsg4979ToEpsg4978(double lat, double lon, double alt)
        {
            double sphi = Math.Sin(lat * Deg2Rad), cphi = Math.Cos(lat * Deg2Rad);
            double slam = Math.Sin(lon * Deg2Rad), clam = Math.Cos(lon * Deg2Rad);
            double n = WGS84_A / Math.Sqrt(1.0 - WGS84_E2 * sphi * sphi);

            return (
                (n + alt) * cphi * clam,
                (n + alt) * cphi * slam,
                (n * (1.0 - WGS84_E2) + alt) * sphi
            );
        }

        /// <summary>
        /// ECEF (EPSG:4978, meters) -> geodetic WGS84 LLA (EPSG:4979, degrees / meters).
        /// Bowring's formula, sub-millimeter for terrestrial heights.
        /// </summary>
        public static (double lat, double lon, double alt) Epsg4978ToEpsg4979(double x, double y, double z)
        {
            double p = Math.Sqrt(x * x + y * y);
            double th = Math.Atan2(z * WGS84_A, p * WGS84_B);
            double sth = Math.Sin(th), cth = Math.Cos(th);

            double phi = Math.Atan2(z + WGS84_EP2 * WGS84_B * sth * sth * sth,
                                    p - WGS84_E2 * WGS84_A * cth * cth * cth);
            double lam = Math.Atan2(y, x);

            double sphi = Math.Sin(phi), cphi = Math.Cos(phi);
            double n = WGS84_A / Math.Sqrt(1.0 - WGS84_E2 * sphi * sphi);
            // near the poles p / cos(phi) is unstable, use z instead
            double alt = Math.Abs(cphi) > 1e-6
                ? p / cphi - n
                : z / sphi - n * (1.0 - WGS84_E2);

            return (phi * Rad2Deg, lam * Rad2Deg, alt);
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
