using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.IO;
using System.Globalization;
using CesiumLanguageWriter;
using System.Net.Http;
using MathNet.Numerics.LinearAlgebra;
using System.Net.Http.Headers;

namespace RamsesSniffer
{
    public partial class Form1 : Form
    {

        string testString = "";
        string TemperatureTest = "";

        string RamsesNetworkcardip = "10.101.9.53";
        static string PusherNetworkcardip = "10.101.9.53";
        private static IPEndPoint pusherEndPoint;

        //UdpClient for Posnet data
        string POSNET_UDPip = "224.100.100.102";
        string POSNET_UDPport = "5600";
        private Socket POSNET_udpSock;
        private byte[] POSNET_buffer;

        //UdpClient for Ramses data. MAXUS
        string RAMSES_UDPip = "239.255.4.2";
        string RAMSES_UDPport = "55402";
        private Socket RAMSES_udpSock;
        private byte[] RAMSES_buffer;

        private bool NewDataFlag = false;

        private Int64 pkgCounter = 0;

        //Lists in which to store the extracted data
        private List<GPSposition> GPSpositionsReceived = new List<GPSposition>();
        private List<GPSposition> OldGPSpositionsReceived = new List<GPSposition>();
        private List<GPS_IIP> GPS_IIP_Received = new List<GPS_IIP>();
        private List<Attitude> AttitudeReceived = new List<Attitude>();
        private List<Attitude> OldAttitudeReceived = new List<Attitude>();
        private List<GLoads> GLoadsReceived = new List<GLoads>();
        private List<GLoads> OldGLoadsReceived = new List<GLoads>();
        private List<AngularRates> ARatesReceived = new List<AngularRates>();
        private List<AngularRates> OldARatesReceived = new List<AngularRates>();

        private int maxGPSIndex = 0;
        private int maxGPS_IIP_Index = 0;
        private int maxAttitudeIndex = 0;
        private int maxGLoadIndex = 0;
        private int maxARateIndex = 0;

        //Identifiers for different packet types
        private PacketIdentifier PacketID_GPSpos_Fix = new PacketIdentifier(0x08, 0x20, 0x00, 0x01, "GPS packet with a valid position");
        private PacketIdentifier PacketID_GPSpos_NoFix = new PacketIdentifier(0x08, 0x20, 0x00, 0x01, "GPS packet without valid position");
        private PacketIdentifier PacketID_GPSIIP_valid = new PacketIdentifier(0x08, 0x21, 0x00, 0x01, "IIP packet with valid navigation data");
        private PacketIdentifier PacketID_GPSIIP_invalid = new PacketIdentifier(0x08, 0x21, 0x00, 0x00, "IIP packet without navigation data");
        private PacketIdentifier PacketID_GCS = new PacketIdentifier(0x08, 0x30, -1, -1, "GCS 50Hz data");
        private PacketIdentifier PacketID_PCDU = new PacketIdentifier(0x08, 0x0D, 0x00, -1, "PCDU data");
        private PacketIdentifier PacketID_THERM = new PacketIdentifier(0x08, 0x10, -1, 0x01, "Thermo board data packet");
        private List<GPS_IIP> NewGPS_IIP_Received = new List<GPS_IIP>();

        /*--------------------------------------------------------------------*/
        /*----------------------------Data pusher-----------------------------*/

        // WGS ellipsoid data
        private static double eccentricity = 0.081819190842622;
        private static double semiMajorAxis = 6378137;
        private Vector<double> xAxis = Vector<double>.Build.DenseOfArray(new double[3] { 1, 0, 0 });
        private Vector<double> yAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 1, 0 });
        private Vector<double> zAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 0, 1 });

        // Constant used for converting degrees to radians
        private static double deg2rad = Math.PI / 180;

        //Variables used to store the last known data points
        private JulianDate oldPositionTime = new JulianDate();
        private Cartographic oldCartographic = new Cartographic(1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private UnitQuaternion oldRealQuaternion = new UnitQuaternion(1, 1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private UnitQuaternion oldCalculatedQuaternion = new UnitQuaternion(1, 1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private double oldSpeed = 0; // Used to store the latest calculated speed.

        //Bools that indicate whether data has been received yet or not
        private bool oldGPSDataExists = false;
        private bool oldRealAttitudeDataExists = false;
        private bool oldCalculatedAttitudeDataExists = false;

        // Check if mission time goes from minus to plus
        private string plusminusTime = "-";

        // List of events
        private int[] eventTimeList = new int[] { -9999, -300, -240, -150, -70, -65, -60, -60, -57, -47, -40, -25, -20, -16, -14, -9, -8, 0, 64, 86, 88, 91, 160, 230, 800, 845 };
        private string[] eventStringList = new string[] { "Server synchronized with data pusher", "Rocket system in 'flight mode'", "TGV/INA on INTERNAL power", "360 Recording started", "XRMON-DIFF2 Internal power (module)", "XRMON-DIFF2 Internal power (furnace 1)", "XRMON-DIFF2 Internal power (furnace 2)", "Launch sequencer activation", "TVA command check", "TVA check (unpressurised)", "Charging pressurising pyro capacitors", "Verifying launcher status and arming TVA", "TVA Pressurising", "TVA pressure check", "TVA Deflection test", "TVA Baseline Retrace test", "Go signal", "Lift-off", "Expected motor burn-out", "Expected motor separation", "XRMON-DIFF2 Image sequence start", "Expected Super-Max separation", "XRMON-DIFF2 Alumina shearing", "XRMON-DIFF2 Graphite shearing", "XRMON-DIFF2 Experiment power down", "Castor 4B destruction" };

        // The frequency of which we will push data to the visualization server.
        private static double pushFrequency = 2;

        // Only sending data that are updated, and "static" data are sent in the first packets
        private bool firstPositionPacket = true;
        private bool firstOrientationPacket = true;

        // Setting the initial coordinates of the rocket
        private double initialLatitude = 67.893332; // Latitude of the topocentric horizon system
        private double initialLongitude = 21.103927; // Longitude of the topocentric horizon system

        // Time variables
        private DateTime currentDate = DateTime.Now;
        private JulianDate launchTime;
        private JulianDate baseTimeTimeStamp; // Base time with redards to the time stamped time
        private JulianDate baseTimeNow; // Base time with redargs to current system time at the time of creating the baseTimeTimeStamp
        private double missionSeconds; // The mission time in seconds
        private string oldRTtimestr = "";
        private bool launched = false; // Bool that indicates if the rocket has launched

        // Bool used to see if the first GPS data is processed. The first GPS data point will define the base times
        private bool firstGpsDataProcessed = false;

        // The path to the server which we push data to.
        private static string serverPath = "http://localhost:8080/czml";
        //public static string serverPath = "https://sscflightdata.herokuapp.com/czml";

        /*--------------------------------------------------------------------*/
        /*--------------------------------------------------------------------*/

        /*-------------TEST VARIABLES-------------------*/
        private double testLong = 21.1068944441480;
        private double testLat = 67.8932469830635;
        private double testAlt = 0;
        private double testSpeed = 0;
        private string testTime = "+00;00;00";
        private GLoads testGLoad;
        private AngularRates testARates;
        private Random rand;
        private StreamReader CSVreader = new StreamReader(File.OpenRead("VSB-predicted-flight.csv"));
        private int linecounter = 0;
        /*----------------------------------------------*/

        // The available modes
        private string[] modes = new string[2] { "Live mode", "Test mode" };


        class PacketIdentifier
        {
            public byte PacketID = 0;
            public byte PacketIdAPID = 0;
            public int SID1 = 0;
            public int SID2 = 0;
            public string Description = "";

            public PacketIdentifier(byte PacketID_inp, byte PacketIdAPID_inp, int SID1_inp_KEY4, int SID2_inp_KEY5, string DescriptionStr)
            {
                PacketID = PacketID_inp;
                PacketIdAPID = PacketIdAPID_inp;
                SID1 = SID1_inp_KEY4;
                SID2 = SID2_inp_KEY5;
                Description = DescriptionStr;
            }

        }

        class GPSposition
        {
            public double Longitude = 0;
            public double Latitude = 0;
            public double Altitude = 0;
            public JulianDate RealTime = new JulianDate(); // This is the real time stamp of the data point
            public JulianDate SimulatedTime = new JulianDate(); // This is the time stamp based on the system time, should NOT be used for speed calculations
            public string RAWmessage = "";
            public int Satellites = 0;

            public GPSposition(double longitude, double latitude, double altitude, JulianDate simulatedTime, JulianDate realTime, string raw, int satellites)
            {
                Longitude = longitude;
                Latitude = latitude;
                Altitude = altitude - 330; // Subtracting 330m in order to compensate for the difference between the WGS84 ellipsoid surface and GPS altitude
                RealTime = realTime;
                SimulatedTime = simulatedTime;
                RAWmessage = raw;
                Satellites = satellites;
            }

            public Vector<double> toCartesian()
            {
                return GPS2cartesian(Longitude, Latitude, Altitude);
            }
        }

        class GPS_IIP
        {
            public double Longitude = 0;
            public double Latitude = 0;
            public JulianDate Time = new JulianDate();
            public string RAWmessage = "";
            public double TimeToImpact = 0;

            public GPS_IIP(double longitude, double latitude, JulianDate time, string raw, double timeToImpact)
            {
                Longitude = longitude;
                Latitude = latitude;
                Time = time;
                RAWmessage = raw;
                TimeToImpact = timeToImpact;
            }

            // Converting the coordinates to a Cartographic property that Cesium can recognize
            public Cartographic toCartographic()
            {
                return new Cartographic(Longitude, Latitude, 0);
            }
        }

        class Attitude
        {
            public float q0 = 0;
            public float q1 = 0;
            public float q2 = 0;
            public float q3 = 0;
            public JulianDate Time = new JulianDate();

            public Attitude(float Q0, float Q1, float Q2, float Q3, JulianDate timeStamp)
            {
                q0 = Q0;
                q1 = Q1;
                q2 = Q2;
                q3 = Q3;
                Time = timeStamp;
            }

            public override string ToString()
            {
                return q0.ToString("0.00") + "," + q1.ToString("0.00") + "," + q2.ToString("0.00") + "," + q3.ToString("0.00");
            }
        }

        class GLoads
        {
            public float Gx = 0;
            public float Gy = 0;
            public float Gz = 0;

            public GLoads(float GX, float GY, float GZ)
            {
                Gx = GX;
                Gy = GY;
                Gz = GZ;
            }

            // Converting to cartesian which will be used to send the 3-component property to Cesium
            public Cartesian toCartesian()
            {
                return new Cartesian(Gx, Gy, Gz);
            }
        }

        class AngularRates
        {
            public double aRateX = 0;
            public double aRateY = 0;
            public double aRateZ = 0;

            public AngularRates(double ARateX, double ARateY, double ARateZ)
            {
                aRateX = ARateX;
                aRateY = ARateY;
                aRateZ = ARateZ;
            }

            // Converting to cartesian which will be used to send the 3-component property to Cesium
            public Cartesian toCartesian()
            {
                return new Cartesian(aRateX, aRateY, aRateZ);
            }
        }

        string message = "";
        string RTtimestr = "-99:00:00";

        public Form1()
        {
            InitializeComponent();

            //get a list of all availible network adapters
            foreach (string s in net_adapters())
            {
                comboBox1.Items.Add(s);
            }

            // Adding the modes to a combobox
            foreach (string s in modes)
            {
                comboBox2.Items.Add(s);
            }

            // Default combobox selection
            comboBox1.SelectedIndex = 0;
            comboBox2.SelectedIndex = 0;
        }

        private bool Packetmatch(byte[] buffer, PacketIdentifier PI)
        {
            bool result = false;
            bool SID1ok = false;
            bool SID2ok = false;

            //Correct ID and APID
            if (buffer[32] == PI.PacketID && buffer[32 + 1] == PI.PacketIdAPID)
            {
                //Check SID1
                if (PI.SID1 != -1)
                {
                    if (buffer[32 + 15] == Convert.ToByte(PI.SID1))
                    {
                        SID1ok = true;
                    }
                }
                else
                {
                    SID1ok = true;
                }

                //Check SID2
                if (PI.SID2 != -1)
                {
                    if (buffer[32 + 16] == Convert.ToByte(PI.SID2))
                    {
                        SID2ok = true;
                    }
                }
                else
                {
                    SID2ok = true;
                }

                //Passed all checks
                if (SID1ok && SID2ok)
                {
                    result = true;
                }
                //Check SID

            }

            return result;
        }

        public List<String> net_adapters()
        {
            List<String> values = new List<String>();



            IPAddress[] IPS = Dns.GetHostAddresses(Dns.GetHostName());
            foreach (IPAddress ip in IPS)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    values.Add(ip.ToString());
                }
            }

            return values;
        }

        private void BindUPD()
        {
            // Take the ip-adress of the selected item in the dropdown box
            RamsesNetworkcardip = comboBox1.SelectedItem.ToString();

            POSNET_udpSock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            try
            {
                pusherEndPoint = new IPEndPoint(IPAddress.Parse(PusherNetworkcardip), 0);

                // Open the UDP port and listen to IP as specified in config.ini
                IPEndPoint ipep = new IPEndPoint(IPAddress.Parse(RamsesNetworkcardip), int.Parse(POSNET_UDPport)); //Use the network card with this adress
                POSNET_udpSock.Bind(ipep);

                IPAddress ip = IPAddress.Parse(POSNET_UDPip);

                POSNET_udpSock.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership, new MulticastOption(ip, IPAddress.Any));

                POSNET_buffer = new byte[5000];

                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                POSNET_udpSock.BeginReceiveFrom(POSNET_buffer, 0, POSNET_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromPosnet, POSNET_udpSock);

                add2listBox("UDP bind");
            }
            catch (Exception)
            {
                MessageBox.Show("UDP socket failed!");
            }

            //Bind the RAMSES UDP
            RAMSES_udpSock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            try
            {
                RAMSES_udpSock.ReceiveBufferSize = 2 * 1024 * 1024; // 2 megabytes

                //Open the UDP port and listen to IP as specified in config.ini
                IPEndPoint ipep = new IPEndPoint(IPAddress.Parse(RamsesNetworkcardip), int.Parse(RAMSES_UDPport)); //Use the network card with this adress
                RAMSES_udpSock.Bind(ipep);

                IPAddress ip = IPAddress.Parse(RAMSES_UDPip);

                RAMSES_udpSock.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership, new MulticastOption(ip, IPAddress.Any));

                RAMSES_buffer = new byte[50000];

                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                RAMSES_udpSock.BeginReceiveFrom(RAMSES_buffer, 0, RAMSES_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromRamses, RAMSES_udpSock);
            }
            catch (Exception)
            {
                MessageBox.Show("UDP socket failed!");
            }
        }

        static public List<int> SearchBytePattern(byte[] pattern, byte[] bytes)
        {
            List<int> positions = new List<int>();

            try
            {
                int patternLength = pattern.Length;
                int totalLength = bytes.Length;
                byte firstMatchByte = pattern[0];
                for (int i = 0; i < totalLength; i++)
                {
                    if (firstMatchByte == bytes[i] && totalLength - i >= patternLength)
                    {
                        byte[] match = new byte[patternLength];
                        Array.Copy(bytes, i, match, 0, patternLength);
                        if (match.SequenceEqual<byte>(pattern))
                        {
                            positions.Add(i);
                            i += patternLength - 1;
                        }
                    }
                }
            }
            catch
            {
            }

            return positions;
        }


        //Asynchronous receive from UDP Posnet
        private void DoReceiveFromPosnet(IAsyncResult iar)
        {
            pkgCounter++;

            try
            {
                //Get the received message.
                Socket recvSock = (Socket)iar.AsyncState;
                EndPoint clientEP = new IPEndPoint(IPAddress.Any, 0);
                int msgLen = recvSock.EndReceiveFrom(iar, ref clientEP);

                //Start listening for a new message.
                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                POSNET_udpSock.BeginReceiveFrom(POSNET_buffer, 0, POSNET_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromPosnet, POSNET_udpSock);

                message = Encoding.UTF8.GetString(POSNET_buffer);

                //Look for RT time packet on posnet
                if (message.Substring(0, 7) == "$PRBERT")
                {
                    RTtimestr = message.Substring(29, 11);
                    StringBuilder sb = new StringBuilder(RTtimestr);
                    sb[sb.Length - 2] = ' ';
                    RTtimestr = sb.ToString();
                }

                NewDataFlag = true;
            }
            catch (ObjectDisposedException)
            {
                add2listBox("exception");
                //expected termination exception on a closed socket.
                // ...I'm open to suggestions on a better way of doing this.
            }
        }

        //Asynchronous receive from UDP Ramses
        private void DoReceiveFromRamses(IAsyncResult iar)
        {
            pkgCounter++;

            try
            {
                //Get the received message.
                Socket recvSock = (Socket)iar.AsyncState;
                EndPoint clientEP = new IPEndPoint(IPAddress.Any, 0);
                int msgLen = recvSock.EndReceiveFrom(iar, ref clientEP);

                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                RAMSES_udpSock.BeginReceiveFrom(RAMSES_buffer, 0, RAMSES_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromRamses, RAMSES_udpSock);

                ProcessRAMSESmessage(RAMSES_buffer);

                NewDataFlag = true;
            }
            catch (ObjectDisposedException)
            {
                MessageBox.Show("error");
                //add2listBox("exception");
                //expected termination exception on a closed socket.
                // ...I'm open to suggestions on a better way of doing this.
            }
        }

        private double DDMToDD(string d = "", string m = "", string s = "")
        {

            double multiplier = 1;
            if (s.Contains('S') || s.Contains('W'))
            {
                multiplier = -1;
            }

            double inputDegrees = Convert.ToDouble(d);
            double inputMinutes = double.Parse(m, CultureInfo.InvariantCulture);
            double latitude = inputDegrees + (inputMinutes / 60) * multiplier;

            return latitude;
        }



        private void ProcessRAMSESmessage(byte[] RAMSES_buffer)
        {
            try
            {
                double longitud = 0;
                double latitude = 0;
                double altitude = 0;
                string msg = "";
                string time = "";

                //If it is a GPS packet with valid position
                if (Packetmatch(RAMSES_buffer, PacketID_GPSpos_Fix))
                {
                    msg = Encoding.UTF8.GetString(RAMSES_buffer, (32 + 17), 104);
                    time = msg.Substring(16, 9);

                    latitude = DDMToDD(msg.Substring(26, 2), msg.Substring(28, 8), msg.Substring(37, 1));
                    longitud = DDMToDD(msg.Substring(39, 3), msg.Substring(42, 8), msg.Substring(51, 1));
                    altitude = double.Parse(msg.Substring(53, 10), CultureInfo.InvariantCulture);
                    int satellites = Convert.ToInt16(msg.Substring(13, 2));

                    DateTime realTimeStamp = new DateTime(currentDate.Year, currentDate.Month, currentDate.Day, Convert.ToInt32(time.Substring(0, 2)), Convert.ToInt32(time.Substring(2, 2)), Convert.ToInt32(time.Substring(4, 2)), 10 * Convert.ToInt32(time.Substring(7, 2)));
                    DateTime simulatedTimeStamp = DateTime.Now;

                    GPSposition p = new GPSposition(longitud, latitude, altitude, new JulianDate(simulatedTimeStamp), new JulianDate(realTimeStamp), msg, satellites);
                    GPSpositionsReceived.Add(p);

                    if (!firstGpsDataProcessed)
                    {
                        baseTimeTimeStamp = new JulianDate(realTimeStamp);
                        baseTimeNow = new JulianDate(simulatedTimeStamp);
                        firstGpsDataProcessed = true;
                    }

                }
                else if (Packetmatch(RAMSES_buffer, PacketID_GPSIIP_valid))//IIP packet
                {
                    msg = Encoding.UTF8.GetString(RAMSES_buffer, (32 + 17), 59);
                    time = msg.Substring(11, 9); //Time for IIP calculation

                    latitude = DDMToDD(msg.Substring(23, 2), msg.Substring(25, 7), msg.Substring(33, 1));
                    longitud = DDMToDD(msg.Substring(35, 3), msg.Substring(38, 7), msg.Substring(46, 1));

                    //Time to impact, seconds
                    double timeToImpact = double.Parse(msg.Substring(48, 7), CultureInfo.InvariantCulture);

                    DateTime realTimeStamp = new DateTime(currentDate.Year, currentDate.Month, currentDate.Day, Convert.ToInt32(time.Substring(0, 2)), Convert.ToInt32(time.Substring(2, 2)), Convert.ToInt32(time.Substring(4, 2)), 10 * Convert.ToInt32(time.Substring(7, 2)));
                    DateTime timeStamp = DateTime.Now;

                    GPS_IIP IIP = new GPS_IIP(longitud, latitude, new JulianDate(timeStamp), msg, timeToImpact);
                    GPS_IIP_Received.Add(IIP);
                }
                else if (Packetmatch(RAMSES_buffer, PacketID_GCS)) //If it is a GCS packet
                {
                    byte[] bquaternion0 = new byte[4];
                    byte[] bquaternion1 = new byte[4];
                    byte[] bquaternion2 = new byte[4];
                    byte[] bquaternion3 = new byte[4];

                    Array.Copy(RAMSES_buffer, 49 + 30, bquaternion0, 0, 4);
                    Array.Copy(RAMSES_buffer, 49 + 34, bquaternion1, 0, 4);
                    Array.Copy(RAMSES_buffer, 49 + 38, bquaternion2, 0, 4);
                    Array.Copy(RAMSES_buffer, 49 + 42, bquaternion3, 0, 4);

                    JulianDate now = new JulianDate(DateTime.Now);

                    if (firstGpsDataProcessed)
                    {
                        JulianDate timeStamp = baseTimeTimeStamp.AddSeconds(baseTimeNow.SecondsDifference(now));
                        Attitude at = new Attitude(floatConversion(bquaternion0), floatConversion(bquaternion1), floatConversion(bquaternion2), floatConversion(bquaternion3), timeStamp);

                        AttitudeReceived.Add(at);
                    }
                }
                else if (Packetmatch(RAMSES_buffer, PacketID_PCDU)) //If it is a PCDU packet
                {
                    var ACCx = (short)(RAMSES_buffer[32 + 105 + 0] << 8 | RAMSES_buffer[32 + 105 + 1]);
                    var ACCy = (short)(RAMSES_buffer[32 + 107 + 0] << 8 | RAMSES_buffer[32 + 107 + 1]);
                    var ACCz = (short)(RAMSES_buffer[32 + 109 + 0] << 8 | RAMSES_buffer[32 + 109 + 1]);

                    //Get gyro data
                    var GyroX = (short)(RAMSES_buffer[32 + 111 + 0] << 8 | RAMSES_buffer[32 + 111 + 1]);
                    var GyroY = (short)(RAMSES_buffer[32 + 113 + 0] << 8 | RAMSES_buffer[32 + 113 + 1]);
                    var GyroZ = (short)(RAMSES_buffer[32 + 115 + 0] << 8 | RAMSES_buffer[32 + 115 + 1]);

                    //Get onboard mission time
                    byte[] bmissiontime = new byte[4];
                    Array.Copy(RAMSES_buffer, 32 + 43, bmissiontime, 0, 4);

                    Int32 missiontime = IntegerFromByteArray(bmissiontime); //MAXUS 9 sequence starts at T-300000ms

                    //Get input signals, motor separation etc
                    byte PIn = RAMSES_buffer[32 + 143];
                    string SepState = "";

                    //Check motor separation
                    if (BitIsSet(PIn, 4))
                    {
                        SepState = "Motor is separated from payload";

                    }
                    else
                    {
                        SepState = "Motor is attatched to payload";
                    }

                    testString = ACCx.ToString() + "mG, " + ACCy.ToString() + "mG, " + ACCz.ToString() + "mG" + " " + SepState + ", time:" + missiontime.ToString() + "ms";

                    GLoadsReceived.Add(new GLoads((float)ACCx / 1000, (float)ACCy / 1000, (float)ACCz / 1000));

                    // Scale in order to get degrees per second
                    double scaleFactor = 0.00037 * 360;
                    ARatesReceived.Add(new AngularRates(GyroX * scaleFactor, GyroY * scaleFactor, GyroZ * scaleFactor));

                }
                else if (Packetmatch(RAMSES_buffer, PacketID_THERM)) //If it is a THERM packet. Extract some temperatures maybe?
                {
                    var TGVTemp3 = (short)(RAMSES_buffer[32 + 65 + 0] << 8 | RAMSES_buffer[32 + 65 + 1]);

                    //Use a simplified PT-100 curve
                    double temp = 0.05 * (double)(TGVTemp3 - 5568);
                    TemperatureTest = temp.ToString("0.0") + "°C";
                }


            }
            catch (Exception except)
            {
                add2listBox("error!");
                //MessageBox.Show(except.StackTrace.ToString());
                //RAMSES decompile error
            }

        }

        int IntegerFromByteArray(byte[] data)
        {
            //Data saved as little endian
            Array.Reverse(data);
            return BitConverter.ToInt32(data, 0);
        }

        public float floatConversion(byte[] bytes)
        {
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes); // Convert big endian to little endian
            }
            float myFloat = BitConverter.ToSingle(bytes, 0);
            return myFloat;
        }

        public static bool BitIsSet(byte b, int bitNumber)
        {
            var bit = (b & (1 << bitNumber - 1)) != 0;
            return bit;
        }

        private void UpdateGUI()
        {
            try
            {
                //Time string from RT counter
                label1.Text = RTtimestr;

                //GPS position messages
                if (GPSpositionsReceived.Count > 0)
                {
                    label3.Text = "GPS: " + GPSpositionsReceived.Count.ToString();
                    label4.Text = GPSpositionsReceived[GPSpositionsReceived.Count - 1].RealTime.ToDateTime().ToString();

                    add2listBox(GPSpositionsReceived[GPSpositionsReceived.Count - 1].RAWmessage);
                }

                //Attitude messages
                if (AttitudeReceived.Count > 0)
                {
                    label6.Text = "GCS: " + AttitudeReceived.Count.ToString();
                    label7.Text = "qt: " + AttitudeReceived[AttitudeReceived.Count - 1].ToString();

                    add2listBox(AttitudeReceived[AttitudeReceived.Count - 1].ToString());
                }

                try
                {
                    //Number of received RAMSES packages
                    label8.Text = "pkg: " + pkgCounter.ToString();
                }
                catch (Exception)
                {

                }


                if (GPS_IIP_Received.Count > 0)
                {
                    label10.Text = "GPS IIP: " + GPS_IIP_Received.Count.ToString() + ", " + GPS_IIP_Received[GPS_IIP_Received.Count - 1].Time.ToString();
                }

                label9.Text = testString + ", " + TemperatureTest;

                //Selects the last item in list
                if (listBox1.Items.Count > 0)
                {
                    listBox1.SelectedIndex = listBox1.Items.Count - 1;
                }
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to update GUI: " + except.StackTrace.ToString());
            }

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (NewDataFlag == true && GUIUpdateChecked.Checked)
            {
                UpdateGUI();
                NewDataFlag = false;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (comboBox2.SelectedIndex == 0)
            {
                BindUPD();
            }
            else if (comboBox2.SelectedIndex == 1)
            {
                rand = new Random();
                System.Timers.Timer testTimer = new System.Timers.Timer();
                testTimer.Elapsed += new System.Timers.ElapsedEventHandler(onTestEvent);
                testTimer.Interval = 100;
                testTimer.Enabled = true;
                testTimer.AutoReset = true;
                testTimer.Start();
            }

            // Initialize the data push:
            pushInitialPacket();

            // If test mode is chosen

            System.Timers.Timer pushTimer = new System.Timers.Timer();
            pushTimer.Elapsed += new System.Timers.ElapsedEventHandler(onPushEvent);
            pushTimer.Interval = 1000 / pushFrequency;
            pushTimer.Enabled = true;
            pushTimer.AutoReset = true;
            pushTimer.SynchronizingObject = button1;
            pushTimer.Start();
        }

        /* onTestEvent is used to test the functionaliny of the pusher without having to use real network data. */
        private void onTestEvent(object sender, System.Timers.ElapsedEventArgs e)
        {
            System.Threading.Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
            if (linecounter > 9)
            {
                try
                {
                    double secondsSince = linecounter * 0.1;

                    double minutes = Math.Truncate(secondsSince / 60);
                    double seconds = Math.Truncate(secondsSince % 60);
                    string secondsString;
                    string minutesString;

                    if (Math.Abs(seconds) < 10)
                    {
                        secondsString = "0" + Math.Abs(seconds).ToString();
                    }
                    else
                    {
                        secondsString = Math.Abs(seconds).ToString();
                    }

                    if (Math.Abs(minutes) < 10)
                    {
                        minutesString = "0" + Math.Abs(minutes).ToString();
                    }
                    else
                    {
                        minutesString = Math.Abs(minutes).ToString();
                    }

                    if (secondsSince >= 0)
                    {
                        RTtimestr = "+00:" + minutesString + ":" + secondsString + " C";
                    }
                    else
                    {
                        RTtimestr = "-00:" + minutesString + ":" + secondsString + " C";
                    }

                    if (secondsSince > 0)
                    {
                        var line = CSVreader.ReadLine();
                        var values = line.Split(';');

                        testLong = Convert.ToDouble(values[16]);
                        testLat = Convert.ToDouble(values[17]);
                        testAlt = Convert.ToDouble(values[39]) * 1000;

                        DateTime timeStamp = DateTime.Now;
                        if (GPSTestCheckBox.Checked)
                        {
                            GPSposition p = new GPSposition(testLong, testLat, testAlt, new JulianDate(timeStamp), new JulianDate(timeStamp), "msg", 11);
                            GPSpositionsReceived.Add(p);
                        }

                        if (attitudeTestCheckBox.Checked)
                        {
                            Vector<double> testQuat = Vector<double>.Build.Dense(4);

                            testQuat[0] = rand.NextDouble();
                            testQuat[1] = rand.NextDouble();
                            testQuat[2] = rand.NextDouble();
                            testQuat[3] = rand.NextDouble();

                            testQuat = testQuat.Normalize(2);

                            AttitudeReceived.Add(new Attitude((float)testQuat[0], (float)testQuat[1], (float)testQuat[2], (float)testQuat[3], new JulianDate(DateTime.Now)));
                        }


                        if (gLoadTestCheckBox.Checked)
                        {
                            GLoads gLoads = new GLoads(float.Parse(values[41]), float.Parse(values[42]), float.Parse(values[43]));
                            GLoadsReceived.Add(gLoads);
                        }

                        if (aRateTestCheckBox.Checked)
                        {
                            testARates = new AngularRates((float)rand.NextDouble() * 100, (float)rand.NextDouble() * 100, (float)rand.NextDouble() * 100);
                            ARatesReceived.Add(testARates);
                        }

                        if (IIPTestCheckbox.Checked)
                        {
                            GPS_IIP testIIP = new GPS_IIP(20.6791, 68.5443, new JulianDate(timeStamp), "what", 20);
                            GPS_IIP_Received.Add(testIIP);
                        }
                    }


                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to push test data: " + except.StackTrace.ToString());
                }



                linecounter++;
                NewDataFlag = true;
            }
            else
            {
                var line = CSVreader.ReadLine();
                var values = line.Split(';');
                linecounter++;
            }
        }

        /* Pushing the initial packet to the server. */
        private void pushInitialPacket()
        {
            StringWriter sw = new StringWriter();
            CesiumOutputStream output = new CesiumOutputStream(sw);
            CesiumStreamWriter writer = new CesiumStreamWriter();
            ClockCesiumWriter clock;
            PacketCesiumWriter packet;

            output.PrettyFormatting = true;

            // First bracket in the array
            output.WriteStartSequence();

            // Add a new packet
            packet = writer.OpenPacket(output);

            // Writing the document package, which is the first package that the client must receive
            packet.WriteId("document");
            packet.WriteName("Ver!WrdZ<?");
            packet.WriteVersion("1.0");

            // Close first package
            packet.Close();

            // Closing bracket
            output.WriteEndSequence();

            //Populating the byte array with the data written
            byte[] byteArray = Encoding.UTF8.GetBytes(sw.ToString());

            // Posting the data to the server
            var responseStatus = postData(byteArray);

            // Closing the string writer
            sw.Close();
        }

        private void add2listBox(string listBoxStr)
        {
            try
            {
                if (GUIUpdateChecked.Checked)
                {
                    listBox1.Invoke((MethodInvoker)(() => listBox1.Items.Add(listBoxStr)));
                }
                if (listBox1.Items.Count > 500)
                {
                    listBox1.Invoke((MethodInvoker)(() => listBox1.Items.RemoveAt(0)));
                }
            }
            catch (Exception except)
            {
                MessageBox.Show("Writing to listBox1 failed: " + except.StackTrace.ToString());
            }
        }

        /* onPushedEvent handles the pushing of data to the server. Pushes should not be more frequent than 2 Hz. */
        private void onPushEvent(object sender, System.Timers.ElapsedEventArgs e)
        {
            System.Diagnostics.Stopwatch watch_tot = new System.Diagnostics.Stopwatch();

            watch_tot.Start();

            StringWriter sw = null;
            CesiumOutputStream output = null;
            CesiumStreamWriter writer = null;
            PacketCesiumWriter packet = null;

            try
            {
                sw = new StringWriter();
                output = new CesiumOutputStream(sw);
                writer = new CesiumStreamWriter();
                packet = new PacketCesiumWriter();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to assign writers: " + except.StackTrace.ToString());
            }

            PositionCesiumWriter position = null;
            PositionCesiumWriter gLoadPacket = null; // Will write the g-forces as the coordinates of an abstract point
            PositionCesiumWriter aRatePacket = null; // Will write the angular rates as the coordinates of an abstract point
            PositionCesiumWriter IIPPacket = null;
            OrientationCesiumWriter orientation = null;
            ModelCesiumWriter model = null;
            PointCesiumWriter point = null;
            PathCesiumWriter path = null;
            PolylineCesiumWriter polyLine = null;

            Duration extrapolationDuration = new Duration(0, 600);

            // Measure performance
            System.Diagnostics.Stopwatch watch_listBox = new System.Diagnostics.Stopwatch();
            watch_listBox.Start();
            //add2listBox("A new data push was initialized");

            watch_listBox.Stop();
            //add2listBox("------------------------------------Time of listBox execution: " + watch_listBox.ElapsedMilliseconds.ToString());

            // Ensure that the comma conventions are correct
            System.Threading.Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

            // Check if the properties are updated
            bool GPSUpdated = false;
            bool attitudeUpdated = false;
            bool speedUpdated = false;
            bool aRatesUpdated = false;
            bool gLoadsUpdated = false;
            bool IIPUpdated = false;
            bool timeUpdated = false;

            int GPSStartIndex = 0;
            int GPSEndIndex = 0;
            int GLoadStartIndex = 0;
            int GLoadEndIndex = 0;
            int AttitudeStartIndex = 0;
            int AttitudeEndIndex = 0;
            int ARateStartIndex = 0;
            int ARateEndIndex = 0;

            List<GPSposition> NewGPSpositionsReceived = new List<GPSposition>();
            List<JulianDate> PositionTimes = new List<JulianDate>();
            List<Attitude> NewAttitudeReceived = new List<Attitude>();
            List<JulianDate> AttitudeTimes = new List<JulianDate>();
            List<GLoads> NewGLoadsReceived = new List<GLoads>();
            List<AngularRates> NewARatesReceived = new List<AngularRates>();

            // URL that points to the model
            //string modelURL = "3Dmodels/Cesium_Air.glb";
            string modelURL = "3Dmodels/maxus_mockup.glb";
            //string modelURL = "https://flight-data-visualization.herokuapp.com//Apps/SampleData/models/CesiumGround/Cesium_Ground.glb";
            //string modelURL = "3Dmodels/rocket.glb";

            string expectedAttitudeFormat = "quaternion"; // Maybe add this to the user interface?

            // Check which properties has been updated
            try
            {

                if (GPSpositionsReceived.Count > maxGPSIndex + 1)
                {
                    GPSUpdated = true;
                    GPSStartIndex = maxGPSIndex;
                    GPSEndIndex = GPSpositionsReceived.Count - 1;
                    maxGPSIndex = GPSEndIndex;
                    NewGPSpositionsReceived = GPSpositionsReceived.GetRange(GPSStartIndex, GPSEndIndex - GPSStartIndex + 1);
                    OldGPSpositionsReceived = GPSpositionsReceived.GetRange(0, GPSStartIndex);
                }

                if (AttitudeReceived.Count > maxAttitudeIndex + 1)
                {
                    attitudeUpdated = true;
                    AttitudeStartIndex = maxAttitudeIndex;
                    AttitudeEndIndex = AttitudeReceived.Count - 1;
                    maxAttitudeIndex = AttitudeEndIndex;
                    NewAttitudeReceived = AttitudeReceived.GetRange(AttitudeStartIndex, AttitudeEndIndex - AttitudeStartIndex + 1);
                    OldAttitudeReceived = AttitudeReceived.GetRange(0, AttitudeStartIndex);
                }

                if (GLoadsReceived.Count > maxGLoadIndex + 1)
                {
                    gLoadsUpdated = true;
                    GLoadStartIndex = maxGLoadIndex;
                    GLoadEndIndex = GLoadsReceived.Count - 1;
                    maxGLoadIndex = GLoadEndIndex;
                    NewGLoadsReceived = GLoadsReceived.GetRange(GLoadStartIndex, GLoadEndIndex - GLoadStartIndex + 1);
                    OldGLoadsReceived = GLoadsReceived.GetRange(0, GLoadStartIndex);
                }

                if (ARatesReceived.Count > maxARateIndex + 1)
                {
                    aRatesUpdated = true;
                    ARateStartIndex = maxARateIndex;
                    ARateEndIndex = ARatesReceived.Count - 1;
                    maxARateIndex = ARateEndIndex;
                    NewARatesReceived = ARatesReceived.GetRange(ARateStartIndex, ARateEndIndex - ARateStartIndex + 1);
                    OldARatesReceived = ARatesReceived.GetRange(0, ARateStartIndex);
                }

                if (GPS_IIP_Received.Count > maxGPS_IIP_Index + 1)
                {
                    IIPUpdated = true;
                    maxGPS_IIP_Index = GPS_IIP_Received.Count - 1;
                }

                if (RTtimestr != oldRTtimestr)
                {
                    timeUpdated = true;
                    oldRTtimestr = RTtimestr;
                }
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to check update properties: " + except.StackTrace.ToString());
            }

            System.Diagnostics.Stopwatch watch_checkTime = new System.Diagnostics.Stopwatch();

            try
            {
                // Checking if the mission time goes from "minus" to "plus", and saving the time of launch as a julian date
                missionSeconds = timeString2seconds(RTtimestr);
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to convert the time to seconds: " + except.StackTrace.ToString());
            }

            watch_checkTime.Start();
            if (!launched)
            {
                try
                {
                    // Checking if the mission time goes from "minus" to "plus", and saving the time of launch as a julian date
                    missionSeconds = timeString2seconds(RTtimestr);

                    if (plusminusTime == "-" && RTtimestr.Substring(0, 1) == "+")
                    {
                        plusminusTime = "+";

                        launchTime = new JulianDate(DateTime.Now).AddSeconds(-missionSeconds);

                        launched = true;
                        add2listBox("Launched!");
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to check the time: " + except.StackTrace.ToString());
                }
            }

            watch_checkTime.Stop();
            //add2listBox("------------------------------------Time to execute the time check: " + watch_checkTime.ElapsedMilliseconds);

            // Lists in which to store the data points that have been selected for visualizing
            List<JulianDate> dates = new List<JulianDate>();
            List<Cartographic> positions = new List<Cartographic>();
            List<UnitQuaternion> orientations = new List<UnitQuaternion>();
            double speed = 0;


            // Write the start sequence
            try
            {
                // Initializing a new json array
                output.WriteStartSequence();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write start sequence: " + except.StackTrace.ToString());
            }

            // Open a new packet for the rocket model
            try
            {
                packet = writer.OpenPacket(output);
                packet.WriteId("rocket");

                // Name the packet in a way so that the client knows if the data point is new or not
                if (GPSUpdated)
                {
                    packet.WriteName("GPS_updated");
                }
                else
                {
                    packet.WriteName("!GPS_updated");
                }
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to open the rocket packet: " + except.StackTrace.ToString());
            }

            // Measure performance
            System.Diagnostics.Stopwatch watch_GPS = new System.Diagnostics.Stopwatch();
            watch_GPS.Start();

            if (GPSUpdated)
            {
                try
                {
                    if (launched)
                    {
                        // Attach a path property to the rocket, enabling us to see the flight trail
                        path = packet.OpenPathProperty();
                        var pathMaterial = path.OpenMaterialProperty();
                        var polyLineMat = pathMaterial.OpenSolidColorProperty();
                        polyLineMat.WriteColorProperty(Color.Red);
                        polyLineMat.Close();
                        pathMaterial.Close();
                        path.WriteWidthProperty(2);
                        path.WriteLeadTimeProperty(0);
                        path.WriteTrailTimeProperty(3600);
                        path.WriteResolutionProperty(0.5);
                        path.Close();
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to rocket trail packet: " + except.StackTrace.ToString());
                }
                if (launched)
                {
                    try
                    {
                        for (int i = 0; i < NewGPSpositionsReceived.Count; i++)
                        {
                            GPSposition tempPosition = NewGPSpositionsReceived[i];
                            positions.Add(new Cartographic(tempPosition.Longitude, tempPosition.Latitude, tempPosition.Altitude));
                            PositionTimes.Add(tempPosition.RealTime);
                        }

                        //// MEAN FILTER, THE USE OF THIS WILL CAUSE TIME DE-SYNC
                        //// Window used for the mean filter
                        //int maxMeanWindow = 5;
                        //int realMeanWindow = OldGPSpositionsReceived.Count > maxMeanWindow ? maxMeanWindow : OldGPSpositionsReceived.Count;

                        //for (int iPos = 0; iPos < NewGPSpositionsReceived.Count; iPos++)
                        //{
                        //    GPSposition tempPosition = NewGPSpositionsReceived[iPos];
                        //    double tempLong = 0;
                        //    double tempLat = 0;
                        //    double tempAlt = 0;
                        //    for (int i = 0; i < realMeanWindow; i++)
                        //    {
                        //        if (i > iPos)
                        //        {
                        //            tempLong += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Longitude;
                        //            tempLat += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Latitude;
                        //            tempAlt += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Altitude;
                        //        }
                        //        else
                        //        {
                        //            tempLong += NewGPSpositionsReceived[iPos - i].Longitude;
                        //            tempLat += NewGPSpositionsReceived[iPos - i].Latitude;
                        //            tempAlt += NewGPSpositionsReceived[iPos - i].Altitude;
                        //        }

                        //    }
                        //    if (realMeanWindow > 0 && tempLong >= 0 && tempLat >= 0 && tempAlt >= 0)
                        //    {
                        //        Cartographic tempCart = new Cartographic(tempLong / realMeanWindow, tempLat / realMeanWindow, tempAlt / realMeanWindow);
                        //        positions.Add(tempCart);
                        //        oldCartographic = tempCart;
                        //        oldGPSDataExists = true;
                        //        //PositionTimes.Add(tempPosition.SimulatedTime);
                        //        PositionTimes.Add(tempPosition.RealTime);
                        //    }
                        //    else if (tempPosition.Longitude >= 0 && tempPosition.Latitude >= 0 && tempPosition.Altitude >= 0)
                        //    {
                        //        Cartographic tempCart = new Cartographic(tempPosition.Longitude, tempPosition.Latitude, tempPosition.Altitude);
                        //        positions.Add(tempCart);
                        //        oldCartographic = tempCart;
                        //        oldGPSDataExists = true;
                        //        //PositionTimes.Add(tempPosition.SimulatedTime);
                        //        PositionTimes.Add(tempPosition.RealTime);
                        //        //oldPositionTime = tempPosition.RealTime;
                        //    }
                        //}

                    }
                    catch (Exception except)
                    {
                        MessageBox.Show("Failed to add the positions: " + except.StackTrace.ToString());
                    }
                }
                else
                {
                    // If it hasn't launched, give it a fixed position
                    Cartographic tempCart = new Cartographic(initialLongitude, initialLatitude, 12);
                    positions.Add(tempCart);
                    oldCartographic = tempCart;
                    oldGPSDataExists = true;
                    PositionTimes.Add(NewGPSpositionsReceived[0].RealTime);
                }

                // Speed calculation
                if (NewGPSpositionsReceived.Count > 1)
                {
                    try
                    {
                        if (launched)
                        {
                            Vector<double> firstSamplePos = GPS2cartesian(NewGPSpositionsReceived[0].Longitude, NewGPSpositionsReceived[0].Latitude, NewGPSpositionsReceived[0].Altitude);
                            Vector<double> lastSamplePos = GPS2cartesian(NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Longitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Latitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Altitude);
                            speed = positions2speed(NewGPSpositionsReceived);

                            // Ensure that the speed keeps its last value if the calculation fails or returns a very low value during flight
                            if (speed < 0.01)
                            {
                                speed = oldSpeed;
                            }
                            else
                            {
                                oldSpeed = speed;
                            }

                            speedUpdated = true;
                        }
                    }
                    catch (Exception except)
                    {
                        MessageBox.Show("Failed to calculate the speed: " + except.StackTrace.ToString());
                    }
                }
                else
                {
                    if (GPSpositionsReceived.Count > 0)
                    {
                        try
                        {
                            if (launched)
                            {
                                Vector<double> secondPos = GPS2cartesian(NewGPSpositionsReceived[0].Longitude, NewGPSpositionsReceived[0].Latitude, NewGPSpositionsReceived[0].Altitude);
                                Vector<double> firstSamplePos = GPS2cartesian(GPSpositionsReceived[GPSpositionsReceived.Count - 1].Longitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Latitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Altitude);
                                speed = positions2speed(new List<GPSposition>() { GPSpositionsReceived[GPSpositionsReceived.Count - 1], NewGPSpositionsReceived[0] });

                                // Ensure that the speed keeps its last value if the calculation fails or returns a very low value during flight
                                if (speed < 0.01)
                                {
                                    speed = oldSpeed;
                                }
                                else
                                {
                                    oldSpeed = speed;
                                }

                                speedUpdated = true;
                            }
                        }
                        catch (Exception except)
                        {
                            MessageBox.Show("Failed to calculate the speed: " + except.StackTrace.ToString());
                        }
                    }
                    else
                    {
                        speedUpdated = false;
                    }
                }
            }
            /* THIS BLOCK OF CODE IS NOW HANDLED BY CESIUMS 'HOLD' EXTRAPOLATION*/
            //else
            //{
            //    // If no new GPS data is received but we have some old data, assume that data stream has initiated, but that it might have been interupted.
            //    if (oldGPSDataExists)
            //    {
            //        try
            //        {
            //            // Re-send the last known position if data is missing
            //            positions.Add(oldCartographic);
            //            ////PositionTimes.Add(new JulianDate(DateTime.Now));
            //            //oldPositionTime = oldPositionTime.AddSeconds(0.5);
            //            PositionTimes.Add(baseTimeTimeStamp.AddSeconds(baseTimeNow.SecondsDifference(new JulianDate(DateTime.Now))));
            //            //add2listBox("Pushing old GPS data");
            //        }
            //        catch (Exception except)
            //        {
            //            MessageBox.Show("Failed to open and write the rocket position packet: " + except.StackTrace.ToString());
            //            MessageBox.Show(OldGPSpositionsReceived.Count.ToString());
            //        }
            //    }
            //}
            watch_GPS.Stop();

            // If positions are waiting to be sent to the server
            if (positions.Count > 0)
            {
                try
                {
                    // Write positions
                    position = packet.OpenPositionProperty();

                    position.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Linear);
                    position.WriteInterpolationDegree(1);
                    position.WriteForwardExtrapolationDuration(extrapolationDuration);
                    position.WriteBackwardExtrapolationDuration(extrapolationDuration);
                    position.WriteForwardExtrapolationType(CesiumExtrapolationType.Hold);

                    position.WriteCartographicDegrees(PositionTimes, positions);

                    position.Close();
                }
                catch (Exception except)
                {

                    MessageBox.Show("Failed to write positions: " + except.StackTrace.ToString());
                }
            }

            System.Diagnostics.Stopwatch watch_attitude = new System.Diagnostics.Stopwatch();

            // Measure performance
            watch_attitude.Start();

            if (attitudeUpdated)
            {

                int medianFrom = 3; // Median window size 
                try
                {
                    for (int i = 0; i < NewAttitudeReceived.Count; i++)
                    {
                        if (expectedAttitudeFormat == "quaternion")
                        {
                            Attitude tempAttitude = NewAttitudeReceived[i];

                            if (tempAttitude != null)
                            {
                                // If the number of previous data points is greater than the median window, enable the filter
                                bool medianFilterPossible = OldAttitudeReceived.Count > medianFrom;

                                // NOTE: This filter is hard coded for a window size of 3
                                if (medianFilterPossible)
                                {
                                    Attitude a1 = tempAttitude;
                                    Attitude a2;
                                    Attitude a3;

                                    // Choose the array to take the values from
                                    if (i == 0)
                                    {
                                        a2 = OldAttitudeReceived[OldAttitudeReceived.Count - 1];
                                        a3 = OldAttitudeReceived[OldAttitudeReceived.Count - 2];
                                    }
                                    else if (i == 1)
                                    {
                                        a2 = NewAttitudeReceived[i - 1];
                                        a3 = OldAttitudeReceived[OldAttitudeReceived.Count - 1];
                                    }
                                    else
                                    {
                                        a2 = NewAttitudeReceived[i - 1];
                                        a3 = NewAttitudeReceived[i - 2];
                                    }

                                    Attitude[] aArray = new Attitude[3] { a1, a2, a3 };
                                    int[] indices = new int[3] { 0, 1, 2 };

                                    double[] medianArray = new double[3];
                                    medianArray[0] = Math.Pow(a1.q0, 2) + Math.Pow(a1.q1, 2) + Math.Pow(a1.q2, 2) + Math.Pow(a1.q3, 2);
                                    medianArray[1] = Math.Pow(a2.q0, 2) + Math.Pow(a2.q1, 2) + Math.Pow(a2.q2, 2) + Math.Pow(a2.q3, 2);
                                    medianArray[2] = Math.Pow(a3.q0, 2) + Math.Pow(a3.q1, 2) + Math.Pow(a3.q2, 2) + Math.Pow(a3.q3, 2);

                                    Array.Sort(medianArray, indices);

                                    UnitQuaternion tempQuat;

                                    // Ensure that the quaternion chosen is approximately of unit size
                                    if (Math.Abs(medianArray[indices[1]] - 1) < 0.01)
                                    {
                                        Attitude medianAttitude = aArray[indices[1]]; //Selecting the attitude corresponding to the median
                                        tempQuat = TH2ECEF(medianAttitude.q0, medianAttitude.q1, medianAttitude.q2, medianAttitude.q3);
                                    }
                                    else
                                    {
                                        tempQuat = oldRealQuaternion;
                                    }

                                    // Extra check, look for NaN values
                                    if (Math.Abs(tempQuat.W) <= 1 && Math.Abs(tempQuat.X) <= 1 && Math.Abs(tempQuat.Y) <= 1 && Math.Abs(tempQuat.Z) <= 1)
                                    {
                                        orientations.Add(tempQuat);
                                        AttitudeTimes.Add(tempAttitude.Time);
                                    }
                                    oldRealQuaternion = tempQuat;
                                    oldRealAttitudeDataExists = true;
                                }
                                // If a median filter isn't possible, just take the data point if is valid
                                else
                                {
                                    UnitQuaternion tempQuat = TH2ECEF(tempAttitude.q0, tempAttitude.q1, tempAttitude.q2, tempAttitude.q3);
                                    if (Math.Abs(tempQuat.W) <= 1 && Math.Abs(tempQuat.X) <= 1 && Math.Abs(tempQuat.Y) <= 1 && Math.Abs(tempQuat.Z) <= 1)
                                    {
                                        orientations.Add(tempQuat);
                                        AttitudeTimes.Add(tempAttitude.Time);
                                    }
                                    oldRealQuaternion = tempQuat;
                                    oldRealAttitudeDataExists = true;
                                }
                            }

                        }
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to add quaternion: " + except.StackTrace.ToString());
                }

                //add2listBox("Pushing new attitude data");
            }
            //If no new attitude data exists push the last known one
            else if (oldRealAttitudeDataExists)
            {
                orientations.Add(oldRealQuaternion);
                AttitudeTimes.Add(baseTimeTimeStamp.AddSeconds(baseTimeNow.SecondsDifference(new JulianDate(DateTime.Now))));
            }
            else if (GPSUpdated && GPSStartIndex > 0)
            {
                // Otherwise, assume that attitude data is NOT available, and make sure that the attitude points in the direction of flight.
                try
                {
                    if (launched)
                    {
                        List<UnitQuaternion> tempAttitude = attitudeFromGPS(NewGPSpositionsReceived, OldGPSpositionsReceived, GPSStartIndex);
                        if (tempAttitude.Count > 0)
                        {
                            oldCalculatedQuaternion = tempAttitude[tempAttitude.Count - 1];
                            oldCalculatedAttitudeDataExists = true;
                        }
                        orientations.AddRange(tempAttitude);
                        AttitudeTimes = PositionTimes;
                    }
                    //if it hasn't launched and no attitude data is available, just push (1,0,0,0)
                    else
                    {
                        UnitQuaternion tempQuat = TH2ECEF(1, 0, 0, 0);
                        // Check the quaternion before pushing, in case something funky happened in the coordinate conversion
                        if (Math.Abs(tempQuat.W) <= 1 && Math.Abs(tempQuat.X) <= 1 && Math.Abs(tempQuat.Y) <= 1 && Math.Abs(tempQuat.Z) <= 1)
                        {
                            orientations.Add(tempQuat);
                            AttitudeTimes.Add(baseTimeTimeStamp.AddSeconds(baseTimeNow.SecondsDifference(new JulianDate(DateTime.Now))));
                        }
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to calculate attitude from GPS data: " + except.StackTrace.ToString());
                }
            }
            // If previous attitude data has been calculated, but no new, push the latest known data point
            else if (oldCalculatedAttitudeDataExists)
            {
                //add2listBox("Pushing old attitude data");
                orientations.Add(oldCalculatedQuaternion);
                AttitudeTimes.Add(baseTimeTimeStamp.AddSeconds(baseTimeNow.SecondsDifference(new JulianDate(DateTime.Now))));
            }

            if (orientations.Count > 0)
            {
                try
                {
                    // Open the orientation packet
                    orientation = packet.OpenOrientationProperty();

                    orientation.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Linear);
                    orientation.WriteInterpolationDegree(1);
                    orientation.WriteForwardExtrapolationDuration(extrapolationDuration);
                    orientation.WriteForwardExtrapolationType(CesiumExtrapolationType.Hold);
                    orientation.WriteUnitQuaternion(AttitudeTimes, orientations);

                    orientation.Close();
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to write the orientation packet: " + except.StackTrace.ToString());
                }
            }

            watch_attitude.Stop();
            //add2listBox("------------------------------------Time to execute attitude write: " + watch_attitude.ElapsedMilliseconds);

            // Write the model packet
            try
            {
                // Opening the model packet
                model = packet.OpenModelProperty();
                model.WriteGltfProperty(modelURL, CesiumResourceBehavior.LinkTo);

                // Correcting the scale
                model.WriteScaleProperty(0.001);
                model.WriteMaximumScaleProperty(1);
                model.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write model packet: " + except.StackTrace.ToString());
            }

            // Close the rocket packet
            try
            {
                // Close packet when everything is appended
                packet.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to close rocket packet: " + except.StackTrace.ToString());
            }

            try
            {
                // Open the packet used for the speed, g-forces, and events.
                packet = writer.OpenPacket(output);
                packet.WriteId("speed_gLoads");
                string eventString = eventStringFromList(missionSeconds);
                packet.WriteName(eventString);
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to open the packet for speed and G-loads: " + except.StackTrace.ToString());
            }

            // Opening the point packet (this is an abstract point, it will not be visible)
            try
            {
                point = packet.OpenPointProperty();
                point.WriteShowProperty(false);
                if (gLoadsUpdated || speedUpdated)
                {
                    // pixelSize will correspond to the speed

                    // Only update the speed if it has a valid update from the positions
                    if (speedUpdated)
                    {
                        point.WritePixelSizeProperty(speed);
                    }

                    // The position of the point will correspont to the 3 component g-force
                    if (gLoadsUpdated)
                    {
                        gLoadPacket = packet.OpenPositionProperty();

                        Cartesian gLoads;
                        //aRates = NewARatesReceived[NewARatesReceived.Count - 1].toCartesian();

                        int medianFrom = 3;

                        int newGLoads = NewGLoadsReceived.Count;
                        GLoads tempGLoads = NewGLoadsReceived[newGLoads - 1];

                        gLoads = tempGLoads.toCartesian();

                        gLoadPacket.WriteCartesian(gLoads);
                        gLoadPacket.Close();
                    }
                }
                // Closing the point packet
                point.Close();
                // Close packet when everything is appended
                packet.Close();
            }
            catch (Exception except) {
                MessageBox.Show("Failed to write the packet for speed and G-loads: " + except.StackTrace.ToString());
            }

            try
            {
                // Open the packet used for the angular rates and time.
                packet = writer.OpenPacket(output);
                packet.WriteId("angularRates_time");
                packet.WriteName(RTtimestr);

                // Opening the point packet (this is an abstract point, it will not be visible)
                point = packet.OpenPointProperty();
                point.WriteShowProperty(false);
                
                // If the GPS is updated, write the time in seconds, which will be used for plotting the height curve
                if (GPSUpdated)
                {
                    if (launched)
                    {
                        point.WritePixelSizeProperty(launchTime.SecondsDifference(new JulianDate(DateTime.Now)));
                    }
                    else
                    {
                        point.WritePixelSizeProperty(-1);
                    }
                }
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write the time in seconds: " + except.StackTrace.ToString());
            }

            // Write angular rates if they are available
            if (aRatesUpdated)
            {
                try
                {
                    aRatePacket = packet.OpenPositionProperty();
                    Cartesian aRates;

                    // Median window
                    int medianFrom = 3;

                    int newARates = NewARatesReceived.Count;
                    AngularRates tempARate = NewARatesReceived[newARates - 1];

                    double aRateX = tempARate.aRateX;
                    double aRateY = tempARate.aRateY;
                    double aRateZ = tempARate.aRateZ;

                    bool medianFilterPossible = OldARatesReceived.Count > medianFrom;

                    if (medianFilterPossible)
                    {
                        double Gx1 = aRateX;
                        double Gx2;
                        double Gx3;

                        double Gy1 = aRateY;
                        double Gy2;
                        double Gy3;

                        double Gz1 = aRateZ;
                        double Gz2;
                        double Gz3;

                        if (newARates == 1)
                        {
                            Gx2 = OldARatesReceived[OldARatesReceived.Count - 1].aRateX;
                            Gx3 = OldARatesReceived[OldARatesReceived.Count - 2].aRateX;

                            Gy2 = OldARatesReceived[OldARatesReceived.Count - 1].aRateY;
                            Gy3 = OldARatesReceived[OldARatesReceived.Count - 2].aRateY;

                            Gz2 = OldARatesReceived[OldARatesReceived.Count - 1].aRateZ;
                            Gz3 = OldARatesReceived[OldARatesReceived.Count - 2].aRateZ;
                        }
                        else if (newARates == 2)
                        {
                            Gx2 = NewARatesReceived[newARates - 2].aRateX;
                            Gx3 = OldARatesReceived[OldARatesReceived.Count - 1].aRateX;

                            Gy2 = NewARatesReceived[newARates - 2].aRateY;
                            Gy3 = OldARatesReceived[OldARatesReceived.Count - 1].aRateY;

                            Gz2 = NewARatesReceived[newARates - 2].aRateZ;
                            Gz3 = OldARatesReceived[OldARatesReceived.Count - 1].aRateZ;
                        }
                        else
                        {
                            Gx2 = NewARatesReceived[newARates - 2].aRateX;
                            Gx3 = NewARatesReceived[newARates - 3].aRateX;

                            Gy2 = NewARatesReceived[newARates - 2].aRateY;
                            Gy3 = NewARatesReceived[newARates - 3].aRateY;

                            Gz2 = NewARatesReceived[newARates - 2].aRateZ;
                            Gz3 = NewARatesReceived[newARates - 3].aRateZ;
                        }

                        double[] GxArray = new double[3] { Gx1, Gx2, Gx3 };
                        double[] GyArray = new double[3] { Gy1, Gy2, Gy3 };
                        double[] GzArray = new double[3] { Gz1, Gz2, Gz3 };
                        int[] Gxindices = new int[3] { 0, 1, 2 };
                        int[] Gyindices = new int[3] { 0, 1, 2 };
                        int[] Gzindices = new int[3] { 0, 1, 2 };

                        double[] medianArray = new double[3];

                        Array.Sort(GxArray, Gxindices);
                        Array.Sort(GyArray, Gyindices);
                        Array.Sort(GzArray, Gzindices);

                        aRates = new Cartesian(GxArray[1], GyArray[1], GzArray[1]);
                    }
                    else
                    {
                        aRates = tempARate.toCartesian();
                    }

                    aRatePacket.WriteCartesian(aRates);
                    aRatePacket.Close();
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to write the aRate packet: " + except.StackTrace.ToString());
                }
            }

            try
            {
                // Closing the packets
                point.Close();
                packet.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to close the packet for angular rates and time: " + except.StackTrace.ToString());
            }

            try
            {
                // Open the packet used for the IIP.
                packet = writer.OpenPacket(output);
                packet.WriteId("IIP_coordinates");

                // Opening a point packet, which will be used to show the IIP
                point = packet.OpenPointProperty();
                point.WriteShowProperty(false);

                if (IIPUpdated)
                {
                    IIPPacket = packet.OpenPositionProperty();
                    Cartographic IIP = GPS_IIP_Received[GPS_IIP_Received.Count - 1].toCartographic();
                    IIPPacket.WriteCartographicDegrees(IIP);
                    IIPPacket.Close();
                }

                // Closing the point packet
                point.Close();
                // Close packet when everything is appended
                packet.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to open the packet for IIP: " + except.StackTrace.ToString());
            }

            // Measuring performance
            System.Diagnostics.Stopwatch watch_flightPath = new System.Diagnostics.Stopwatch();
            watch_flightPath.Start();

            // Write the packet used to store positions without having to sort out the time stamps
            try
            {
                // Open the packet used for the position backlog.
                packet = writer.OpenPacket(output);
                polyLine = packet.OpenPolylineProperty();
                var material = polyLine.OpenMaterialProperty();
                var pathColor = material.OpenSolidColorProperty();
                pathColor.WriteColorProperty(Color.Red);
                pathColor.Close();
                material.Close();
                polyLine.WriteFollowSurfaceProperty(false);
                polyLine.WriteWidthProperty(2);
                if (GPSUpdated)
                {
                    polyLine.WritePositionsPropertyCartographicDegrees(positions);
                }

                // Close packets
                polyLine.Close();
                packet.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write flight path polyline packet: " + except.StackTrace.ToString());
            }

            watch_flightPath.Stop();
            //add2listBox("------------------------------------Time to write flight path: " + watch_flightPath.ElapsedMilliseconds);

            try
            {
                // End sequence to close the json array
                output.WriteEndSequence();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write end sequence on JSON array: " + except.StackTrace.ToString());
            }
            
            try
            {
                // Create POST data and convert it to a byte array.
                byte[] byteArray = Encoding.UTF8.GetBytes(sw.ToString());

                // Close the string writer
                sw.Close();

                // Initiate a new thread to take care of the data pushing
                new System.Threading.Thread(delegate ()
                {
                    var responseStatus = postData(byteArray);
                }).Start();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to push data: " + except.StackTrace.ToString());
            }

            watch_tot.Stop();
            //add2listBox("------------------------------------Total time to push data: " + watch_tot.ElapsedMilliseconds);
        }

        // Extract the correct event when the time of that event is reached
        private string eventStringFromList(double missionTimeInSeconds)
        {
            int maxIndex = 0;
            for (int eventIndex = 0; eventIndex < eventTimeList.Length; eventIndex++)
            {
                if (eventTimeList[eventIndex] <= missionTimeInSeconds)
                {
                    maxIndex = eventIndex;
                }
            }

            return eventStringList[maxIndex];
        }

        /* Converting a time string of the format "+-hh;mm;ss" to seconds. */
        private double timeString2seconds(string missionTime)
        {
            string plusminus = missionTime.Substring(0, 1);
            string hourString = missionTime.Substring(1, 2);
            string minuteString = missionTime.Substring(4, 2);
            string secondString = missionTime.Substring(7, 2);

            double seconds = Convert.ToDouble(hourString) * 3600 + Convert.ToDouble(minuteString) * 60 + Convert.ToDouble(secondString);

            if (plusminus == "+")
            {
                return seconds;
            }
            else
            {
                return -seconds;
            }
        }

        /* Transforming the Topocentric Horizon reference system to the Earth Centered, Earth Fixed system. */
        private UnitQuaternion TH2ECEF(double q0, double q1, double q2, double q3)
        {
            // Defining the different matrices
            Matrix<double> qDMC = Matrix<double>.Build.Dense(3, 3);
            Matrix<double> ECEF2TH = Matrix<double>.Build.Dense(3, 3);
            Matrix<double> ECEF2body = Matrix<double>.Build.Dense(3, 3);

            // Constructing the DCM from the quaternions
            qDMC[0, 0] = Math.Pow(q0, 2) + Math.Pow(q1, 2) - Math.Pow(q2, 2) - Math.Pow(q3, 2);
            qDMC[0, 1] = 2 * (q1 * q2 + q0 * q3);
            qDMC[0, 2] = 2 * (q1 * q3 - q0 * q2);
            qDMC[1, 0] = 2 * (q1 * q2 - q0 * q3);
            qDMC[1, 1] = Math.Pow(q0, 2) - Math.Pow(q1, 2) + Math.Pow(q2, 2) - Math.Pow(q3, 2);
            qDMC[1, 2] = 2 * (q2 * q3 + q0 * q1);
            qDMC[2, 0] = 2 * (q1 * q3 + q0 * q2);
            qDMC[2, 1] = 2 * (q2 * q3 - q0 * q1);
            qDMC[2, 2] = Math.Pow(q0, 2) - Math.Pow(q1, 2) - Math.Pow(q2, 2) + Math.Pow(q3, 2);

            // Defining cos and sin of the longitude and latitude
            double cLong = Math.Cos(deg2rad * initialLongitude);
            double sLong = Math.Sin(deg2rad * initialLongitude);

            double cLat = Math.Cos(deg2rad * initialLatitude);
            double sLat = Math.Sin(deg2rad * initialLatitude);

            //// Constructing the DCM from ECEF to TH
            ECEF2TH[0, 0] = cLong * cLat;
            ECEF2TH[0, 1] = sLong * cLat;
            ECEF2TH[0, 2] = sLat;
            ECEF2TH[1, 0] = -sLong;
            ECEF2TH[1, 1] = cLong;
            ECEF2TH[1, 2] = 0;
            ECEF2TH[2, 0] = -cLong * sLat;
            ECEF2TH[2, 1] = -sLong * sLat;
            ECEF2TH[2, 2] = cLat;

            // Multiplying TH2Body with ECEF2body to obtain a rotation matrix 
            // that describes the orientation of the body axes in the ECEF system
            qDMC.Multiply(ECEF2TH, ECEF2body);

            bool error = !(ECEF2body[0, 0] < 1 && ECEF2body[0, 1] < 1 && ECEF2body[0, 2] < 1 && ECEF2body[1, 0] < 1 && ECEF2body[1, 1] < 1 && ECEF2body[1, 2] < 1 && ECEF2body[2, 0] < 1 && ECEF2body[2, 1] < 1 && ECEF2body[2, 2] < 1);

            if (!error)
            {
                Vector<double> quaternionVector = markley(ECEF2body);
                return new UnitQuaternion(quaternionVector[0], quaternionVector[1], quaternionVector[2], quaternionVector[3]);
            }
            else if (oldRealAttitudeDataExists)
            {
                return oldRealQuaternion;
            }
            else
            {
                Vector<double> quaternionVector = markley(ECEF2body);
                return new UnitQuaternion(quaternionVector[0], quaternionVector[1], quaternionVector[2], quaternionVector[3]);
            }


        }

        /* The Markley method converts a DCM to quaternions while maintaining orthonormality. */
        private Vector<double> markley(Matrix<double> ECEF2body)
        {
            // Defining the trace of the rotation matrix
            double trace = ECEF2body.Trace();

            List<Vector<double>> vectorList = new List<Vector<double>>();

            Vector<double> x0 = Vector<double>.Build.Dense(4);
            x0[0] = 1 + trace;
            x0[1] = ECEF2body[1, 2] - ECEF2body[2, 1];
            x0[2] = ECEF2body[2, 0] - ECEF2body[0, 2];
            x0[3] = ECEF2body[0, 1] - ECEF2body[1, 0];
            vectorList.Add(x0);

            Vector<double> x1 = Vector<double>.Build.Dense(4);
            x1[0] = ECEF2body[1, 2] - ECEF2body[2, 1];
            x1[1] = 1 + 2 * ECEF2body[0, 0] - trace;
            x1[2] = ECEF2body[0, 1] + ECEF2body[1, 0];
            x1[3] = ECEF2body[0, 2] + ECEF2body[2, 0];
            vectorList.Add(x1);

            Vector<double> x2 = Vector<double>.Build.Dense(4);
            x2[0] = ECEF2body[2, 0] - ECEF2body[0, 2];
            x2[1] = ECEF2body[0, 1] + ECEF2body[1, 0];
            x2[2] = 1 + 2 * ECEF2body[1, 1] - trace;
            x2[3] = ECEF2body[1, 2] + ECEF2body[2, 1];
            vectorList.Add(x2);

            Vector<double> x3 = Vector<double>.Build.Dense(4);
            x3[0] = ECEF2body[0, 1] - ECEF2body[1, 0];
            x3[1] = ECEF2body[2, 0] + ECEF2body[0, 2];
            x3[2] = ECEF2body[1, 2] + ECEF2body[2, 1];
            x3[3] = 1 + 2 * ECEF2body[2, 2] - trace;
            vectorList.Add(x3);

            int maxIndex = 0;
            double maxMagnitude = 0;

            for (int i = 0; i < vectorList.Count; i++)
            {
                if (Math.Pow(vectorList[i].L2Norm(), 2) > maxMagnitude)
                {
                    maxMagnitude = Math.Pow(vectorList[i].L2Norm(), 2);
                    maxIndex = i;
                }
            }
            return vectorList[maxIndex].Normalize(2);
        }

        /* Converting GPS coordinates to cartesian in ECEF */
        private static Vector<double> GPS2cartesian(double longitude, double latitude, double height)
        {
            // Empty vector to store the results in
            Vector<double> cartesianPos = Vector<double>.Build.Dense(3);

            try
            {
                // Radius of the curvature in the prime vertical
                double N = semiMajorAxis / (1 - Math.Pow(eccentricity * Math.Sin(latitude * deg2rad), 2));

                // X
                cartesianPos[0] = (N + height) * Math.Cos(latitude * deg2rad) * Math.Cos(longitude * deg2rad);
                // Y
                cartesianPos[1] = (N + height) * Math.Cos(latitude * deg2rad) * Math.Sin(longitude * deg2rad);
                // Z
                cartesianPos[2] = ((1 - Math.Pow(eccentricity, 2)) * N + height) * Math.Sin(latitude * deg2rad);
            }
            catch (Exception except)
            {
                MessageBox.Show(except.StackTrace.ToString());
            }

            return cartesianPos;
        }

        // Calculate the speed from the position
        private double positions2speed(List<GPSposition> GPSpos)
        {
            double speedResult = 0;
            Vector<double> velocity = Vector<double>.Build.DenseOfArray(new double[3] { 0, 0, 0 });
            try
            {
                for (int i = 0; i < GPSpos.Count - 1; i++)
                {
                    Vector<double> firstPos = GPS2cartesian(GPSpos[i].Longitude, GPSpos[i].Latitude, GPSpos[i].Altitude);
                    Vector<double> secondPos = GPS2cartesian(GPSpos[i + 1].Longitude, GPSpos[i + 1].Latitude, GPSpos[i + 1].Altitude);
                    Vector<double> posDiff = secondPos - firstPos;
                    
                    JulianDate firstTime = GPSpos[i].RealTime;
                    JulianDate secondTime = GPSpos[i + 1].RealTime;
                    double timeDiff = secondTime.SecondsDifference(firstTime);

                    if (Math.Abs(posDiff.L2Norm()) < 0.001)
                    {
                        return 0;
                    }

                    // Don't wanna divide by zero
                    if (Math.Abs(timeDiff) < 0.0000001)
                    {
                        return 0;
                    }
                    // Calculating the velocity
                    velocity = posDiff.Divide(timeDiff);
                    speedResult += velocity.L2Norm() / (GPSpos.Count - 1);
                }
            }
            catch (Exception except)
            {
                MessageBox.Show(except.StackTrace.ToString());
            }
            // Returning the speed, which is the L2 norm of the velocity
            return speedResult;
        }

        // Finds an orthogonal vector
        Vector<double> orthogonal(Vector<double> v)
        {
            double x;
            double y;
            double z;
            Vector<double> other;

            try
            {
                x = Math.Abs(v[0]);
                y = Math.Abs(v[1]);
                z = Math.Abs(v[2]);

                other = x < y ? (x < z ? xAxis : zAxis) : (y < z ? yAxis : zAxis);
                return crossProduct(v, other);
            }
            catch (Exception except)
            {
                MessageBox.Show(except.StackTrace.ToString());
                return crossProduct(v, v);
            }
        }

        // For some reason, MathDotNET has not implemented a cross product yet, so this is a temporary fix.
        Vector<double> crossProduct(Vector<double> v1, Vector<double> v2)
        {
            Vector<double> resultVector = Vector<double>.Build.Dense(3);
            resultVector[0] = v1[1] * v2[2] - v2[1] * v1[2];
            resultVector[1] = v1[2] * v2[0] - v2[2] * v1[0];
            resultVector[2] = v1[0] * v2[1] - v2[0] * v1[1];

            return resultVector;
        }

        /* Used to find the quaternion that describes the orientation in the velocity direction. */
        private List<UnitQuaternion> attitudeFromGPS(List<GPSposition> NewGPSposList, List<GPSposition> OldGPSposList, int startIndex)
        {
            Vector<double> currentPosition = Vector<double>.Build.Dense(3);
            Vector<double> previousPosition;

            List<UnitQuaternion> resultsList = new List<UnitQuaternion>();

            try
            {
                // Using data points further from the current point, which makes it less sensitive to sudden changes
                int maxBackTrack = OldGPSposList.Count - 1 > 10 ? 10 : OldGPSposList.Count - 1;

                Vector<double> xVector = Vector<double>.Build.Dense(3);
                xVector[0] = 1;
                xVector[1] = 0;
                xVector[2] = 0;

                Vector<double> quat = Vector<double>.Build.Dense(4);

                for (int i = 0; i < NewGPSposList.Count; i++)
                {
                    Vector<double> tempQuat = Vector<double>.Build.Dense(4);
                    // Choose which vector to take the previous value from
                    if (i <= maxBackTrack - 1)
                    {
                        previousPosition = GPS2cartesian(OldGPSposList[OldGPSposList.Count + i - maxBackTrack].Longitude, OldGPSposList[OldGPSposList.Count + i - maxBackTrack].Latitude, OldGPSposList[OldGPSposList.Count + i - maxBackTrack].Altitude);
                    }
                    else
                    {
                        previousPosition = GPS2cartesian(NewGPSposList[i - maxBackTrack].Longitude, NewGPSposList[i - maxBackTrack].Latitude, NewGPSposList[i - maxBackTrack].Altitude);
                    }

                    currentPosition = GPS2cartesian(NewGPSposList[i].Longitude, NewGPSposList[i].Latitude, NewGPSposList[i].Altitude);

                    // If the speed is almost zero, do not update the attitude
                    if ((currentPosition - previousPosition).L2Norm() < 0.001)
                    {
                        continue;
                    }

                    // Finding the direction of flight, and normalizing it with respect to the L2-norm
                    Vector<double> directionOfFlight = (currentPosition - previousPosition).Normalize(2);

                    // Special case for when the direction of flight is anti-paralell to the x-vector
                    if (directionOfFlight == -xVector)
                    {
                        // In this case, the rotation can be described by a 180 degrees rotation around any axis that is orthogonal to directionOfFlight
                        tempQuat[0] = 0;

                        Vector<double> subQuat = orthogonal(directionOfFlight);
                        tempQuat[1] = subQuat[0];
                        tempQuat[2] = subQuat[1];
                        tempQuat[3] = subQuat[2];
                    }
                    else
                    {
                        // Creating a unit vector that is located halfway inbetween the two vectors
                        Vector<double> halfVector = (directionOfFlight + xVector).Normalize(2);

                        tempQuat[0] = halfVector.DotProduct(directionOfFlight);
                        
                        Vector<double> subQuat = crossProduct(halfVector, directionOfFlight);
                        tempQuat[1] = subQuat[0];
                        tempQuat[2] = subQuat[1];
                        tempQuat[3] = subQuat[2];
                    }
                    //add2listBox("Adding quat");
                    resultsList.Add(new UnitQuaternion(tempQuat[0], tempQuat[1], tempQuat[2], tempQuat[3]));

                }
            }
            catch (Exception except)
            {
                MessageBox.Show(except.StackTrace.ToString());
            }
            return resultsList;
        }

        /* postData is handles the HTTP request. */
        public static async Task<string> postData(byte[] data)
        {
            try
            {
                HttpWebRequest request = (HttpWebRequest)WebRequest.Create(serverPath);
                request.Method = "POST";
                request.ContentType = "application/json";
                request.ContentLength = data.Length;

                using (var stream = request.GetRequestStream())
                {
                    stream.Write(data, 0, data.Length);
                }

                var response = (HttpWebResponse)request.GetResponse();
                var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();
                
                return "";
            }
            catch (Exception except)
            {
                MessageBox.Show("postData failed to send data to the server: " + except.StackTrace.ToString());
                MessageBox.Show(System.Text.Encoding.Default.GetString(data));
                return "";
            }
        }

        private void listBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (comboBox2.SelectedIndex == 0)
            {
                testTypeLabel.Hide();
                GPSTestCheckBox.Hide();
                attitudeTestCheckBox.Hide();
                gLoadTestCheckBox.Hide();
                aRateTestCheckBox.Hide();
                IIPTestCheckbox.Hide();
            }
            else if (comboBox2.SelectedIndex == 1)
            {
                testTypeLabel.Show();
                GPSTestCheckBox.Show();
                attitudeTestCheckBox.Show();
                gLoadTestCheckBox.Show();
                aRateTestCheckBox.Show();
                IIPTestCheckbox.Show();
            }
        }

        private void comboBox3_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void label5_Click(object sender, EventArgs e)
        {

        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {

        }
    }
}
