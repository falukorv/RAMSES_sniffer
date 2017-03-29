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
        //string RamsesNetworkcardip = "192.168.1.102";

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

        private List<GPSposition> GPSpositionsReceived = new List<GPSposition>();
        private List<GPS_IIP> GPS_IIP_Received = new List<GPS_IIP>();
        private List<Attitude> AttitudeReceived = new List<Attitude>();
        private List<GLoads> GLoadsReceived = new List<GLoads>();
        private List<AngularRates> ARatesReceived = new List<AngularRates>();

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
        private double eccentricity = 0.081819190842622;
        private double semiMajorAxis = 6378137;
        private Vector<double> xAxis = Vector<double>.Build.DenseOfArray(new double[3] { 1, 0, 0 });
        private Vector<double> yAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 1, 0 });
        private Vector<double> zAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 0, 1 });

        //private List<GPSposition> NewGPSpositionsReceived = new List<GPSposition>();
        private List<JulianDate> PositionTimes = new List<JulianDate>();
        //private List<Attitude> NewAttitudeReceived = new List<Attitude>();
        private List<JulianDate> AttitudeTimes = new List<JulianDate>();
        //private List<GLoads> NewGLoadsReceived = new List<GLoads>();
        //private List<AngularRates> NewARatesReceived = new List<AngularRates>();
        private Cartographic oldCartographic = new Cartographic(1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private UnitQuaternion oldRealQuaternion = new UnitQuaternion(1, 1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private UnitQuaternion oldCalculatedQuaternion = new UnitQuaternion(1, 1, 1, 1); // Used to store the latest attitude calculated from the GPS data.
        private bool oldGPSDataExists = false;
        private bool oldRealAttitudeDataExists = false;
        private bool oldCalculatedAttitudeDataExists = false;

        // Check if mission time goes from minus to plus
        private string plusminusTime = "-";

        // List of events
        private int[] eventTimeList = new int[] { -9999, -300, -240, -150, -70, -65, -60, -60, -57, -47, -40, -25, -20, -16, -14, -9, -8, 0, 62, 64, 86, 96, 160, 230, 800, 845 };
        private string[] eventStringList = new string[] { "Server synchronized with data pusher", "Rocket system in 'flight mode'", "TGV/INA on INTERNAL power", "360 Recording started", "XRMON-DIFF2 Internal power (module)", "XRMON-DIFF2 Internal power (furnace 1)", "XRMON-DIFF2 Internal power (furnace 2)", "Launch sequencer activation", "TVA command check", "TVA check (unpressurised)", "Charging pressurising pyro capacitors", "Verifying launcher status and arming TVA", "TVA Pressurising", "TVA pressure check", "TVA Deflection test", "TVA Baseline Retrace test", "Go signal", "Lift-off", "Expected motor burn-out", "XRMON-DIFF2 Image sequence start", "Expected motor separation", "Expected Super-Max separation", "XRMON-DIFF2 Alumina shearing", "XRMON-DIFF2 Graphite shearing", "XRMON-DIFF2 Experiment power down", "Castor 4B destruction" };

        // The frequency of which we will push data to the visualization server.
        private static double pushFrequency = 2;

        // Only sending data that are updated, and "static" data are sent in the first packets
        private bool firstPositionPacket = true;
        private bool firstOrientationPacket = true;

        //// The different czml-writers used
        //private static StringWriter sw = new StringWriter();
        //private CesiumOutputStream output = new CesiumOutputStream(sw);
        //private CesiumStreamWriter writer = new CesiumStreamWriter();
        //private PositionCesiumWriter position;
        //private PositionCesiumWriter gLoadPacket; // Will write the g-forces as the coordinates of an abstract point
        //private PositionCesiumWriter aRatePacket; // Will write the angular rates as the coordinates of an abstract point
        //private PositionCesiumWriter IIPPacket;
        //private OrientationCesiumWriter orientation;
        //private ModelCesiumWriter model;
        //private PointCesiumWriter point;
        //private PathCesiumWriter path;
        //private PolylineCesiumWriter polyLine;
        //private ClockCesiumWriter clock;
        //private PacketCesiumWriter packet;

        // Properties of interest
        //private double time;
        //private double speed;
        private double initialLatitude; // Latitude of the topocentric horizon system
        private double initialLongitude; // Longitude of the topocentric horizon system
        //private double altitude;
        private double yaw;
        private double pitch;
        private double roll;

        // Quaternion variables:
        private double w;
        private double x;
        private double y;
        private double z;

        // The string that points towards the model used
        private string modelURL;

        // The time expressed in julian date
        private JulianDate currentJuliandate; // The current time
        private JulianDate lastJulianDate; // Julian date of the last received message
        private DateTime currentDate = DateTime.Now;
        private JulianDate launchTime;
        private double missionSeconds; // The mission time in seconds
        private bool launched = false;
        private string oldRTtimestr = "";

        // The duration for how long we are extrapolating
        private Duration extrapolationDuration;

        // Constant used for converting degrees to radians
        private double deg2rad = Math.PI / 180;

        // An array in which we store the data as bytes
        private byte[] byteArray;

        // A timer that decides how ofter we should read from the file.This should be equal to the simulated sampling time;
        private System.Timers.Timer pushTimer;

        // The path to the server that we push data to.
        //private static string serverPath = "http://localhost:8080/czml";
        public static string serverPath = "https://sscflightdata.herokuapp.com/czml";

        private static HttpWebRequest request = (HttpWebRequest)WebRequest.Create(serverPath);

        // The object that takes care of the HTTP connection    
        private static HttpClient client = new HttpClient();
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
        System.Timers.Timer testTimer;

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
            public JulianDate Time = new JulianDate();
            public string RAWmessage = "";
            public int Satellites = 0;

            public GPSposition(double longitude, double latitude, double altitude, JulianDate time, string raw, int satellites)
            {
                Longitude = longitude;
                Latitude = latitude;
                Altitude = altitude - 330;
                Time = time;
                RAWmessage = raw;
                Satellites = satellites;
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
                comboBox3.Items.Add(s);
            }

            foreach (string s in modes)
            {
                comboBox2.Items.Add(s);
            }

            comboBox1.SelectedIndex = 0;
            comboBox2.SelectedIndex = 0;
            comboBox3.SelectedIndex = 0;


            //BindUPD();
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
                    //add2listBox("IP address: " + ip);
                }
            }

            return values;
        }
        
        private void BindUPD()
        {
            //Take the ip-adress of the selected item in the dropdown box
            RamsesNetworkcardip = comboBox1.SelectedItem.ToString();
            PusherNetworkcardip = comboBox3.SelectedItem.ToString();

            POSNET_udpSock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);


            try
            {
                pusherEndPoint = new IPEndPoint(IPAddress.Parse(PusherNetworkcardip), 0);

                //Open the UDP port and listen to IP as specified in config.ini
                IPEndPoint ipep = new IPEndPoint(IPAddress.Parse(RamsesNetworkcardip), int.Parse(POSNET_UDPport)); //Use the network card with this adress
                //IPEndPoint ipep = new IPEndPoint(IPAddress.Any, int.Parse(UDPport));
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
                //IPEndPoint ipep = new IPEndPoint(IPAddress.Any, int.Parse(UDPport));
                RAMSES_udpSock.Bind(ipep);

                IPAddress ip = IPAddress.Parse(RAMSES_UDPip);

                RAMSES_udpSock.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership, new MulticastOption(ip, IPAddress.Any));


                RAMSES_buffer = new byte[50000];

                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                RAMSES_udpSock.BeginReceiveFrom(RAMSES_buffer, 0, RAMSES_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromRamses, RAMSES_udpSock);

                //add2listBox("UDP bind");
            }
            catch (Exception)
            {
                //MessageBox.Show("UDP socket failed!");
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
            //LastUDPTime = DateTime.Now;
            pkgCounter++;


            try
            {
                //Get the received message.
                Socket recvSock = (Socket)iar.AsyncState;
                EndPoint clientEP = new IPEndPoint(IPAddress.Any, 0);
                int msgLen = recvSock.EndReceiveFrom(iar, ref clientEP);
                //byte[] localMsg = new byte[msgLen];
                //Array.Copy(POSNET_buffer, localMsg, msgLen);

                //if (!Stop)
                //{
                //Start listening for a new message.
                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                POSNET_udpSock.BeginReceiveFrom(POSNET_buffer, 0, POSNET_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromPosnet, POSNET_udpSock);
                //}

                message = Encoding.UTF8.GetString(POSNET_buffer);
                //add2listBox("Data");
                //add2listBox(buffer.ToString());

                //Look for RT time packet on posnet
                if (message.Substring(0, 7) == "$PRBERT")
                {
                    RTtimestr = message.Substring(29, 9);
                    //RTtimestr = "-11:22:33";
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
            //LastUDPTime = DateTime.Now;
            pkgCounter++;

            try
            {
                //Get the received message.
                Socket recvSock = (Socket)iar.AsyncState;
                EndPoint clientEP = new IPEndPoint(IPAddress.Any, 0);
                int msgLen = recvSock.EndReceiveFrom(iar, ref clientEP);
                //byte[] localMsg = new byte[msgLen];
                //Array.Copy(RAMSES_buffer, localMsg, msgLen);

                //if (!Stop)
                //{
                //Start listening for a new message.
                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                RAMSES_udpSock.BeginReceiveFrom(RAMSES_buffer, 0, RAMSES_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromRamses, RAMSES_udpSock);
                //}



                //string msg = System.Text.Encoding.UTF8.GetString(RAMSES_buffer);
                //add2listBox("Data");
                //add2listBox(buffer.ToString());

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
            double latitude = inputDegrees + (inputMinutes / 60) * multiplier;  // 52.632363


            return latitude;
        }



        private void ProcessRAMSESmessage(byte[] RAMSES_buffer)
        {
            try
            {
                //Search the message for Pattern for a new message
                //byte[] pattern = Encoding.ASCII.GetBytes("$PASHR");
                //List<int> PatternsPositions = SearchBytePattern(pattern, RAMSES_buffer);

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

                    //DateTime timeStamp = new DateTime(currentDate.Year, currentDate.Month, currentDate.Day, Convert.ToInt32(time.Substring(0, 2)), Convert.ToInt32(time.Substring(2, 2)), Convert.ToInt32(time.Substring(4, 2)), 10 * Convert.ToInt32(time.Substring(7, 2)));
                    DateTime timeStamp = DateTime.Now;

                    GPSposition p = new GPSposition(longitud, latitude, altitude, new JulianDate(timeStamp), msg, satellites);
                    GPSpositionsReceived.Add(p);

                    //NewGPSpositionsReceived.Add(p);
                    //add2listBox("Number of positions batched: " + GPSpositionsReceived.Count.ToString());
                }
                else if (Packetmatch(RAMSES_buffer, PacketID_GPSIIP_valid))//IIP packet
                {
                    msg = Encoding.UTF8.GetString(RAMSES_buffer, (32 + 17), 59);
                    time = msg.Substring(11, 9); //Time for IIP calculation

                    latitude = DDMToDD(msg.Substring(23, 2), msg.Substring(25, 7), msg.Substring(33, 1));
                    longitud = DDMToDD(msg.Substring(35, 3), msg.Substring(38, 7), msg.Substring(46, 1));

                    //Time to impact, seconds
                    double timeToImpact = double.Parse(msg.Substring(48, 7), CultureInfo.InvariantCulture);

                    //DateTime timeStamp = new DateTime(currentDate.Year, currentDate.Month, currentDate.Day, Convert.ToInt32(time.Substring(0, 2)), Convert.ToInt32(time.Substring(2, 2)), Convert.ToInt32(time.Substring(4, 2)), 10 * Convert.ToInt32(time.Substring(7, 2)));
                    DateTime timeStamp = DateTime.Now;

                    GPS_IIP IIP = new GPS_IIP(longitud, latitude, new JulianDate(timeStamp), msg, timeToImpact);
                    NewGPS_IIP_Received.Add(IIP);
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

                    DateTime timeStamp = DateTime.Now;

                    //Do something fancy with the quaternions
                    Attitude at = new Attitude(floatConversion(bquaternion0), floatConversion(bquaternion1), floatConversion(bquaternion2), floatConversion(bquaternion3), new JulianDate(timeStamp));
                    
                    AttitudeReceived.Add(at);
                    //AttitudeTimes.Add(new JulianDate(DateTime.Now));
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
                    //float missiontime = BitConverter.ToSingle(bmissiontime, 0);

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

                    GLoadsReceived.Add(new GLoads((float)ACCx/1000, (float)ACCy/1000, (float)ACCz/1000));

                    // Scale in order to get degrees per second
                    double scaleFactor = 0.00037 * 360 / 60;
                    ARatesReceived.Add(new AngularRates(GyroX* scaleFactor, GyroY * scaleFactor, GyroZ * scaleFactor)); 

                }
                else if (Packetmatch(RAMSES_buffer, PacketID_THERM)) //If it is a THERM packet. Extract some temperatures maybe?
                {

                    var TGVTemp3 = (short)(RAMSES_buffer[32 + 65 + 0] << 8 | RAMSES_buffer[32 + 65 + 1]);

                    //Use a simplified PT-100 curve
                    double temp = 0.05 * (double)(TGVTemp3 - 5568);
                    TemperatureTest = temp.ToString("0.0") + "°C";

                }


            }
            catch
            {
                add2listBox("error!");
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

            //Time string from RT counter
            label1.Text = RTtimestr;

            //GPS position messages
            if (GPSpositionsReceived.Count > 0)
            {
                label3.Text = "GPS: " + GPSpositionsReceived.Count.ToString();
                label4.Text = GPSpositionsReceived[GPSpositionsReceived.Count - 1].Time.ToString();

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

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (NewDataFlag == true)
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
                testTimer = new System.Timers.Timer();
                testTimer.Elapsed += new System.Timers.ElapsedEventHandler(onTestEvent);
                testTimer.Interval = 100;
                testTimer.Enabled = true;
                testTimer.AutoReset = true;
                testTimer.Start();
            }

            // Initialize the data push:
            pushInitialPacket();

            // If test mode is chosen

            pushTimer = new System.Timers.Timer();
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

                    double secondsSince = linecounter * 0.1 - 90;

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
                        RTtimestr = "+00:" + minutesString + ":" + secondsString;
                    }
                    else
                    {
                        RTtimestr = "-00:" + minutesString + ":" + secondsString;
                    }

                    if (secondsSince > -100)
                    {
                        var line = CSVreader.ReadLine();
                        var values = line.Split(';');

                        //time = Convert.ToDouble(values[0]);
                        //speed = Convert.ToDouble(values[4]);
                        testLong = Convert.ToDouble(values[16]);
                        testLat = Convert.ToDouble(values[17]);
                        testAlt = Convert.ToDouble(values[39]) * 1000;
                        //yaw = Convert.ToDouble(values[46]);
                        //pitch = Convert.ToDouble(values[45]);
                        //roll = Convert.ToDouble(values[44]);
                        DateTime timeStamp = DateTime.Now;
                        if (GPSTestCheckBox.Checked)
                        {
                            //add2listBox("Testing GPS");

                            GPSposition p = new GPSposition(testLong, testLat, testAlt, new JulianDate(timeStamp), "msg", 11);
                            GPSpositionsReceived.Add(p);

                            //NewGPSpositionsReceived.Add(p);
                        }

                        if (attitudeTestCheckBox.Checked)
                        {
                            Vector<double> testQuat = Vector<double>.Build.Dense(4);

                            testQuat[0] = rand.NextDouble();
                            testQuat[1] = rand.NextDouble();
                            testQuat[2] = rand.NextDouble();
                            testQuat[3] = rand.NextDouble();

                            testQuat = testQuat.Normalize(2);

                            //NewAttitudeReceived.Add(new Attitude((float)testQuat[0], (float)testQuat[1], (float)testQuat[2], (float)testQuat[3]));
                            AttitudeReceived.Add(new Attitude((float)testQuat[0], (float)testQuat[1], (float)testQuat[2], (float)testQuat[3], new JulianDate(DateTime.Now)));
                        }


                        if (gLoadTestCheckBox.Checked)
                        {
                            GLoads gLoads = new GLoads(float.Parse(values[41]), float.Parse(values[42]), float.Parse(values[43]));
                            //NewGLoadsReceived.Add(gLoads);
                            GLoadsReceived.Add(gLoads);
                        }

                        if (aRateTestCheckBox.Checked)
                        {
                            testARates = new AngularRates((float)rand.NextDouble() * 100, (float)rand.NextDouble() * 100, (float)rand.NextDouble() * 100);
                            //NewARatesReceived.Add(testARates);
                            ARatesReceived.Add(testARates);
                        }

                        if (IIPTestCheckbox.Checked)
                        {
                            GPS_IIP testIIP = new GPS_IIP(20.6791, 68.5443, new JulianDate(timeStamp), "what", 20);
                            //NewGPS_IIP_Received.Add(testIIP);
                            GPS_IIP_Received.Add(testIIP);
                        }
                    }


                }
                catch (Exception except)
                {
                    add2listBox("Failed to push test data: " + except.StackTrace.ToString());
                }



                linecounter++;
                //NewDataFlag = true;
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
            extrapolationDuration = new Duration(0, 0.001);

            // First bracket in the array
            output.WriteStartSequence();

            // Add a new packet
            packet = writer.OpenPacket(output);

            // The timeinterval of which the packet is defined. Basically
            TimeInterval timeInterval = new TimeInterval(new JulianDate(DateTime.Now), new JulianDate(3456293));

            // Writing the document package, which is the first package that the client must receive
            packet.WriteId("document");
            packet.WriteName("initial-packet");
            packet.WriteVersion("1.0");
            clock = packet.OpenClockProperty();
            clock.WriteInterval(timeInterval);
            clock.WriteCurrentTime(new JulianDate(DateTime.Now));
            clock.Close();

            // Close first package
            packet.Close();

            // Closing bracket
            output.WriteEndSequence();

            // URL that points to the model
            //modelURL = "3Dmodels/Cesium_Air.glb";
            modelURL = "3Dmodels/maxus_mockup.glb";
            //string modelURL = "https://flight-data-visualization.herokuapp.com//Apps/SampleData/models/CesiumGround/Cesium_Ground.glb";
            //modelURL = "3Dmodels/rocket.glb";

            //Populating the byte array with the data written
            byteArray = Encoding.UTF8.GetBytes(sw.ToString());

            // Posting the data to the server
            var responseStatus = postData(byteArray);

            // Closing the string writer
            sw.Close();
        }

        private void add2listBox(string listBoxStr)
        {
            try
            {
                listBox1.Invoke((MethodInvoker)(() => listBox1.Items.Add(listBoxStr)));
            }
            catch(Exception except)
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
            }catch(Exception except)
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

            System.Diagnostics.Stopwatch watch_listBox = new System.Diagnostics.Stopwatch();
            watch_listBox.Start();
            add2listBox("A new data push was initialized");

            watch_listBox.Stop();
            add2listBox("------------------------------------Time of listBox execution: " + watch_listBox.ElapsedMilliseconds.ToString());

           



            // Check if the properties are updated
            bool GPSUpdated;
            bool attitudeUpdated;
            bool speedUpdated = false;
            bool aRatesUpdated;
            bool gLoadsUpdated;
            bool eventStringUpdated;
            bool IIPUpdated;
            bool timeUpdated;

            int GPSStartIndex = 0;
            int GPSEndIndex = 0;
            int GLoadStartIndex = 0;
            int GLoadEndIndex = 0;
            int AttitudeStartIndex = 0;
            int AttitudeEndIndex = 0;
            int ARateStartIndex = 0;
            int ARateEndIndex = 0;

            List<GPSposition> NewGPSpositionsReceived = new List<GPSposition>();
            List<GPSposition> OldGPSpositionsReceived = new List<GPSposition>();
            List<Attitude> NewAttitudeReceived = new List<Attitude>();
            List<Attitude> OldAttitudeReceived = new List<Attitude>();
            List<GLoads> NewGLoadsReceived = new List<GLoads>();
            List<AngularRates> NewARatesReceived = new List<AngularRates>();

            

            string expectedAttitudeFormat = "quaternion"; // Maybe add this to the user interface?

            GPSUpdated = false;
            attitudeUpdated = false;
            gLoadsUpdated = false;
            aRatesUpdated = false;
            IIPUpdated = false;
            timeUpdated = false;

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
                }

                if (ARatesReceived.Count > maxARateIndex + 1)
                {
                    aRatesUpdated = true;
                    ARateStartIndex = maxARateIndex;
                    ARateEndIndex = ARatesReceived.Count - 1;
                    maxARateIndex = ARateEndIndex;
                    NewARatesReceived = ARatesReceived.GetRange(ARateStartIndex, ARateEndIndex - ARateStartIndex + 1);
                }

                if (GPS_IIP_Received.Count > maxGPS_IIP_Index + 1)
                {
                    IIPUpdated = true;
                    //GPS_IIP_StartIndex = maxGPS_IIP_Index;
                    //GPS_IIP_EndIndex = GPS_IIP_Received.Count - 1;
                    //maxGPS_IIP_Index = GPS_IIP_EndIndex;
                }

                if (RTtimestr != oldRTtimestr)
                {
                    timeUpdated = true;
                    //add2listBox("Time updated");
                    oldRTtimestr = RTtimestr;
                }
            }catch(Exception except){
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

                        if (GPSUpdated)
                        {
                            launchTime = NewGPSpositionsReceived[0].Time;
                        }
                        else
                        {
                            launchTime = new JulianDate(DateTime.Now);
                        }
                        launched = true;
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to check the time: " + except.StackTrace.ToString());
                }
            }

            watch_checkTime.Stop();
            add2listBox("------------------------------------Time to execute the time check: " + watch_checkTime.ElapsedMilliseconds);

            
            System.Threading.Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

         
            // Dates and positions lists, in case we want to sample more data before writing to server.
            List<JulianDate> dates = new List<JulianDate>();
            List<Cartographic> positions = new List<Cartographic>();
            List<UnitQuaternion> orientations = new List<UnitQuaternion>();
            List<double> speeds = new List<double>();
            double speed = 0;
            Cartesian gLoads = new Cartesian();
            Cartesian aRates = new Cartesian();

            // Trying to write the start sequence
            try
            {
                // Initializing a new json array
                output.WriteStartSequence();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write start sequence: " + except.StackTrace.ToString());
            }

            try
            {
                // Open a new packet for the rocket model
                //add2listBox("Opening rocket packet");
                packet = writer.OpenPacket(output);
                packet.WriteId("rocket");
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

                int maximumBackTrack = launched ? 5 : 50;
                int maxBackTrack = OldGPSpositionsReceived.Count > maximumBackTrack ? maximumBackTrack : OldGPSpositionsReceived.Count;
                try
                {
                    for (int iPos = 0; iPos < NewGPSpositionsReceived.Count; iPos++)
                    {
                        GPSposition tempPosition = NewGPSpositionsReceived[iPos];
                        double tempLong = 0;
                        double tempLat = 0;
                        double tempAlt = 0;
                        for (int i = 0; i < maxBackTrack; i++)
                        {
                            if (i > iPos)
                            {
                                tempLong += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Longitude;
                                tempLat += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Latitude;
                                tempAlt += OldGPSpositionsReceived[OldGPSpositionsReceived.Count + iPos - i].Altitude;
                            }
                            else
                            {
                                tempLong += NewGPSpositionsReceived[iPos - i].Longitude;
                                tempLat += NewGPSpositionsReceived[iPos - i].Latitude;
                                tempAlt += NewGPSpositionsReceived[iPos - i].Altitude;
                            }
                            
                        }
                        add2listBox("Alt: " + (tempAlt).ToString());
                        if (maxBackTrack > 0)
                        {
                            Cartographic tempCart = new Cartographic(tempLong / maxBackTrack, tempLat / maxBackTrack, tempAlt / maxBackTrack);
                            positions.Add(tempCart);
                            oldCartographic = tempCart;
                            oldGPSDataExists = true;
                        }
                        else
                        {
                            Cartographic tempCart = new Cartographic(tempPosition.Longitude, tempPosition.Latitude, tempPosition.Altitude);
                            positions.Add(tempCart);
                            oldCartographic = tempCart;
                            oldGPSDataExists = true;
                        }
                        PositionTimes.Add(tempPosition.Time);
                        add2listBox(tempPosition.Time.ToString());
                    }
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to add the positions: " + except.StackTrace.ToString());
                }

                if (NewGPSpositionsReceived.Count > 1)
                {
                    try
                    {
                        if (launched)
                        {
                            Vector<double> firstSamplePos = GPS2cartesian(NewGPSpositionsReceived[0].Longitude, NewGPSpositionsReceived[0].Latitude, NewGPSpositionsReceived[0].Altitude);
                            Vector<double> lastSamplePos = GPS2cartesian(NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Longitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Latitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Altitude);
                            speed = positions2speed(NewGPSpositionsReceived);

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

                //add2listBox(positions.Count.ToString() + "-----" + PositionTimes.Count.ToString());

            }
            else
            {
                // If no new GPS data is received but we have some old data, assume that data stream has initiated, but that it might have been interupted.
                if (oldGPSDataExists)
                {
                    try
                    {
                        // Re-send the last known position if data is missing
                        positions.Add(oldCartographic);
                        PositionTimes.Add(new JulianDate(DateTime.Now));
                        //add2listBox("Pushing old GPS data");
                    }
                    catch (Exception except)
                    {
                        MessageBox.Show("Failed to open and write the rocket position packet: " + except.StackTrace.ToString());
                    }
                }
            }
            watch_GPS.Stop();
            add2listBox("------------------------------------Time to execute the GPS write: " + watch_GPS.ElapsedMilliseconds);

            if (positions.Count > 0)
            {
                if (PositionTimes.Count > positions.Count)
                {
                    PositionTimes = PositionTimes.GetRange(0, positions.Count);
                }
                else if (PositionTimes.Count < positions.Count)
                {
                    positions = positions.GetRange(0, PositionTimes.Count);
                }
            }

            if (positions.Count > 0)
            {
                try
                {
                    // Open the positions packet
                    //add2listBox("Opening the rocket position packet");
                    position = packet.OpenPositionProperty();

                    // The first position packet states the inter- and extrapolating properties [Do we need this?]
                    if (firstPositionPacket)
                    {
                        position.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Hermite);
                        position.WriteInterpolationDegree(2);
                        position.WriteForwardExtrapolationDuration(extrapolationDuration);
                        position.WriteBackwardExtrapolationDuration(extrapolationDuration);
                        position.WriteForwardExtrapolationType(CesiumExtrapolationType.Extrapolate);
                        firstPositionPacket = false;
                    }

                    position.WriteCartographicDegrees(PositionTimes, positions);

                    //add2listBox("Closing the rocket position packet");
                    position.Close();
                }
                catch (Exception except)
                {

                    MessageBox.Show("Failed to write positions: " + except.StackTrace.ToString());
                }
            }

            System.Diagnostics.Stopwatch watch_attitude = new System.Diagnostics.Stopwatch();

            watch_attitude.Start();

            if (attitudeUpdated)
            {
                // INCLUDE A TIMER HERE TO KEEP TRACK ON THE TIME BETWEEN NEW DATA POINTS
                foreach (Attitude tempAttitude in NewAttitudeReceived)
                {
                    if (expectedAttitudeFormat == "quaternion")
                    {
                        UnitQuaternion tempQuat = TH2ECEF(tempAttitude.q0, tempAttitude.q1, tempAttitude.q2, tempAttitude.q3);
                        orientations.Add(tempQuat);
                        oldRealQuaternion = tempQuat;
                        oldRealAttitudeDataExists = true;
                        AttitudeTimes.Add(tempAttitude.Time);
                    }
                }
                //add2listBox("Pushing new attitude data");
            }
            else if (oldRealAttitudeDataExists)
            {
                orientations.Add(oldRealQuaternion);
                AttitudeTimes.Add(new JulianDate(DateTime.Now));
            }
            else if(GPSUpdated && GPSStartIndex > 0)
            {
                // Otherwise, assume that attitude data is NOT available, and make sure that the attitude points in the direction of flight.
                try
                {
                    List<UnitQuaternion> tempAttitude = attitudeFromGPS(NewGPSpositionsReceived, OldGPSpositionsReceived, GPSStartIndex);
                    if (tempAttitude.Count > 0)
                    {
                        oldCalculatedQuaternion = tempAttitude[tempAttitude.Count - 1];
                        oldCalculatedAttitudeDataExists = true;
                    }
                    orientations.AddRange(tempAttitude);
                    AttitudeTimes = PositionTimes;
                    //add2listBox("Number of new GPS positions: " + NewGPSpositionsReceived.Count.ToString());
                    //add2listBox("Total number of GPS positions: " + GPSpositionsReceived.Count.ToString());
                    //add2listBox("Number of new Attitude positions: " + orientations.Count.ToString());
                    //oldQuaternion = tempAttitude[tempAttitude.Count - 1];
                    //oldAttitudeDataExists = true;
                }
                catch (Exception except)
                {
                    MessageBox.Show("Failed to calculate attitude from GPS data: " + except.StackTrace.ToString());
                }
            } else if (oldCalculatedAttitudeDataExists)
            {
                orientations.Add(oldCalculatedQuaternion);
                AttitudeTimes.Add(new JulianDate(DateTime.Now));
            }

            if (orientations.Count > 0)
            {
                if (AttitudeTimes.Count > orientations.Count)
                {
                    AttitudeTimes = AttitudeTimes.GetRange(0, orientations.Count);
                }
                else if (AttitudeTimes.Count < orientations.Count)
                {
                    orientations = orientations.GetRange(0, AttitudeTimes.Count);
                }
            }

            if (orientations.Count > 0)
            {
                // The first orientation packet states the inter- and extrapolating properties
                
                try
                {
                    // Open the orientation packet
                    //add2listBox("Opening rocket orientation packet");
                    orientation = packet.OpenOrientationProperty();

                    if (firstOrientationPacket)
                    {
                        orientation.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Hermite);
                        orientation.WriteInterpolationDegree(1);
                        orientation.WriteForwardExtrapolationDuration(extrapolationDuration);
                        orientation.WriteForwardExtrapolationType(CesiumExtrapolationType.Extrapolate);
                        firstOrientationPacket = false;
                    }
                }
                catch (Exception except)
                {
                    add2listBox("Failed to open the orientation packet: " + except.StackTrace.ToString());
                }

                try
                {
                    //add2listBox("The orientations: " + orientations[0].ToString());
                    orientation.WriteUnitQuaternion(AttitudeTimes, orientations);
                }
                catch (Exception argE)
                {
                    add2listBox("Failed to write the attitudes. " + argE.StackTrace.ToString());
                }


                try
                {
                    //add2listBox("Closing rocket orientation packet");
                    orientation.Close();
                }
                catch (Exception except)
                {
                    add2listBox("Failed to close the attitude packet: " + except.StackTrace.ToString());
                }
            }

            watch_attitude.Stop();
            add2listBox("------------------------------------Time to execute attitude write: " + watch_attitude.ElapsedMilliseconds);

            // Write the model packet, and finally close the rocket packet
            try
            {
                // Opening the model packet
                //add2listBox("Opening rocket model packet");
                model = packet.OpenModelProperty();
                model.WriteGltfProperty(modelURL, CesiumResourceBehavior.LinkTo);
                //model.WriteMinimumPixelSizeProperty(128);

                // Correcting the scale
                model.WriteScaleProperty(0.001);
                model.WriteMaximumScaleProperty(1);
                //add2listBox("Closing rocket model packet");
                model.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to write model packet: " + except.StackTrace.ToString());
            }

            try
            {
                // Close packet when everything is appended
                //add2listBox("Closing rocket packet");
                packet.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to close rocket packet: " + except.StackTrace.ToString());
            }

            try
            {
                // Open the packet used for the speed, g-forces, and possibly events.
                packet = writer.OpenPacket(output);
                packet.WriteId("speed_gLoads");
                string eventString = eventStringFromList(missionSeconds);
                packet.WriteName(eventString);
            }
            catch (Exception except)
            {
                add2listBox("Failed to open the packet for speed and G-loads: " + except.StackTrace.ToString());
            }

            // Opening the point packet (this is an abstract point, it will not be visible)
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
                    gLoads = NewGLoadsReceived[NewGLoadsReceived.Count - 1].toCartesian();
                    gLoadPacket.WriteCartesian(gLoads);
                    gLoadPacket.Close();
                }
            }

            try
            {
                // Closing the point packet
                point.Close();
                // Close packet when everything is appended
                packet.Close();
            }
            catch (Exception except)
            {
                add2listBox("Failed to close the packet for speed and G-loads: " + except.StackTrace.ToString());
            }



            try
            {
                // Open the packet used for the angular rates and time.
                packet = writer.OpenPacket(output);
                packet.WriteId("angularRates_time");
                packet.WriteName(RTtimestr);
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to open the packet for angular rates and time: " + except.StackTrace.ToString());
            }

            try
            {
                // Opening the point packet (this is an abstract point, it will not be visible)
                point = packet.OpenPointProperty();
                point.WriteShowProperty(false);
            }
            catch (Exception except)
            {
                add2listBox("Failed to open the point packet for time and angular rates: " + except.StackTrace.ToString());
            }

            try
            {
                //add2listBox(launchTime.ToString());
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
                else
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
                add2listBox(except.StackTrace.ToString());
            }

            if (aRatesUpdated)
            {

                if (NewARatesReceived.Count > 0)
                {
                    try
                    {
                        aRatePacket = packet.OpenPositionProperty();
                        aRates = NewARatesReceived[NewARatesReceived.Count - 1].toCartesian();
                        aRatePacket.WriteCartesian(aRates);
                        aRatePacket.Close();
                    }
                    catch (Exception except)
                    {
                        add2listBox("Failed to write the aRate packet: " + except.StackTrace.ToString());
                    }
                }
            }

            try
            {
                // Closing the point packet
                point.Close();
                // Close packet when everything is appended
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
            }
            catch (Exception except)
            {
                add2listBox("Failed to open the packet for IIP: " + except.StackTrace.ToString());
            }

            try
            {
                // Opening a point packet, which will be used to show the IIP
                point = packet.OpenPointProperty();
                point.WriteShowProperty(false);
            }
            catch (Exception except)
            {
                add2listBox("Failed to close the point packet for IIP: " + except.StackTrace.ToString());
            }

            if (IIPUpdated)
            {
                IIPPacket = packet.OpenPositionProperty();
                Cartographic IIP = GPS_IIP_Received[GPS_IIP_Received.Count - 1].toCartographic();
                IIPPacket.WriteCartographicDegrees(IIP);
                IIPPacket.Close();
            }

            try
            {
                // Closing the point packet
                point.Close();
                // Close packet when everything is appended
                packet.Close();
            }
            catch (Exception except)
            {
                add2listBox("Failed to close the packet for IIP: " + except.StackTrace.ToString());
            }

            try
            {
                // Opening the last packet, which will be used to store the previous positions, used if a client connects mid-flight
                packet = writer.OpenPacket(output);
                //add2listBox("Writing flight path");
                packet.WriteId("flightPath");
            }
            catch (Exception except)
            {
                add2listBox("Failed to open flight path packet: " + except.StackTrace.ToString());
            }

            System.Diagnostics.Stopwatch watch_flightPath = new System.Diagnostics.Stopwatch();

            watch_flightPath.Start();
            try
            {
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
            }
            catch (Exception except)
            {
                add2listBox("Failed to open flight path polyline packet: " + except.StackTrace.ToString());
            }

            watch_flightPath.Stop();
            add2listBox("------------------------------------Time to write flight path: " + watch_flightPath.ElapsedMilliseconds);

            try
            {
                polyLine.Close();
                packet.Close();
            }
            catch (Exception except)
            {
                add2listBox("Failed to close flight path packet: " + except.StackTrace.ToString());
            }

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
                byteArray = Encoding.UTF8.GetBytes(sw.ToString());

                //add2listBox(sw.ToString());

                // Close the string writer
                sw.Close();
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to convert sw to a byte array: " + except.StackTrace.ToString());
            }

            System.Diagnostics.Stopwatch watch_postData = new System.Diagnostics.Stopwatch();
            watch_postData.Start();

            //add2listBox("Pushing...");
            new System.Threading.Thread(delegate () {
                var responseStatus = postData(byteArray);
            }).Start();
            

            watch_postData.Stop();
            add2listBox("------------------------------------Time to post data: " + watch_postData.ElapsedMilliseconds);

            try
            {
                // Resetting some variables
                NewGPSpositionsReceived = new List<GPSposition>();
                NewGPS_IIP_Received = new List<GPS_IIP>();
                NewAttitudeReceived = new List<Attitude>();
                NewGLoadsReceived = new List<GLoads>();
                NewARatesReceived = new List<AngularRates>();
                PositionTimes = new List<JulianDate>();
                AttitudeTimes = new List<JulianDate>();
                //}
            }
            catch (Exception except)
            {
                MessageBox.Show("Failed to reset the lists: " + except.StackTrace.ToString());
            }

            watch_tot.Stop();
            add2listBox("------------------------------------Total time to push data: " + watch_tot.ElapsedMilliseconds);
        }


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

            // Defining cos and sin of the euler angles
            double cPitch = Math.Cos(deg2rad * pitch);
            double sPitch = Math.Sin(deg2rad * pitch);

            double cYaw = Math.Cos(deg2rad * yaw);
            double sYaw = Math.Sin(deg2rad * yaw);

            double cRoll = Math.Cos(deg2rad * roll);
            double sRoll = Math.Sin(deg2rad * roll);

            // Constructing the DCM from the quaternions
            qDMC[0, 0] = Math.Pow(q0, 2) + Math.Pow(q1, 2) + Math.Pow(q2, 2) + Math.Pow(q3, 2);
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

            // Markleys algorithm -------------------

            Vector<double> quaternionVector = markley(ECEF2body);

            return new UnitQuaternion(quaternionVector[0], quaternionVector[1], quaternionVector[2], quaternionVector[3]);
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
            x2[1] = ECEF2body[1, 0] + ECEF2body[0, 1];
            x2[2] = 1 + 2 * ECEF2body[1, 1] - trace;
            x2[3] = ECEF2body[1, 2] + ECEF2body[2, 1];
            vectorList.Add(x2);

            Vector<double> x3 = Vector<double>.Build.Dense(4);
            x3[0] = ECEF2body[0, 1] - ECEF2body[1, 0];
            x3[1] = ECEF2body[2, 0] + ECEF2body[0, 2];
            x3[2] = ECEF2body[2, 1] + ECEF2body[1, 2];
            x3[3] = 1 + 2 * ECEF2body[2, 2] - trace;
            vectorList.Add(x3);

            int maxIndex = 0;
            double maxMagnitude = 0;

            for (int i = 0; i < vectorList.Count; i++)
            {
                //Console.WriteLine(vectorList[i].ToString());
                if (vectorList[i].L2Norm() > maxMagnitude)
                {
                    maxMagnitude = vectorList[i].L2Norm();
                    maxIndex = i;
                }
            }

            return vectorList[maxIndex].Divide(maxMagnitude);
        }

        /*  */
        private Vector<double> GPS2cartesian(double longitude, double latitude, double height)
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

                    JulianDate firstTime = GPSpos[i].Time;
                    JulianDate secondTime = GPSpos[i + 1].Time;
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
                int maxBackTrack = OldGPSposList.Count - 1 > 10 ? 10 : OldGPSposList.Count - 1;

                Vector<double> xVector = Vector<double>.Build.Dense(3);
                xVector[0] = 1;
                xVector[1] = 0;
                xVector[2] = 0;

                Vector<double> quat = Vector<double>.Build.Dense(4);

                for (int i = 0; i < NewGPSposList.Count; i++)
                {
                    Vector<double> tempQuat = Vector<double>.Build.Dense(4);
                    if (i <= maxBackTrack - 1)
                    {
                        //add2listBox(maxBackTrack.ToString() + " " + startIndex.ToString() + " " + i.ToString());
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

                        //tempQuat[0] = directionOfFlight.DotProduct(halfVector);
                        tempQuat[0] = halfVector.DotProduct(directionOfFlight);

                        //Vector<double> subQuat = crossProduct(directionOfFlight, halfVector);
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

        private static IPEndPoint BindIPEndPointCallback(ServicePoint servicePoint, IPEndPoint remoteEndPoint, int retryCount)
        {
            if (retryCount < 3)
                return new IPEndPoint(IPAddress.Parse(PusherNetworkcardip), 0);
            else
                return new IPEndPoint(IPAddress.Any, 0);
        }

        /* postData is handles the HTTP request. */
        public static async Task<string> postData(byte[] data)
        {
            try
            {
                request = (HttpWebRequest)WebRequest.Create(serverPath);
                request.Method = "POST";
                request.ContentType = "application/json";
                request.ContentLength = data.Length;
                //request.ServicePoint.BindIPEndPointDelegate = new BindIPEndPoint(BindIPEndPointCallback);
                using (var stream = request.GetRequestStream())
                {
                    stream.Write(data, 0, data.Length);
                }

                var response = (HttpWebResponse)request.GetResponse();

                var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();

                //response.Close();
                return "";
                //return response.StatusCode.ToString();

                //// Convert data to a type that httpClient can handle
                //ByteArrayContent byteContent = new ByteArrayContent(data);
                //byteContent.Headers.ContentType = new MediaTypeHeaderValue("application/json");
                //HttpResponseMessage response = await client.PostAsync(serverPath, byteContent);
                //Console.WriteLine(response.Content.ReadAsStringAsync().ToString());
                ////Return the URI of the created resource.
                //return response.IsSuccessStatusCode;
            }
            catch (Exception except)
            {
                MessageBox.Show("postData failed to send data to the server: " + except.StackTrace.ToString());
                MessageBox.Show(System.Text.Encoding.Default.GetString(data));
                return "";
                //return false;
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

       
    }
}
