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

        //Identifiers for different packet types
        private PacketIdentifier PacketID_GPSpos_Fix = new PacketIdentifier(0x08, 0x20, 0x00, 0x01, "GPS packet with a valid position");
        private PacketIdentifier PacketID_GPSpos_NoFix = new PacketIdentifier(0x08, 0x20, 0x00, 0x01, "GPS packet without valid position");
        private PacketIdentifier PacketID_GPSIIP_valid = new PacketIdentifier(0x08, 0x21, 0x00, 0x01, "IIP packet with valid navigation data");
        private PacketIdentifier PacketID_GPSIIP_invalid = new PacketIdentifier(0x08, 0x21, 0x00, 0x00, "IIP packet without navigation data");
        private PacketIdentifier PacketID_GCS = new PacketIdentifier(0x08, 0x30, -1, -1, "GCS 50Hz data");
        private PacketIdentifier PacketID_PCDU = new PacketIdentifier(0x08, 0x0D, 0x00, -1, "PCDU data");
        private PacketIdentifier PacketID_THERM = new PacketIdentifier(0x08, 0x10, -1, 0x01, "Thermo board data packet");

        /*--------------------------------------------------------------------*/
        /*----------------------------Data pusher-----------------------------*/

        // WGS ellipsoid data
        private double eccentricity = 0.081819190842622;
        private double semiMajorAxis = 6378137;
        private Vector<double> xAxis = Vector<double>.Build.DenseOfArray(new double[3] { 1, 0, 0 });
        private Vector<double> yAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 1, 0 });
        private Vector<double> zAxis = Vector<double>.Build.DenseOfArray(new double[3] { 0, 0, 1 });

        private List<GPSposition> NewGPSpositionsReceived = new List<GPSposition>();
        private List<JulianDate> PositionTimes = new List<JulianDate>();
        private List<Attitude> NewAttitudeReceived = new List<Attitude>();
        private List<JulianDate> AttitudeTimes = new List<JulianDate>();

        // Check if mission time goes from minus to plus
        string plusminusTime = "-";

        // Check if the properties are updated
        private bool GPSUpdated;
        private bool attitudeUpdated;
        private bool speedUpdated;
        private bool angularRatesAvailable;
        private bool gLoadsAvailable;
        private bool eventStringUpdated;
        private bool impactPointUpdated;

        // The frequency of which we will push data to the visualization server.
        private static double pushFrequency = 2;

        // Only sending data that are updated, and "static" data are sent in the first packets
        private bool firstPositionPacket = true;
        private bool firstOrientationPacket = true;

        // The different czml-writers used
        private static StringWriter sw = new StringWriter();
        private CesiumOutputStream output = new CesiumOutputStream(sw);
        private CesiumStreamWriter writer = new CesiumStreamWriter();
        private PositionCesiumWriter position;
        private PositionCesiumWriter gLoadPacket; // Will write the g-forces as the coordinates of an abstract point
        private PositionCesiumWriter aRatePacket; // Will write the angular rates as the coordinates of an abstract point
        private OrientationCesiumWriter orientation;
        private ModelCesiumWriter model;
        private PointCesiumWriter point;
        private PathCesiumWriter path;
        private PolylineCesiumWriter polyLine;
        private ClockCesiumWriter clock;
        private PacketCesiumWriter packet;

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

        // The duration for how long we are extrapolating
        private Duration extrapolationDuration;

        // Constant used for converting degrees to radians
        private double deg2rad = Math.PI / 180;

        // An array in which we store the data as bytes
        private byte[] byteArray;

        // A timer that decides how ofter we should read from the file.This should be equal to the simulated sampling time;
        private System.Timers.Timer pushTimer;

        // The path to the server that we push data to.
        private static string serverPath = "http://localhost:8080/czml";
        //public static string serverPath = "https://flight-data-visualization.herokuapp.com/czml";
        //public static string serverPath = "https://api.heroku.com/status";

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
        private StreamReader CSVreader = new StreamReader(File.OpenRead(@"C:\Users\Johan\Dropbox\Exjobb\EX200 - Development\DataSender\VSB-predicted-flight.csv"));
        private int linecounter = 0;


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
                Altitude = altitude;
                Time = time;
                RAWmessage = raw;
                Satellites = satellites;
            }
        }

        class GPS_IIP
        {
            public double Longitude = 0;
            public double Latitude = 0;
            public string Time = "";
            public string RAWmessage = "";
            public double TimeToImpact = 0;

            public GPS_IIP(double longitude, double latitude, string time, string raw, double timeToImpact)
            {
                Longitude = longitude;
                Latitude = latitude;
                Time = time;
                RAWmessage = raw;
                TimeToImpact = timeToImpact;
            }
        }

        class Attitude
        {
            public float q0 = 0;
            public float q1 = 0;
            public float q2 = 0;
            public float q3 = 0;

            public Attitude(float Q0, float Q1, float Q2, float Q3)
            {
                q0 = Q0;
                q1 = Q1;
                q2 = Q2;
                q3 = Q3;
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

            public Cartesian toCartesian() {
                return new Cartesian(Gx, Gy, Gz);
            }
        }

        class AngularRates
        {
            public float aRateX = 0;
            public float aRateY = 0;
            public float aRateZ = 0;

            public AngularRates(float ARateX, float ARateY, float ARateZ)
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
        string RTtimestr = "";

        public Form1()
        {
            InitializeComponent();

            //get a list of all availible network adapters
            foreach (string s in net_adapters())
            {
                comboBox1.Items.Add(s);
            }
            comboBox1.SelectedIndex = 0;

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
                    //listBox1.Items.Add("IP address: " + ip);
                }
            }

            return values;
        }

        private void BindUPD()
        {
            //Take the ip-adress of the selected item in the dropdown box
            RamsesNetworkcardip = comboBox1.SelectedItem.ToString();

            POSNET_udpSock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);


            try
            {
                //Open the UDP port and listen to IP as specified in config.ini
                IPEndPoint ipep = new IPEndPoint(IPAddress.Parse(RamsesNetworkcardip), int.Parse(POSNET_UDPport)); //Use the network card with this adress
                //IPEndPoint ipep = new IPEndPoint(IPAddress.Any, int.Parse(UDPport));
                POSNET_udpSock.Bind(ipep);

                IPAddress ip = IPAddress.Parse(POSNET_UDPip);

                POSNET_udpSock.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership, new MulticastOption(ip, IPAddress.Any));


                POSNET_buffer = new byte[5000];

                EndPoint newClientEP = new IPEndPoint(IPAddress.Any, 0);
                POSNET_udpSock.BeginReceiveFrom(POSNET_buffer, 0, POSNET_buffer.Length, SocketFlags.None, ref newClientEP, DoReceiveFromPosnet, POSNET_udpSock);

                listBox1.Items.Add("UDP bind");
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

                //listBox1.Items.Add("UDP bind");
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

                message = System.Text.Encoding.UTF8.GetString(POSNET_buffer);
                //listBox1.Items.Add("Data");
                //listBox1.Items.Add(buffer.ToString());

                //Look for RT time packet on posnet
                if (message.Substring(0, 7) == "$PRBERT")
                {
                    RTtimestr = message.Substring(29, 9);
                }

                NewDataFlag = true;

            }
            catch (ObjectDisposedException)
            {
                listBox1.Items.Add("exception");
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
                //listBox1.Items.Add("Data");
                //listBox1.Items.Add(buffer.ToString());

                ProcessRAMSESmessage(RAMSES_buffer);

                NewDataFlag = true;


            }
            catch (ObjectDisposedException)
            {
                MessageBox.Show("error");
                listBox1.Items.Add("exception");
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

                    DateTime timeStamp = new DateTime(currentDate.Year, currentDate.Month, currentDate.Day, Convert.ToInt32(time.Substring(0, 2)), Convert.ToInt32(time.Substring(2, 2)), Convert.ToInt32(time.Substring(4, 2)), 10 * Convert.ToInt32(time.Substring(7, 2)));

                    GPSposition p = new GPSposition(longitud, latitude, altitude, new JulianDate(timeStamp), msg, satellites);
                    //GPSpositionsReceived.Add(p);

                    NewGPSpositionsReceived.Add(p);
                    listBox1.Items.Add("Number of positions batched: " + NewGPSpositionsReceived.Count.ToString());
                    listBox1.Items.Add("Number of times batched: " + PositionTimes.Count.ToString());
                }
                else if (Packetmatch(RAMSES_buffer, PacketID_GPSIIP_valid))//IIP packet
                {

                    msg = Encoding.UTF8.GetString(RAMSES_buffer, (32 + 17), 59);
                    time = msg.Substring(11, 9); //Time for IIP calculation

                    latitude = DDMToDD(msg.Substring(23, 2), msg.Substring(25, 7), msg.Substring(33, 1));
                    longitud = DDMToDD(msg.Substring(35, 3), msg.Substring(38, 7), msg.Substring(46, 1));

                    //Time to impact, seconds
                    double timeToImpact = double.Parse(msg.Substring(48, 7), CultureInfo.InvariantCulture);

                    GPS_IIP IIP = new GPS_IIP(longitud, latitude, time, msg, timeToImpact);
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

                    //Do something fancy with the quaternions
                    Attitude at = new Attitude(floatConversion(bquaternion0), floatConversion(bquaternion1), floatConversion(bquaternion2), floatConversion(bquaternion3));
                    

                    NewAttitudeReceived.Add(at);
                    AttitudeTimes.Add(new JulianDate(DateTime.Now));
                }
                else if (Packetmatch(RAMSES_buffer, PacketID_PCDU)) //If it is a PCDU packet
                {
                    var ACCx = (short)(RAMSES_buffer[32 + 105 + 0] << 8 | RAMSES_buffer[32 + 105 + 1]);
                    var ACCy = (short)(RAMSES_buffer[32 + 107 + 0] << 8 | RAMSES_buffer[32 + 107 + 1]);
                    var ACCz = (short)(RAMSES_buffer[32 + 109 + 0] << 8 | RAMSES_buffer[32 + 109 + 1]);

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

                    GLoadsReceived.Add(new GLoads(ACCx, ACCy, ACCz)); //What are the units?
                    gLoadsAvailable = true;
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
                listBox1.Items.Add("error!");
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

                listBox1.Items.Add(GPSpositionsReceived[GPSpositionsReceived.Count - 1].RAWmessage);
            }

            //Attitude messages
            if (AttitudeReceived.Count > 0)
            {
                label6.Text = "GCS: " + AttitudeReceived.Count.ToString();
                label7.Text = "qt: " + AttitudeReceived[AttitudeReceived.Count - 1].ToString();

                listBox1.Items.Add(AttitudeReceived[AttitudeReceived.Count - 1].ToString());
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
            BindUPD();

            // Initialize the data push:
            pushInitialPacket();


            rand = new Random();
            System.Timers.Timer testTimer = new System.Timers.Timer();
            testTimer.Elapsed += new System.Timers.ElapsedEventHandler(onTestEvent);
            testTimer.Interval = 100;
            testTimer.Enabled = true;
            testTimer.AutoReset = true;
            testTimer.Start();

            pushTimer = new System.Timers.Timer();
            pushTimer.Elapsed += new System.Timers.ElapsedEventHandler(onPushEvent);
            pushTimer.Interval = 2000;
            pushTimer.Enabled = true;
            pushTimer.AutoReset = true;
            pushTimer.Start();
        }

        /* onTestEvent is used to test the functionaliny of the pusher without having to use real network data. */
        private void onTestEvent(object sender, System.Timers.ElapsedEventArgs e) {
            bool testGPS = true;
            bool testAttitude = true;
            bool testGLoads = true;
            bool testAngularRates = false;

            if (linecounter > 9)
            {
                var line = CSVreader.ReadLine();
                var values = line.Split(';');
                
                //time = Convert.ToDouble(values[0]);
                //speed = Convert.ToDouble(values[4]);
                testLong = Convert.ToDouble(values[16]);
                testLat = Convert.ToDouble(values[17]);
                testAlt = Convert.ToDouble(values[39])*1000;
                //yaw = Convert.ToDouble(values[46]);
                //pitch = Convert.ToDouble(values[45]);
                //roll = Convert.ToDouble(values[44]);
                if (testGPS)
                {
                    DateTime timeStamp = DateTime.Now;

                    //testLong += rand.NextDouble() * 0.00001;
                    //testLat += rand.NextDouble() * 0.00001;
                    //testAlt += rand.NextDouble() * 5;
                    GPSposition p = new GPSposition(testLong, testLat, testAlt, new JulianDate(timeStamp), "msg", 11);
                    //GPSpositionsReceived.Add(p);

                    NewGPSpositionsReceived.Add(p);

                    double secondsSince = linecounter * 0.1;

                    double minutes = Math.Floor(secondsSince / 60);
                    double seconds = Math.Floor(secondsSince % 60);
                    string secondsString;
                    string minutesString;

                    if (seconds < 10)
                    {
                        secondsString = "0" + seconds.ToString();
                    }
                    else {
                        secondsString = seconds.ToString();
                    }

                    if (minutes < 10)
                    {
                        minutesString = "0" + minutes.ToString();
                    }
                    else
                    {
                        minutesString = minutes.ToString();
                    }

                    RTtimestr = "+00:"+minutesString+":"+secondsString;
                }

                if (testGLoads)
                {
                    GLoads gLoads = new GLoads(float.Parse(values[41]), float.Parse(values[42]), float.Parse(values[43]));
                    GLoadsReceived.Add(gLoads);
                }

                if (testAngularRates) {
                    testARates = new AngularRates((float)rand.NextDouble()*100, (float)rand.NextDouble() * 100, (float)rand.NextDouble() * 100);
                    ARatesReceived.Add(testARates);
                }

                linecounter++;
                //NewDataFlag = true;
            }
            else {
                var line = CSVreader.ReadLine();
                var values = line.Split(';');
                linecounter++;
            }
        }

        /* Pushing the initial packet to the server. */
        private void pushInitialPacket()
        {
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

            // Writing the response to the console
            //Console.WriteLine(responseStatus.ToString());

            // Closing the string writer
            sw.Close();
        }

        

        /* onPushedEvent handles the pushing of data to the server. Pushes should not be more frequent than 2 Hz. */
        private void onPushEvent(object sender, System.Timers.ElapsedEventArgs e)
        {
            listBox1.Items.Add("A new data push was initialized");
            string expectedAttitudeFormat = "quaternion"; // Maybe add this to the user interface?

            if (NewGPSpositionsReceived.Count > 0)
            {
                GPSUpdated = true;
            }
            else {
                GPSUpdated = false;
            }

            if (NewAttitudeReceived.Count > 0)
            {
                attitudeUpdated = true;
            }
            else
            {
                attitudeUpdated = false;
            }

            if (GLoadsReceived.Count > 0)
            {
                gLoadsAvailable = true;
            }
            else
            {
                gLoadsAvailable = false;
            }



            if (GPSUpdated || attitudeUpdated)
            {
                var watch = System.Diagnostics.Stopwatch.StartNew();
                // the code that you want to measure comes here
                
                try
                {

                    string missionTime = NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Time.ToString();
                    double missionSeconds = timeString2seconds(RTtimestr);

                    if (plusminusTime == "-" && RTtimestr.Substring(0, 1) == "+") {
                        launchTime = NewGPSpositionsReceived[0].Time;
                        plusminusTime = "+";
                    }

                    //writer = new CesiumStreamWriter();
                    System.Threading.Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
                    // Dates and positions lists, in case we want to sample more data before writing to server.
                    List<JulianDate> dates = new List<JulianDate>();
                    List<Cartographic> positions = new List<Cartographic>();
                    List<UnitQuaternion> orientations = new List<UnitQuaternion>();
                    List<double> speeds = new List<double>();
                    double speed = 0;
                    Cartesian gLoads = new Cartesian();
                    Cartesian aRates = new Cartesian();

                    // Resetting the stringwriter
                    sw = new StringWriter();

                    // Resetting the output
                    output = new CesiumOutputStream(sw);

                    // Initializing a new json array
                    output.WriteStartSequence();

                    // Open a new packet for the rocket model
                    packet = writer.OpenPacket(output);
                    packet.WriteId("rocket");

                    // Attach a path property to the rocket, enabling us to see the flight trail
                    path = packet.OpenPathProperty();
                    var pathMaterial = path.OpenMaterialProperty();
                    var polyLineMat = pathMaterial.OpenSolidColorProperty();
                    polyLineMat.WriteColorProperty(Color.Red);
                    polyLineMat.Close();
                    pathMaterial.Close();
                    path.WriteWidthProperty(1);
                    path.WriteLeadTimeProperty(0);
                    path.WriteTrailTimeProperty(3600);
                    path.WriteResolutionProperty(0.5);
                    path.Close();


                    // Open the positions packet
                    position = packet.OpenPositionProperty();
                    listBox1.Items.Add("Writing position property");

                    // The first position packet states the inter- and extrapolating properties [Do we need this?]
                    if (firstPositionPacket)
                    {
                        position.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Linear);
                        position.WriteInterpolationDegree(1);
                        position.WriteForwardExtrapolationDuration(extrapolationDuration);
                        position.WriteBackwardExtrapolationDuration(extrapolationDuration);
                        position.WriteForwardExtrapolationType(CesiumExtrapolationType.Extrapolate);
                        firstPositionPacket = false;
                    }
                    if (GPSUpdated)
                    {
                        // INCLUDE A TIMER HERE TO KEEP TRACK ON THE TIME BETWEEN NEW DATA POINTS

                        foreach (GPSposition tempPosition in NewGPSpositionsReceived)
                        {
                            positions.Add(new Cartographic(tempPosition.Longitude, tempPosition.Latitude, tempPosition.Altitude));
                            PositionTimes.Add(tempPosition.Time);
                        }
                        listBox1.Items.Add("Pushing new GPS data");


                        if (NewGPSpositionsReceived.Count > 1)
                        {
                            Vector<double> firstSamplePos = GPS2cartesian(NewGPSpositionsReceived[0].Longitude, NewGPSpositionsReceived[0].Latitude, NewGPSpositionsReceived[0].Altitude);
                            Vector<double> lastSamplePos = GPS2cartesian(NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Longitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Latitude, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Altitude);
                            speed = positions2speed(firstSamplePos, lastSamplePos, NewGPSpositionsReceived[0].Time, NewGPSpositionsReceived[NewGPSpositionsReceived.Count - 1].Time);

                            speedUpdated = true;

                        }
                        else
                        {
                            if (GPSpositionsReceived.Count > 0)
                            {
                                Vector<double> secondPos = GPS2cartesian(NewGPSpositionsReceived[0].Longitude, NewGPSpositionsReceived[0].Latitude, NewGPSpositionsReceived[0].Altitude);
                                Vector<double> firstSamplePos = GPS2cartesian(GPSpositionsReceived[GPSpositionsReceived.Count - 1].Longitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Latitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Altitude);
                                speed = positions2speed(firstSamplePos, secondPos, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Time, NewGPSpositionsReceived[0].Time);

                                speedUpdated = true;
                                listBox1.Items.Add("Speed2: " + speed.ToString());
                            }
                            else
                            {
                                speedUpdated = false;
                            }
                        }

                    }
                    else
                    {
                        // If no new GPS data is received but we have some old data, assume that data stream has initiated, but that it might have been interupted.
                        if (GPSpositionsReceived.Count > 0)
                        {
                            // Re-send the last known position if data is missing
                            GPSposition tempPosition = GPSpositionsReceived[GPSpositionsReceived.Count - 1];
                            positions.Add(new Cartographic(tempPosition.Longitude, tempPosition.Latitude, tempPosition.Altitude));
                            PositionTimes.Add(new JulianDate(DateTime.Now));
                            listBox1.Items.Add("Pushing old GPS data");
                        }
                    }

                    listBox1.Items.Add(positions.Count.ToString() + "-----" + PositionTimes.Count.ToString());
                    try
                    {
                        position.WriteCartographicDegrees(PositionTimes, positions);
                    }
                    catch (ArgumentException argE)
                    {
                        if (PositionTimes.Count > positions.Count)
                        {
                            PositionTimes = PositionTimes.GetRange(0, positions.Count);
                        }
                        else if (PositionTimes.Count < positions.Count)
                        {
                            positions = positions.GetRange(0, PositionTimes.Count);
                        }
                        else
                        {
                            listBox1.Items.Add(argE.StackTrace.ToString());
                        }
                        listBox1.Items.Add("Exception! " + positions.Count.ToString() + "------" + PositionTimes.Count.ToString());
                    }
                    position.Close();

                    // Open the orientation packet
                    orientation = packet.OpenOrientationProperty();
                    listBox1.Items.Add("Writing orientation property");

                    // The first orientation packet states the inter- and extrapolating properties
                    if (firstOrientationPacket)
                    {
                        orientation.WriteInterpolationAlgorithm(CesiumInterpolationAlgorithm.Lagrange);
                        orientation.WriteInterpolationDegree(1);
                        orientation.WriteForwardExtrapolationDuration(extrapolationDuration);
                        orientation.WriteForwardExtrapolationType(CesiumExtrapolationType.Extrapolate);
                        firstOrientationPacket = false;
                    }

                    if (attitudeUpdated)
                    {
                        // INCLUDE A TIMER HERE TO KEEP TRACK ON THE TIME BETWEEN NEW DATA POINTS
                        foreach (Attitude tempAttitude in NewAttitudeReceived)
                        {
                            if (expectedAttitudeFormat == "quaternion")
                            {
                                orientations.Add(TH2ECEF(tempAttitude.q0, tempAttitude.q1, tempAttitude.q2, tempAttitude.q3));
                            }
                        }
                        listBox1.Items.Add("Pushing new attitude data");
                    }
                    else
                    {
                        // If no new attitude data is received but we have some old data, assume that attitude data should be available, but that the stream might have been interupted.
                        if (AttitudeReceived.Count > 0)
                        {
                            Attitude tempAttitude = AttitudeReceived[AttitudeReceived.Count - 1];
                            if (expectedAttitudeFormat == "quaternion")
                            {
                                orientations.Add(TH2ECEF(tempAttitude.q0, tempAttitude.q1, tempAttitude.q2, tempAttitude.q3));
                            }
                        }
                        // Otherwise, assume that attitude data is NOT available, and make sure that the attitude points in the direction of flight.
                        else
                        {
                            if (NewGPSpositionsReceived.Count > 0 && GPSpositionsReceived.Count > 0)
                            {
                                orientations.AddRange(attitudeFromGPS(NewGPSpositionsReceived));
                                AttitudeTimes = PositionTimes;
                                listBox1.Items.Add("Number of new GPS positions: " + NewGPSpositionsReceived.Count.ToString());
                                listBox1.Items.Add("Total number of GPS positions: " + GPSpositionsReceived.Count.ToString());
                                listBox1.Items.Add("Number of new Attitude positions: " + orientations.Count.ToString());
                            }
                            else
                            {
                                orientations.Add(new UnitQuaternion(1, 0, 0, 0));
                                AttitudeTimes.Add(new JulianDate(DateTime.Now));
                            }
                        }
                    }

                    try
                    {
                        listBox1.Items.Add("The orientations: " + orientations[0].ToString());
                        orientation.WriteUnitQuaternion(AttitudeTimes, orientations);
                    }
                    catch (Exception argE)
                    {
                        if (AttitudeTimes.Count > orientations.Count)
                        {
                            AttitudeTimes = AttitudeTimes.GetRange(0, orientations.Count);
                            orientation.WriteUnitQuaternion(AttitudeTimes, orientations);
                        }
                        else if (AttitudeTimes.Count < orientations.Count)
                        {
                            orientations = orientations.GetRange(0, AttitudeTimes.Count);
                            orientation.WriteUnitQuaternion(AttitudeTimes, orientations);
                        }
                        else
                        {
                            listBox1.Items.Add(argE.StackTrace.ToString());
                        }
                    }

                    orientation.Close();

                    // Opening the model packet
                    model = packet.OpenModelProperty();
                    listBox1.Items.Add("Writing model property");
                    model.WriteGltfProperty(modelURL, CesiumResourceBehavior.LinkTo);
                    model.WriteMinimumPixelSizeProperty(128);

                    // Correcting the scale
                    model.WriteScaleProperty(0.001);
                    model.WriteMaximumScaleProperty(1);
                    model.Close();

                    // Close packet when everything is appended
                    packet.Close();

                    // Open the packet used for the speed, g-forces, adn possibly events.
                    packet = writer.OpenPacket(output);
                    packet.WriteId("speed_gLoads");
                    packet.WriteName("event placeholder");

                    // Opening the point packet (this is an abstract point, it will not be visible)
                    point = packet.OpenPointProperty();
                    point.WriteShowProperty(false);
                    // pixelSize will correspond to the speed

                    // Only update the speed if it has a valid update from the positions
                    if (speedUpdated)
                    {
                        point.WritePixelSizeProperty(speed);
                    }

                    //// outlineWidth will correspond to the time (for now)
                    //point.WriteOutlineWidthProperty(missionSeconds);
                    // The position of the point will correspont to the 3 component g-force
                    if (GLoadsReceived.Count > 0)
                    {
                        gLoadPacket = packet.OpenPositionProperty();
                        gLoads = GLoadsReceived[GLoadsReceived.Count - 1].toCartesian();
                        gLoadPacket.WriteCartesian(gLoads);
                        gLoadPacket.Close();
                    }
                    else
                    {
                        gLoads = new Cartesian(0, 0, 0);
                    }

                    

                    // Closing the point packet
                    point.Close();

                    // Close packet when everything is appended
                    packet.Close();

                    // Open the packet used for the angular rates and time.
                    packet = writer.OpenPacket(output);
                    packet.WriteId("angularRates_time");
                    packet.WriteName(RTtimestr);

                    // Opening the point packet (this is an abstract point, it will not be visible)
                    point = packet.OpenPointProperty();
                    point.WriteShowProperty(false);

                    try
                    {
                        listBox1.Items.Add(launchTime);
                        point.WritePixelSizeProperty(launchTime.SecondsDifference(NewGPSpositionsReceived[0].Time));
                    }
                    catch (Exception except){
                        listBox1.Items.Add(except.StackTrace.ToString());
                    }

                    if (ARatesReceived.Count > 0)
                    {
                        aRatePacket = packet.OpenPositionProperty();
                        aRates = ARatesReceived[ARatesReceived.Count - 1].toCartesian();
                        aRatePacket.WriteCartesian(aRates);
                        aRatePacket.Close();
                    }

                    // Closing the point packet
                    point.Close();

                    // Close packet when everything is appended
                    packet.Close();


                    if (GPSUpdated)
                    {
                        // Opening the last packet, which will be used to store the previous positions, used if a client connects mid-flight
                        packet = writer.OpenPacket(output);
                        listBox1.Items.Add("Writing flight path");
                        packet.WriteId("flightPath");
                        polyLine = packet.OpenPolylineProperty();
                        var material = polyLine.OpenMaterialProperty();
                        var pathColor = material.OpenSolidColorProperty();
                        pathColor.WriteColorProperty(Color.Red);
                        pathColor.Close();
                        material.Close();
                        polyLine.WriteFollowSurfaceProperty(false);
                        polyLine.WriteWidthProperty(1);
                        polyLine.WritePositionsPropertyCartographicDegrees(positions);
                        polyLine.Close();

                        packet.Close();
                    }

                    // End sequence to close the json array
                    output.WriteEndSequence();

                    // Create POST data and convert it to a byte array.
                    byteArray = Encoding.UTF8.GetBytes(sw.ToString());

                    listBox1.Items.Add(sw.ToString());

                    // Close the string writer
                    sw.Close();

                    listBox1.Items.Add("Pushing...");
                    var responseStatus = postData(byteArray);

                    GPSpositionsReceived.AddRange(NewGPSpositionsReceived);
                    AttitudeReceived.AddRange(NewAttitudeReceived);

                    // Resetting some variables
                    NewGPSpositionsReceived = new List<GPSposition>();
                    NewAttitudeReceived = new List<Attitude>();
                    PositionTimes = new List<JulianDate>();
                    AttitudeTimes = new List<JulianDate>();
                }
                catch (Exception except) {
                    listBox1.Items.Add(except.StackTrace.ToString());
                }
                watch.Stop();
                listBox1.Items.Add("Time of execution: " + watch.ElapsedMilliseconds.ToString());
            }
            else
            {
                // If we are not receiving any data, just ping the server to ensure that we have a connection

                pingServer();
            }
        }



        /* When no packets are received, just ping the server to make sure that the connection is maintained. */
        private void pingServer()
        {
            // Resetting the stringwriter
            sw = new StringWriter();

            // Resetting the output
            output = new CesiumOutputStream(sw);

            // First bracket in the array
            output.WriteStartSequence();

            // Add a new packet
            packet = writer.OpenPacket(output);

            // Writing the document package, which is the first package that the client must receive
            packet.WriteId("document");
            packet.WriteName("Ping");
            packet.WriteVersion("1.0");

            // Close first package
            packet.Close();

            // Closing bracket
            output.WriteEndSequence();

            //Populating the byte array with the data written
            byteArray = Encoding.UTF8.GetBytes(sw.ToString());

            // Posting the data to the server
            var responseStatus = postData(byteArray);

            sw.Close();
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
        private Vector<double> GPS2cartesian(double longitude, double latitude, double height) {
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
            catch (Exception except) {
                listBox1.Items.Add(except.StackTrace.ToString());
            }

            return cartesianPos;
        } 

        private double positions2speed(Vector<double> firstPos, Vector<double> secondPos, JulianDate firstTime, JulianDate secondTime) {
            Vector<double> velocity = Vector<double>.Build.DenseOfArray(new double[3] {0,0,0});
            try
            {
                Vector<double> posDiff = secondPos - firstPos;
                if (Math.Abs(posDiff.L2Norm()) < 0.001)
                {
                    return 0;
                }
                double timeDiff = secondTime.SecondsDifference(firstTime);

                // Don't wanna divide by zero
                if (Math.Abs(timeDiff) < 0.0000001) {
                    return 0;
                }
                // Calculating the velocity
                velocity = posDiff.Divide(timeDiff);
            }
            catch (Exception except)
            {
                listBox1.Items.Add(except.StackTrace.ToString());
            }
            // Returning the speed, which is the L2 norm of the velocity
            return velocity.L2Norm();
        }

        // Finds an orthogonal vector
        Vector<double> orthogonal(Vector<double> v)
        {
            double x = Math.Abs(v[0]);
            double y = Math.Abs(v[1]);
            double z = Math.Abs(v[2]);

            Vector<double> other = x < y ? (x < z ? xAxis : zAxis) : (y < z ? yAxis : zAxis);
            return crossProduct(v, other);
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
        private List<UnitQuaternion> attitudeFromGPS(List<GPSposition> GPSposList)
        {
            Vector<double> currentPosition = Vector<double>.Build.Dense(3);
            Vector<double> previousPosition;

            List<UnitQuaternion> resultsList = new List<UnitQuaternion>();

            try
            {

                Vector<double> xVector = Vector<double>.Build.Dense(3);
                xVector[0] = 1;
                xVector[1] = 0;
                xVector[2] = 0;

                Vector<double> quat = Vector<double>.Build.Dense(4);

                for (int i = 0; i < GPSposList.Count; i++)
                {
                    Vector<double> tempQuat = Vector<double>.Build.Dense(4);
                    if (i == 0)
                    {
                        previousPosition = GPS2cartesian(GPSpositionsReceived[GPSpositionsReceived.Count - 1].Longitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Latitude, GPSpositionsReceived[GPSpositionsReceived.Count - 1].Altitude);
                    }
                    else
                    {
                        previousPosition = currentPosition;
                    }
                    currentPosition = GPS2cartesian(GPSposList[i].Longitude, GPSposList[i].Latitude, GPSposList[i].Altitude);

                    // If the speed is almost zero, do not update the attitude
                    if ((currentPosition - previousPosition).L2Norm() < 0.000001)
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
                    listBox1.Items.Add("Adding quat");
                    resultsList.Add(new UnitQuaternion(tempQuat[0], tempQuat[1], tempQuat[2], tempQuat[3]));

                }
            }
            catch (Exception except) {
                listBox1.Items.Add(except.StackTrace.ToString());
            }

            return resultsList;
        }

        /* postData is handles the HTTP request. */
        public static async Task<bool> postData(byte[] data)
        {
            // Convert data to a type that httpClient can handle
            ByteArrayContent byteContent = new ByteArrayContent(data);
            byteContent.Headers.ContentType = new MediaTypeHeaderValue("application/json");
            try
            {
                HttpResponseMessage response = await client.PostAsync(serverPath, byteContent);
                Console.WriteLine(response.Content.ReadAsStringAsync().ToString());
                // Return the URI of the created resource.
                return response.IsSuccessStatusCode;
            }
            catch (Exception e)
            {
                Console.WriteLine("postData failed to send data to the server");
                return false;
            }
        }

        private void listBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}
