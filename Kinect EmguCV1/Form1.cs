using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Kinect;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.Util;

using System.Net;
using System.Net.Sockets;


namespace Kinect_EmguCV1
{
    public struct Point3D
    {
        private double x;
        private double y;
        private double z;
        public double X //X坐标
        {
            get { return (this.x); }
            set { x = value; }
        }
        public double Y //X坐标
        {
            get { return (this.y); }
            set { y = value; }
        }
        public double Z //X坐标
        {
            get { return (this.z); }
            set { z = value; }
        }
        public override string ToString()
        {
            return this.x + "," + this.y + "," + this.z;//输出x,y,z的形式表示一个三维点
        }       
    }
    
    public partial class Form1 : Form
    {
        //Socket
        //const int BufferSize = 8192;//缓存大小，8192字节，可以保存4096个汉字和英文字符
        //IPAddress ip = IPAddress.Parse("127.0.0.1");//获取ip地址
        //TcpListener listener = null;
        //private IPAddress serverIP = IPAddress.Parse("127.0.0.1");
        //private IPEndPoint serverFullAddr;
        //private Socket sock;
        private Socket sSocket;        
        private Socket serverSocket;
        private string recStr = "";


        private const float HandSize = 5;//绘制手圈时的半径
        private const double JointThickness = 3;//接缝厚度        
        private const double ClipBoundsThickness = 10;//夹边矩形的厚度        
        private const float InferredZPositionClamp = 0.1f;//常数为夹紧Z值的相机空间点负面

        private FrameDescription frameDescription = null;
        private KinectSensor kinectSensor = null;
        private MultiSourceFrameReader multiSourceFrameReader = null;
        private Point3D[] p3dTeam=null;//各关节坐标
        private int length = 0;//关节个数
        private double[] angle = new double[6];
        private string angleStr = "";

        private Body[] _body = null;//身体数组
        byte[] _bodyIndex = null;//BodyIndex数组
        private int displayWidth;//展示的宽度（纵深空间）
        private int displayHeight;//展示的高度（纵深空间）
        private CoordinateMapper coordinateMapper = null;
        Image<Bgra, byte> imageSourse;
        private double rad = 360 / Math.PI / 2;

        public Form1()
        {
            InitializeComponent();
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;//获得Kinect的坐标映射器
            frameDescription = kinectSensor.BodyIndexFrameSource.FrameDescription;//BodyIndexFrameSource的帧描述

            multiSourceFrameReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
            multiSourceFrameReader.MultiSourceFrameArrived += Framereader_MultiSourceFrameArrived;

            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            _bodyIndex = new byte[frameDescription.LengthInPixels * 4];
            imageSourse = new Image<Bgra, byte>(displayWidth, displayHeight);
            kinectSensor.Open();
            SocketInit();//Socket初始化
        }

        private JointType[] _JointType = new JointType[]
        {
            JointType.HandLeft,JointType.SpineShoulder,JointType.ShoulderRight,JointType.ElbowRight,JointType.WristRight,
            JointType.HandRight,JointType.HandTipRight,JointType.ThumbRight,JointType.ShoulderLeft,JointType.HandTipRight
        };


        private unsafe void ProcessBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize)
        {
            byte* frameData = (byte*)bodyIndexFrameData;
            for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            {
                if (frameData[i] < 6)
                {
                    this._bodyIndex[i * 4] = 0;
                    this._bodyIndex[i * 4 + 1] = 0;
                    this._bodyIndex[i * 4 + 2] = 150;
                    this._bodyIndex[i * 4 + 3] = 255;     //人物显示为红色  
                }
                else
                {
                    this._bodyIndex[i * 4] = 0;
                    this._bodyIndex[i * 4 + 1] = 0;
                    this._bodyIndex[i * 4 + 2] = 0;
                    this._bodyIndex[i * 4 + 3] = 255;     //背景显示为黑色  
                }
            }
        }

        private void Framereader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();
            //骨骼帧和彩色帧都不为空  
            using (var bodyIndexFrame = reference.BodyIndexFrameReference.AcquireFrame())
            {
                using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
                {
                    if (bodyIndexFrame != null)
                    {
                        using (KinectBuffer kB = bodyIndexFrame.LockImageBuffer())
                        {
                            if (frameDescription.Width * frameDescription.Height == kB.Size)
                                this.ProcessBodyIndexFrameData(kB.UnderlyingBuffer, kB.Size);
                            imageSourse.Bytes = _bodyIndex;
                            imageBox1.Image = imageSourse;
                        }
                    }
                    if (bodyFrame != null)
                    {
                        _body = new Body[bodyFrame.BodyCount];
                        bodyFrame.GetAndRefreshBodyData(this._body);
                        DrawBodies();
                        angle = Angle();
                        angleStr = "[" + angle[0].ToString() + "," + angle[1].ToString() + "," + angle[2].ToString() + ","
                                + angle[3].ToString() + "," + angle[4].ToString() + "," + angle[5].ToString() + "]";
                        if (p3dTeam != null)
                        {
                            SocketSend();
                            //Console.WriteLine("ang1= "+angle[0].ToString()+ ", ang2= " + angle[1].ToString() + ", ang3= " + angle[2].ToString());
                            //Console.WriteLine("ang4= " + angle[3].ToString() + ", ang5= " + angle[4].ToString() + ", ang6= " + angle[5].ToString());
                            //Console.WriteLine("");
                            Console.WriteLine(angleStr);
                        }
                    }
                }
            }
        }

        private void DrawBodies()
        {
            for (int i = 0; i < 6; i++)
            {
                if (this._body[i].IsTracked == true)
                {
                    Body oneBody = this._body[i];
                    IReadOnlyDictionary<JointType, Joint> joints = oneBody.Joints;
                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                    foreach (JointType jointType in joints.Keys)
                    {   
                        // 有时推断的关节点的深度(Z)可能显示为负
                        //夹到0.1f,以防止coordinatemapper返回(无限大,无限)
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }
                    }
                    CameraSpacePoint handLeft = joints[JointType.HandLeft].Position;
                    CameraSpacePoint spineShoulder = joints[JointType.SpineShoulder].Position;
                    CameraSpacePoint shoulderRight = joints[JointType.ShoulderRight].Position;
                    CameraSpacePoint elbowRight = joints[JointType.ElbowRight].Position;
                    CameraSpacePoint wristRight = joints[JointType.WristRight].Position;
                    CameraSpacePoint handRight = joints[JointType.HandRight].Position;
                    CameraSpacePoint thumbRight = joints[JointType.ThumbRight].Position;
                    CameraSpacePoint handTipRight = joints[JointType.HandTipRight].Position;
                    CameraSpacePoint shoulderLeft = joints[JointType.ShoulderLeft].Position;
                    

                    CameraSpacePoint[] csp = { shoulderLeft, shoulderRight, elbowRight, wristRight, handTipRight };
                    length = csp.Length;//关节个数
                    DepthSpacePoint[] dsp = new DepthSpacePoint[length];
                    Point[] pointTeam = new Point[length];
                    System.Drawing.PointF[] pointTeamF = new System.Drawing.PointF[length];
                    CircleF[] circleTeam = new CircleF[length];
                    p3dTeam = new Point3D[length];//存储所用3d点

                    for (int j = 0; j < length; j++)
                    {
                        dsp[j] = coordinateMapper.MapCameraPointToDepthSpace(csp[j]);
                        pointTeam[j].X = (int)dsp[j].X;
                        pointTeam[j].Y = (int)dsp[j].Y; 

                        p3dTeam[j].X = csp[j].X;
                        p3dTeam[j].Y = csp[j].Y;
                        p3dTeam[j].Z = csp[j].Z;

                        pointTeamF[j] = pointTeam[j];
                        circleTeam[j] = new CircleF(pointTeamF[j],HandSize);
                        imageSourse.Draw(circleTeam[j], new Bgra(255, 255, 255, 255), 0);
                        //Console.WriteLine(p3dTeam[j].ToString());
                    }
                    for (int j = 0; j < (length - 1); j++)
                        CvInvoke.Line(imageSourse, pointTeam[j], pointTeam[j+1], new MCvScalar(0, 0, 255), 2);
                   
                    imageBox1.Image = imageSourse;
                }
            }
        }

        private double[] Angle()
        {
            double[] temp = { 0, 0, 0, 0, 0, 0 };
            if (p3dTeam != null)
            {
                temp[0] = vectorAngleXZ(p3dTeam[0], p3dTeam[1])*rad;//底座角度
                if ((p3dTeam[1].X - p3dTeam[2].X) != 0)
                    temp[1] = Math.Abs(vectorAngleXY(p3dTeam[1], p3dTeam[2])) * rad;//肩膀
                if ((p3dTeam[2].X - p3dTeam[3].X) != 0)
                    temp[2] = Math.Abs(vectorAngleXY(p3dTeam[2], p3dTeam[3])) * rad;//手肘
                if ((p3dTeam[3].X - p3dTeam[4].X) != 0)
                    temp[3] = Math.Abs(vectorAngleXY(p3dTeam[3], p3dTeam[4])) * rad;//手腕俯仰
                temp[4] = vectorAngleXZ(p3dTeam[3], p3dTeam[4]) * rad;//手腕旋转
                temp[5] = 0;
            }
            return temp;
        }

        public double distanceXZ(Point3D a, Point3D b)
        {
            double temp = 0;
            temp = (a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z);
            temp = Math.Sqrt(temp);
            return temp;
        }

        public double distanceXY(Point3D a, Point3D b)
        {
            double temp = 0;
            temp = (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y);
            temp = System.Math.Sqrt(temp);
            return temp;
        }

        public double distanceYZ(Point3D a, Point3D b)
        {
            double temp = 0;
            temp = (a.Z - b.Z) * (a.Z - b.Z) + (a.Y - b.Y) * (a.Y - b.Y);
            temp = System.Math.Sqrt(temp);
            return temp;
        }

        public double vectorAngleXZ(Point3D a, Point3D b)
        {
            double temp = 0;
            //temp = Math.Acos((a.X -b.X) / distanceXZ(a, b));
            temp = (b.Z - a.Z) / (b.X - a.X);
            temp = Math.Atan(temp);
            return temp;
        }

        public double vectorAngleYZ(Point3D a, Point3D b)
        {
            double temp = 0;
            //temp = Math.Acos((a.Y - b.Y) / distanceYZ(a, b));
            temp = (b.Z - a.Z) / (b.Y - a.Y);
            temp = Math.Atan(temp);
            return temp;
        }

        public double vectorAngleXY(Point3D a, Point3D b)
        {
            double temp = 0;
            //temp = Math.Acos((a.X - b.X) / distanceXY(a, b));
            temp = (b.Y - a.Y) / (b.X - a.X);
            temp = Math.Atan(temp);
            return temp;
        }

        /*
        public void SocketInit()
        {
            try
            {
                serverFullAddr = new IPEndPoint(serverIP, 8500);//设置IP，端口  
                sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                //指定本地主机地址和端口号  
                sock.Connect(serverFullAddr);
                Console.WriteLine("连接服务器成功。。。。");
            }

            catch
            {
                Console.WriteLine("连接服务器失败。。。请仔细检查服务器是否开启");
            }
                   
            string msg = "movej(" + angleStr + ",a=3,v=0.75)/n";
            byte[] byteSend = System.Text.Encoding.Default.GetBytes(msg);
            byte[] message = new byte[1024];
            string mess = "";
            int bytes = 0;
            try
            {                
                //发送数据  
                sock.Send(byteSend);
                bytes = sock.Receive(message);//接收数据  
                mess = mess + Encoding.Default.GetString(message, 0, bytes);
                //编码（当接收的字节大于1024的时候 这应该是循环接收，测试就没有那样写了）                  
                Console.WriteLine(mess);
            }
            catch (Exception ex)
            {
                Console.WriteLine("出现错误，请联系管理员");
            }            
        }
            */

        private void SocketInit()
        {
            int port = 30002;
            string host = "127.0.0.1";

            IPAddress ip = IPAddress.Parse(host);
            IPEndPoint ipe = new IPEndPoint(ip, port);

            sSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            sSocket.Bind(ipe);
            sSocket.Listen(0);
            Console.WriteLine("监听已经打开，请等待");

            //receive message
            serverSocket = sSocket.Accept();
            Console.WriteLine("连接已经建立");    
                    
        }

        private void SocketSend()
        {
            if (this.serverSocket != null && sSocket != null)
            {
                string recStr = "";
                byte[] recByte = new byte[4096];
                int bytes = 0;
                try
                {
                    bytes = serverSocket.Receive(recByte, recByte.Length, 0);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("出现错误，连接已断开！出现错误：" + ex.ToString());
                    multiSourceFrameReader.MultiSourceFrameArrived -= Framereader_MultiSourceFrameArrived;
                }
                recStr += Encoding.ASCII.GetString(recByte, 0, bytes);
                //send message                
                Console.WriteLine("服务器端获得信息:{0}", recStr);
                string sendStr = "movej(" + angleStr + ",a=3,v=0.75)/n";
                byte[] sendByte = Encoding.ASCII.GetBytes(sendStr);
                try
                {
                    serverSocket.Send(sendByte, sendByte.Length, 0);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("出现错误，连接已断开！出现错误：" + ex.ToString());
                    multiSourceFrameReader.MultiSourceFrameArrived -= Framereader_MultiSourceFrameArrived;
                }
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            if (this.serverSocket != null && sSocket != null) 
            {
                serverSocket.Close();
                sSocket.Close();
            }
        }       
    }

}

