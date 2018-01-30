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


namespace Kinect_EmguCV1
{
    public partial class Form1 : Form
    {

        private const float HandSize = 5;//绘制手圈时的半径
        private const double JointThickness = 3;//接缝厚度        
        private const double ClipBoundsThickness = 10;//夹边矩形的厚度        
        private const float InferredZPositionClamp = 0.1f;//常数为夹紧Z值的相机空间点负面

        private FrameDescription frameDescription = null;
        private KinectSensor kinectSensor = null;
        private MultiSourceFrameReader multiSourceFrameReader = null;
        private Body[] _body = null;//身体数组
        byte[] _bodyIndex = null;//BodyIndex数组
        private int displayWidth;//展示的宽度（纵深空间）
        private int displayHeight;//展示的高度（纵深空间）
        private CoordinateMapper coordinateMapper = null;
        Image<Bgra, byte> imageSourse;


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

        }

        private JointType[] _JointType = new JointType[]
        {
            JointType.HandLeft,JointType.ShoulderRight,JointType.ElbowRight,JointType.WristRight,
            JointType.HandRight,JointType.HandTipRight,JointType.ThumbRight
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
                    CameraSpacePoint shoulderRight = joints[JointType.ShoulderRight].Position;
                    CameraSpacePoint elbowRight = joints[JointType.ElbowRight].Position;
                    CameraSpacePoint wristRight = joints[JointType.WristRight].Position;
                    CameraSpacePoint handRight = joints[JointType.HandRight].Position;
                    CameraSpacePoint thumbRight = joints[JointType.ThumbRight].Position;
                    CameraSpacePoint handTipRight = joints[JointType.HandTipRight].Position;

                    CameraSpacePoint[] csp = { shoulderRight, elbowRight, wristRight, handRight };
                    DepthSpacePoint[] dsp = new DepthSpacePoint[csp.Length];
                    Point[] pointTeam = new Point[csp.Length];
                    System.Drawing.PointF[] pointTeamF = new System.Drawing.PointF[csp.Length];
                    CircleF[] circleTeam = new CircleF[csp.Length];

                    for (int j = 0; j < csp.Length; j++)
                    {
                        dsp[j] = coordinateMapper.MapCameraPointToDepthSpace(csp[j]);
                        pointTeam[j].X = (int)dsp[j].X;
                        pointTeam[j].Y = (int)dsp[j].Y;
                        pointTeamF[j] = pointTeam[j];
                        circleTeam[j] = new CircleF(pointTeamF[j],HandSize);
                        imageSourse.Draw(circleTeam[j], new Bgra(255, 255, 255, 255), 0);
                        Console.WriteLine("关节" + j.ToString() + "坐标为x=" + pointTeam[j].X.ToString() + ",y=" + pointTeam[j].Y.ToString() + ",z=" + csp[j].Z.ToString());
                    }

                    for (int j = 0; j < (csp.Length - 1); j++)
                        CvInvoke.Line(imageSourse, pointTeam[j], pointTeam[j+1], new MCvScalar(0, 0, 255), 2);
                   
                    imageBox1.Image = imageSourse;
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
        }       
    }

}

