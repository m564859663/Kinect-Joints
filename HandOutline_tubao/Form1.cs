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
using Emgu.CV.Util;
using Emgu.CV.CvEnum;

namespace HandOutline_tubao
{
    public partial class Form1 : Form
    {

        private const double HandSize = 30;//绘制手圈时的半径
        private const double JointThickness = 3;//接缝厚度        
        private const double ClipBoundsThickness = 10;//夹边矩形的厚度        
        private const float InferredZPositionClamp = 0.1f;//常数为夹紧Z值的相机空间点负面

        private FrameDescription bodyIndexFrameDescription = null;
        private FrameDescription depthFrameDescription = null;
        private KinectSensor kinectSensor = null;
        private MultiSourceFrameReader multiSourceFrameReader = null;
        private Body[] _body = null;//身体数组
        byte[] _bodyFrame = null;
        byte[] _bodyIndex = null;//BodyIndex数组
        ushort[] depthData;
        int[] depthDataConvert;

        private int displayWidth;//展示的宽度（纵深空间）
        private int displayHeight;//展示的高度（纵深空间）
        private CoordinateMapper coordinateMapper = null;
        Image<Bgr, byte> imageSourse;
        Image<Gray, byte> b, c;
        Image<Bgr, byte> d;


        public Form1()
        {
            InitializeComponent();
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;//获得Kinect的坐标映射器
            bodyIndexFrameDescription = kinectSensor.BodyIndexFrameSource.FrameDescription;//BodyIndexFrameSource的帧描述
            depthFrameDescription = kinectSensor.DepthFrameSource.FrameDescription;//DepthFrameSource的帧描述

            multiSourceFrameReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.BodyIndex | FrameSourceTypes.Depth);
            multiSourceFrameReader.MultiSourceFrameArrived += Framereader_MultiSourceFrameArrived;

            this.displayWidth = bodyIndexFrameDescription.Width;
            this.displayHeight = bodyIndexFrameDescription.Height;

            depthData = new ushort[depthFrameDescription.LengthInPixels];
            depthDataConvert = new int[depthFrameDescription.LengthInPixels];
            _bodyIndex = new byte[bodyIndexFrameDescription.LengthInPixels * 3];
            _bodyFrame = new byte[bodyIndexFrameDescription.LengthInPixels * 3];
            imageSourse = new Image<Bgr, byte>(displayWidth, displayHeight);
            
            kinectSensor.Open();

            //---------------------------------------------------------------------

            b = new Image<Gray, byte>(imageSourse.Width, imageSourse.Height);
            c = new Image<Gray, byte>(imageSourse.Width, imageSourse.Height);
            d = new Image<Bgr, byte>(imageSourse.Width, imageSourse.Height);
            

        }

        private unsafe void ProcessBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize)
        {
            byte* frameData = (byte*)bodyIndexFrameData;
            for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            {
                if (frameData[i] < 6)
                    if (_bodyFrame[i * 3] == 0 & _bodyFrame[i * 3 + 1] == 0 & _bodyFrame[i * 3 + 2] == 150)
                    {
                        this._bodyIndex[i * 3] = 255;
                        this._bodyIndex[i * 3 + 1] = 255;
                        this._bodyIndex[i * 3 + 2] = 255;
                        //this._bodyIndex[i * 4 + 3] = 255;     //手显示为白色  
                    }
                    else
                    {
                        this._bodyIndex[i * 3] = 0;
                        this._bodyIndex[i * 3 + 1] = 0;
                        this._bodyIndex[i * 3 + 2] = 0;
                        //this._bodyIndex[i * 4 + 3] = 255;     //其他显示为黑色  
                    }
                else
                {
                    this._bodyIndex[i * 3] = 0;
                    this._bodyIndex[i * 3 + 1] = 0;
                    this._bodyIndex[i * 3 + 2] = 0;
                    //this._bodyIndex[i * 3 + 3] = 255;     //其他显示为黑色  
                }
            }
        }

        private void Framereader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();
            d = new Image<Bgr, byte>(imageSourse.Width, imageSourse.Height);
            //骨骼帧和彩色帧都不为空  
            using (var bodyIndexFrame = reference.BodyIndexFrameReference.AcquireFrame())
            {
                using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
                {
                    using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
                    {
                        if (bodyFrame != null)
                        {
                            _body = new Body[bodyFrame.BodyCount];
                            bodyFrame.GetAndRefreshBodyData(this._body);
                            DrawBodies();
                        }

                        if (bodyIndexFrame != null)
                        {
                            using (KinectBuffer kB = bodyIndexFrame.LockImageBuffer())
                            {
                                if (bodyIndexFrameDescription.Width * bodyIndexFrameDescription.Height == kB.Size)
                                    this.ProcessBodyIndexFrameData(kB.UnderlyingBuffer, kB.Size);
                                imageSourse.Bytes = _bodyIndex;
                                imageBox1.Image = imageSourse;
                            }
                        }
                    }
                }
            }

            //CvInvoke.Canny(imageSourse, b, 100, 60);
            CvInvoke.CvtColor(imageSourse, b, Emgu.CV.CvEnum.ColorConversion.Bgr2Gray);
                
            VectorOfVectorOfPoint con = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(b, con, c, RetrType.Ccomp, ChainApproxMethod.ChainApproxSimple);
            Point[][] con1 = con.ToArrayOfArray();
            System.Drawing.PointF[][] con2 = Array.ConvertAll<Point[], System.Drawing.PointF[]>(con1, new Converter<Point[], System.Drawing.PointF[]>(PointToPointF));
            for (int i = 0; i < con.Size; i++)
            {
                System.Drawing.PointF[] hull = CvInvoke.ConvexHull(con2[i], true);
                for (int j = 0; j < hull.Length; j++)
                {
                    Point p1 = new Point((int)(hull[j].X + 0.5), (int)(hull[j].Y + 0.5));
                    Point p2;
                    if (j == hull.Length - 1)
                        p2 = new Point((int)(hull[0].X + 0.5), (int)(hull[0].Y + 0.5));
                    else
                        p2 = new Point((int)(hull[j + 1].X + 0.5), (int)(hull[j + 1].Y + 0.5));
                    CvInvoke.Circle(d, p1, 3, new MCvScalar(0, 255, 255, 255), 6);
                    CvInvoke.Line(d, p1, p2, new MCvScalar(255, 255, 0, 255), 3);
                }
            }
            for (int i = 0; i < con.Size; i++)
                CvInvoke.DrawContours(d, con, i, new MCvScalar(255, 0, 255, 255), 2);

            //imageBox2.Image = imageSourse.ConcateVertical(d);
            imageBox2.Image = d;
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
                    CameraSpacePoint handTipLeft = joints[JointType.HandTipLeft].Position;

                    DepthSpacePoint handLeft_depth = coordinateMapper.MapCameraPointToDepthSpace(handLeft);
                    DepthSpacePoint handTipLeft_depth = coordinateMapper.MapCameraPointToDepthSpace(handTipLeft);

                    float leftZ = handLeft.Z;
                    float circleSizeL = 0f;

                    if (leftZ < 0.5f)
                    { circleSizeL = 70; }
                    else if (leftZ < 1.0f)
                    { circleSizeL = 60; }
                    else if (leftZ < 1.5f)
                    { circleSizeL = 45; }
                    else
                    { circleSizeL = 35; }

                    System.Drawing.PointF leftPointF = new System.Drawing.PointF();

                    leftPointF.X = (handLeft_depth.X + handTipLeft_depth.X) / 2;
                    leftPointF.Y = (handLeft_depth.Y + handTipLeft_depth.Y) / 2;

                    CircleF circle2 = new CircleF(leftPointF, circleSizeL);
                    imageSourse.Draw(circle2, new Bgr(0, 0, 150), 0);
                    _bodyFrame = imageSourse.Bytes;
                }
            }
        }
        public static System.Drawing.PointF[] PointToPointF(Point[] pf)
        {
            System.Drawing.PointF[] aaa = new System.Drawing.PointF[pf.Length];
            int num = 0;
            foreach (var point in pf)
            {
                aaa[num].X = (int)point.X;
                aaa[num++].Y = (int)point.Y;
            }
            return aaa;
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
