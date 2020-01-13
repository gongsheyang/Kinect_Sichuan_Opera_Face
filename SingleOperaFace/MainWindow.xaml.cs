using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace SingleOperaFace
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// 最多可支持检测到的脸
        /// </summary>
        int maxFaceNum = 8;

        /// <summary>
        /// 当前检测到的人数
        /// </summary>
        int count = 0;

        private KinectSensor kinectSensor = null;
        private ColorFrameReader colorFrameReader = null;
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// 脸
        /// </summary>
        Image[] faces = new Image[6];

        /// <summary>
        /// 图片资源
        /// </summary>
        BitmapImage[] bitimg = new BitmapImage[8];

        /// <summary>
        /// 脸（faces）对应的图片
        /// </summary>
        int[] faceimg = new int[6];

        /// <summary>
        /// 脸谱大小
        /// </summary>
        double[] facesSize = new double[6];

        /// <summary>
        /// 控制变脸时脸谱变换太频繁
        /// </summary>
        bool[] changeFace = new bool[6];
        Body[] bodies;

        /// <summary>
        /// 是否开始变脸
        /// </summary>
        bool[] startChangeFace = new bool[6];
        MultiSourceFrameReader msfr;

        public MainWindow()
        {
            InitializeComponent();

            //脸谱图片资源初始化
            for (int i = 0; i < bitimg.Length; i++)
            {
                bitimg[i] = new BitmapImage(new Uri(string.Format("./Images/{0}.png", i + 1), UriKind.Relative));
            }
            bodies = new Body[6];

            //向canvas中添加初始化图片对象
            for (int i = 0; i < faces.Length; i++)
            {
                faces[i] = new Image();
                faceCanves.Children.Add(faces[i]);
            }
            this.kinectSensor = KinectSensor.GetDefault();
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            msfr = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.Color);
            msfr.MultiSourceFrameArrived += msfr_MultiSourceFrameArrived;
            this.kinectSensor.Open();

        }

        private void msfr_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame msf = e.FrameReference.AcquireFrame();
            if (msf != null)
            {
                using (BodyFrame bodyFrame = msf.BodyFrameReference.AcquireFrame())
                {
                    using (ColorFrame colorFrame = msf.ColorFrameReference.AcquireFrame())
                    {
                        if (bodyFrame != null && colorFrame != null)
                        {
                            FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                            using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                            {
                                this.colorBitmap.Lock();
                                if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                                {
                                    colorFrame.CopyConvertedFrameDataToIntPtr(this.colorBitmap.BackBuffer, (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4), ColorImageFormat.Bgra);
                                    this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                                }
                                this.colorBitmap.Unlock();
                                bodyFrame.GetAndRefreshBodyData(bodies);
                                count = 0;

                                for (int i = 0; i < bodies.Length; i++)
                                {
                                    //如果追踪到body
                                    if (bodies[i].IsTracked)
                                    {
                                        count++;
                                        paintingFace(bodies, i);
                                    }
                                    else
                                    {
                                        editFace(faces[i], faceimg[i], 0, 0, 0, 0, 0);
                                    }
                                }
                                if (count != 0)
                                {
                                    sayHello.Content = string.Format("欢迎来到Kinect世界！目前已有{0}人进入···", count);
                                }
                                else
                                {
                                    sayHello.Content = string.Format("欢迎来到Kinect世界！");
                                }
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 绘制脸谱
        /// </summary>
        /// <param name="bodies"></param>
        /// <param name="i"></param>
        private void paintingFace(Body[] bodies, int i)
        {
            Joint headJoint = bodies[i].Joints[JointType.Head];
            Joint neck = bodies[i].Joints[JointType.Neck];
            Joint handLeft = bodies[i].Joints[JointType.HandLeft];
            Joint handRight = bodies[i].Joints[JointType.HandRight];
            Joint shoulderLeft = bodies[i].Joints[JointType.ShoulderLeft];
            Joint shoulderRight = bodies[i].Joints[JointType.ShoulderRight];
            if (headJoint.TrackingState == TrackingState.Tracked)
            {

                ColorSpacePoint msr = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(headJoint.Position);
                ColorSpacePoint joint_neck = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(neck.Position);
                ColorSpacePoint joint_handLeft = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(handLeft.Position);
                ColorSpacePoint joint_handRight = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(handRight.Position);
                ColorSpacePoint joint_shoulderLeft = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(shoulderLeft.Position);
                ColorSpacePoint joint_shoulderRight = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(shoulderRight.Position);
                if (startChangeFace[i] == true)
                {
                    if (checkEnd(msr, joint_handLeft, joint_handRight, i))
                    {
                        startChangeFace[i] = false;
                    }
                    if (changeFaceMethod(msr, joint_handLeft, joint_handRight))//触发变脸的方法
                    {
                        if (!changeFace[i])
                        {
                            faceimg[i] = (++faceimg[i]) % faceimg.Length;//当前body应该对应的脸谱id
                            changeFace[i] = true;//每次只触发一次变脸
                        }
                    }
                    else
                    {
                        changeFace[i] = false;
                    }
                    refaceSize(headJoint, i);//矫正脸谱大小
                    double toLeft = (msr.X - facesSize[i] / 2) * 1600 / 1920;//在X轴方向矫正脸谱
                    double toTop = (msr.Y - (facesSize[i] / 2) * headJoint.Position.Z) * 900 / 1080;//在Y轴方向矫正脸谱
                    editFace(faces[i], faceimg[i], facesSize[i], facesSize[i], toLeft, 700 - toTop- 20 / headJoint.Position.Z, faceAngle(msr, joint_neck));//更新显示脸谱
                }
                else
                {
                    checkStart(msr, joint_handLeft, joint_handRight,i);
                }
            }
        }

        /// <summary>
        /// 触发变脸的方法
        /// </summary>
        /// <param name="joint_head"></param>
        /// <param name="joint_handLeft"></param>
        /// <param name="joint_handRight"></param>
        /// <returns></returns>
        private bool changeFaceMethod(ColorSpacePoint joint_head, ColorSpacePoint joint_handLeft, ColorSpacePoint joint_handRight)
        {
            //通过判断左手或右手关节点与头结点的距离是否小于7厘米来触发变脸；单位：毫米
            bool isOverMatrix = ((Math.Abs(joint_handLeft.X - joint_head.X) < 70 && Math.Abs(joint_handLeft.Y - joint_head.Y) < 70)
                || (Math.Abs(joint_handRight.X - joint_head.X) < 70 && Math.Abs(joint_handRight.Y - joint_head.Y) < 70));
            return isOverMatrix;
        }

        private bool checkStart(ColorSpacePoint joint_head, ColorSpacePoint joint_handLeft, ColorSpacePoint joint_handRight, int i)
        {
            //通过判断左手或右手关节点与头结点的距离是否小于7厘米来触发变脸；单位：毫米
            if (startChangeFace[i] == false)
            {
                startChangeFace[i] = ((Math.Abs(joint_handLeft.X - joint_head.X) < 70 && Math.Abs(joint_handLeft.Y - joint_head.Y) < 70)
                || (Math.Abs(joint_handRight.X - joint_head.X) < 70 && Math.Abs(joint_handRight.Y - joint_head.Y) < 70));
            }
            return false;
        }
        private bool checkEnd(ColorSpacePoint joint_head, ColorSpacePoint joint_handLeft, ColorSpacePoint joint_handRight, int i)
        {
            //通过判断左手或右手关节点与头结点的距离是否小于7厘米来触发变脸；单位：毫米
            if (startChangeFace[i] == true)
            {
                return -(joint_handLeft.Y - joint_head.Y) > 100&& -(joint_handRight.Y - joint_head.Y) > 100;
            }
            else
            {
                return false;
            }
        }


        /// <summary>
        /// 脸谱的角度
        /// </summary>
        /// <param name="msr"></param>
        /// <param name="joint_neck"></param>
        /// <returns></returns>
        private double faceAngle(ColorSpacePoint msr, ColorSpacePoint joint_neck)
        {
            double tanFace = ((msr.X - joint_neck.X) / (msr.Y - joint_neck.Y));//获取头结点和脖子结点所形成的正切 
            double angles = -Math.Atan(tanFace) / Math.PI * 180;//将正切值转化为角度
            return angles;
        }

        /// <summary>
        /// 脸谱大小校正
        /// </summary>
        /// <param name="head"></param>
        /// <param name="i">body索引</param>
        private void refaceSize(Joint head, int i)
        {
            facesSize[i] = 280 / head.Position.Z;//设置脸谱大小
        }

        /// <summary>
        /// 更新显示脸谱
        /// </summary>
        /// <param name="image">body对应的脸</param>
        /// <param name="i">body索引</param>
        /// <param name="imgWidth">图片宽度</param>
        /// <param name="imgHeight">脸谱图片高度</param>
        /// <param name="x">脸谱坐标x</param>
        /// <param name="y">脸谱坐标y</param>
        /// <param name="angle">脸谱角度</param>
        private void editFace(Image image, int i, double imgWidth, double imgHeight, double x, double y, double angle)
        {
            image.Source = bitimg[i];
            image.Width = imgWidth;
            image.Height = imgHeight;
            image.SetValue(Canvas.LeftProperty, x);
            image.SetValue(Canvas.BottomProperty, y);
            image.RenderTransform = new RotateTransform(angle, imgWidth / 2, imgHeight / 2);
        }

        //加载图片
        private void image_Loaded(object sender, RoutedEventArgs e)
        {
            image.Source = colorBitmap;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }
            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
    }
}