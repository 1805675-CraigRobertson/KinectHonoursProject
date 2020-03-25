using System;
using System.ComponentModel;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
using Microsoft.Kinect;

namespace HonsProjectKinect
{
    public partial class MainWindow : Window
    {
        //Skeleton
        private const double HandSize = 30;
        private const double JointThickness = 3;
        private const double ClipBoundsThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));       
        private readonly Brush inferredJointBrush = Brushes.Yellow;       
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;
        private int displayWidth;
        private int displayHeight;
        private List<Pen> bodyColors;

        //MultiFrame
        private MultiSourceFrameReader multiFrameSourceReader = null;
        
        //BodyIndex 
        private FrameDescription bodyIndexFrameDescription = null;
        private WriteableBitmap bodyIndexBitmap = null;
        private uint[] bodyIndexPixels = null;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.bones = new List<Tuple<JointType, JointType>>();

            //MultiFrame Reader
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.BodyIndex | FrameSourceTypes.Depth);
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            //BodyIndexDescription
            this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;

            //BodyIndex Pixels Array
            this.bodyIndexPixels = new uint[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];

            //BodyIndex Bitmap
            this.bodyIndexBitmap = new WriteableBitmap(this.bodyIndexFrameDescription.Width, this.bodyIndexFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // open the sensor
            this.kinectSensor.Open();
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.DataContext = this;

            InitializeComponent();
        }

        private unsafe void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e) {
            BodyIndexFrame bodyIndexFrame = null;
            BodyFrame bodyFrame = null;
            DepthFrame depthFrame = null;

            //Get frames
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            //Check is frames are null
            if (multiSourceFrame == null)
            {
                return;
            }

            //Try acquiring frames
            try {
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();

                if ((bodyIndexFrame == null) || (bodyFrame == null) || (depthFrame == null))
                {
                    return;
                }

                Microsoft.Kinect.KinectBuffer bodyIndexBuffer = bodyIndexFrame.LockImageBuffer();
                Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer();

                //bool bodyIndexFrameProcessed = false;
                bool dataReceived = false;

                //BodyIndex verify data and write to bitmap 
                //if (((this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height) == bodyIndexBuffer.Size) &&
                //    (this.bodyIndexFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.bodyIndexFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                //{
                //    this.ProcessBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size);
                //    bodyIndexFrameProcessed = true;
                //}

                //BodyIndex render pixels
                //if (bodyIndexFrameProcessed)
                //{
                //    this.RenderBodyIndexPixels();
                //}

                //Check if BodyFrame null when add new body
                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }
                bodyFrame.GetAndRefreshBodyData(this.bodies);
                dataReceived = true;
               


                if(dataReceived){
                    //Depth
                    ushort* frameDataDepth = (ushort*)depthBuffer.UnderlyingBuffer;
                    //BodyIndex
                    byte* frameDataBodyIndex = (byte*)bodyIndexBuffer.UnderlyingBuffer;

                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // Draw a transparent background to set the render size
                        dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        int penIndex = 0;
                        foreach (Body body in this.bodies)
                        {
                            Pen drawPen = this.bodyColors[penIndex++];


                            if (body.IsTracked)
                            {
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                double totalHeight = getHeight(joints);

                                Console.WriteLine(joints[JointType.HandRight].Position.Y);

                                //Console.WriteLine("{0}", distanceToBody(joints[JointType.HandLeft].Position));
                                //Console.WriteLine(totalHeight);

                                // convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                                foreach (JointType jointType in joints.Keys)
                                {
                                    // sometimes the depth(Z) of an inferred joint may show as negative
                                    // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }

                                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                }

                                this.DrawBody(joints, jointPoints, dc, drawPen);
                            }
                        }

                        // prevent drawing outside of our render area
                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                    bodyIndexBuffer.Dispose();
                    bodyIndexFrame.Dispose();
                    bodyFrame.Dispose();
                    depthFrame.Dispose();
                    depthBuffer.Dispose();
                }
            }
            finally
            {
                if (bodyIndexFrame != null)
                {
                    bodyIndexFrame.Dispose();
                }
                if (bodyFrame != null)
                {
                    bodyFrame.Dispose();
                }
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }
            }
        }

        public static double getHeight(IReadOnlyDictionary<JointType, Joint> joints)
        {
            var head = joints[JointType.Head];
            var neck = joints[JointType.Neck];
            var spineShoulder = joints[JointType.SpineShoulder];
            var spineMid = joints[JointType.SpineMid];
            var spineBase = joints[JointType.SpineBase];
            var hipRight = joints[JointType.HipRight];
            var hipLeft = joints[JointType.HipLeft];
            var kneeRight = joints[JointType.KneeRight];
            var kneeLeft = joints[JointType.KneeLeft];
            var ankleRight = joints[JointType.AnkleRight];
            var ankleLeft = joints[JointType.AnkleLeft];
            var footRight = joints[JointType.FootRight];
            var footLeft = joints[JointType.FootLeft];

            double torsoHeight = getLength(head, neck) + getLength(neck, spineShoulder) + getLength(spineShoulder, spineMid) + getLength(spineMid, spineBase) + (getLength(spineBase, hipLeft) + getLength(spineBase, hipRight)) / 2;

            double leftLegHeight = getLength(hipLeft, kneeLeft) + getLength(kneeLeft, ankleLeft) + getLength(ankleLeft, footLeft);

            double rightLegHeight = getLength(hipRight, kneeRight) + getLength(kneeRight, ankleRight) + getLength(ankleRight, footRight);

            double totalHeight = torsoHeight + (leftLegHeight + rightLegHeight) / 2 + 0.01;

            return totalHeight;
        }

        public static double getLength(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2) +
                Math.Pow(p1.Position.Z - p2.Position.Z, 2));
        }

        public double distanceToBody(CameraSpacePoint point) {
            return Math.Sqrt(
                point.X * point.X +
                point.Y * point.Y +
                point.Z * point.Z
                );
        }

        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        //Gets data to display 
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        //Check if window is loaded
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
                Console.WriteLine("hey");
            }
        }

        //Check if window is closed
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }    
    }
}
