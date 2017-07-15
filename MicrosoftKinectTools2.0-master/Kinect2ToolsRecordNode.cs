#region usings
using System;
using System.ComponentModel;
using System.ComponentModel.Composition;
using System.Threading;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;

using VVVV.Core.Logging;
using Microsoft.Kinect.Tools; 

#endregion usings

namespace VVVV.Kinect2Tools.Nodes  
{
	#region PluginInfo
	[PluginInfo(Name = "Record",
				Category = "Kinect2 Tools", 
				Version = "Microsoft", 
				Author = "id144",
				Help = "Create recording of sensor stream using Kinect Tools", 
				Tags = "")]
	#endregion PluginInfo
	
	public class Kinect2ToolsRecordNode : IPluginEvaluate
	{
		#region fields & pins 
		[Input("Record", DefaultValue = 0.0, IsToggle=true)]
		public IDiffSpread<bool> FInputRecord;

        [Input("RGB", IsSingle = true, IsToggle = true, DefaultValue = 1.0)]
        public ISpread<bool> FInputRGBStream;

        [Input("IR", IsSingle = true, IsToggle = true, DefaultValue = 1.0)]
        public ISpread<bool> FInputIRStream;

        [Input("Depth", IsSingle = true, IsToggle = true, DefaultValue = 1.0)]
        public ISpread<bool> FInputDepthStream;		
		
		[Input("Buffer Size", IsSingle = true, DefaultValue = 4096)]
		public ISpread<double> FInputRecordingBufferSizeMB;
        
		[Input("Filename", IsSingle = true, StringType = StringType.Filename, DefaultString = ".xef")]
        public ISpread<string> FInputFilename;
		
		[Output("Status")]
		public ISpread<string> FRecord;

        [Output("Duration")]
        public ISpread<double> FRecordDuration;

        [Output("FileSizeBytes", IsSingle = true)]
        public ISpread<ulong> FRecordFileSizeBytes;

        [Output("BufferSizeMegabytes", IsSingle = true)]
        public ISpread<ulong> FRecordBufferSizeMegabytes;

		[Output("BufferInUseSizeMegabytes", IsSingle = true)]
        public ISpread<ulong> FRecordBufferInUseSizeMegabytes;

        [Import()]
		public ILogger FLogger;		 
		#endregion fields & pins
		
        private delegate void OneArgDelegate(string arg);      
		private bool doRecord;
		private KStudioRecording recording = null;
		private uint recordingBufferSizeMB;
		
		public Kinect2ToolsRecordNode()
		{

		}
		public void Evaluate(int SpreadMax)
		{          
          if (FInputRecord.IsChanged)
          {
              if (FInputRecord[0])
              {
                  string xefFilePath = @FInputFilename[0];

                  if (!string.IsNullOrEmpty(xefFilePath))
                  {		
                  	doRecord = true;
                      OneArgDelegate recording = new OneArgDelegate(this.RecordClip);
		            recording.BeginInvoke(xefFilePath, null, null);
                  }                	
              }
          	else
          	{
          		StopRecording();
          	}
          }
		}

        private void RecordClip(string filePath)
        {
   			using (KStudioClient client = KStudio.CreateClient())
            {          	
                client.ConnectToService();

                KStudioEventStreamSelectorCollection streamCollection = new KStudioEventStreamSelectorCollection();

                if (FInputIRStream[0]) streamCollection.Add(KStudioEventStreamDataTypeIds.Ir);
                if (FInputDepthStream[0]) streamCollection.Add(KStudioEventStreamDataTypeIds.Depth);
                streamCollection.Add(KStudioEventStreamDataTypeIds.Body);
                streamCollection.Add(KStudioEventStreamDataTypeIds.BodyIndex);           	
                if (FInputRGBStream[0]) streamCollection.Add(KStudioEventStreamDataTypeIds.UncompressedColor);

        		recordingBufferSizeMB = (uint)FInputRecordingBufferSizeMB[0];

                recording = client.CreateRecording(filePath, streamCollection, recordingBufferSizeMB, KStudioRecordingFlags.IgnoreOptionalStreams);
				using (recording)                
            	{
	                recording.PropertyChanged += Recording_PropertyChanged;
	            	recording.Start();
	            	
	            	FRecordBufferSizeMegabytes[0] = (uint)recording.BufferSizeMegabytes;
	            	
	                while ((recording.State == KStudioRecordingState.Recording) && (doRecord))
	                {
	                    Thread.Sleep(100);
	                	//recording reports no values :(
	                	FRecordBufferInUseSizeMegabytes[0] = recording.BufferInUseSizeMegabytes;
	                }
	                recording.Stop();

            		//wait until the recording buffer is empty
	        	    while (recording.BufferInUseSizeMegabytes > 0)
	                {
	                    Thread.Sleep(50);
	                    FRecordBufferInUseSizeMegabytes[0] = recording.BufferInUseSizeMegabytes;
	                }
            		
	            	recording.PropertyChanged -= Recording_PropertyChanged;            		
            	}
				client.DisconnectFromService();                
            }				
        }		
        public void StopRecording()
        {
			doRecord = false;
        }
		
		// when we create the recording object in a different threadm this handler gets 
		// fired only at the beginning and the end of the recording
		// when we create the recording object in a main thread, thigs get ugly
		
        private void Recording_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if ((e != null))
            {
                try
                {
                    FRecord[0] = ((KStudioRecording)sender).State.ToString();
                    FRecordDuration[0] = ((KStudioRecording)sender).Duration.TotalMilliseconds;
                    FRecordFileSizeBytes[0] = ((KStudioRecording)sender).FileSizeBytes;
                    FRecordBufferInUseSizeMegabytes[0] = ((KStudioRecording)sender).BufferInUseSizeMegabytes;
                }
                catch (Exception)
                {
                    
                }
            }        					
        }	
	}
}
