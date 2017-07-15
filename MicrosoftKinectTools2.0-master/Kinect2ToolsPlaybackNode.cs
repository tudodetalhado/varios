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
	[PluginInfo(Name = "Playback",
				Category = "Kinect2 Tools", 
				Version = "Microsoft", 
				Author = "id144",
				Help = "Playback recording of sensor stream using Kinect Tools", 
				Tags = "")]
	#endregion PluginInfo
	
	public class Kinect2ToolsPlaybackNode : IPluginEvaluate
	{
		#region fields & pins 
		[Input("Play", DefaultValue = 0.0, IsToggle=true)]
		public IDiffSpread<bool> FInputPlay;

        [Input("Loop", DefaultValue = 0.0, IsToggle = true)]
        public IDiffSpread<bool> FInputLoop;

        [Input("Pause", IsSingle = true, IsToggle = true, DefaultValue = 0.0)]
        public IDiffSpread<bool> FInputPause;

        [Input("StepOnce", IsSingle = true, IsBang = true, DefaultValue = 0.0)]
        public IDiffSpread<bool> FInputStep;
	
		[Input("Do Seek", IsSingle = true, IsBang = true, DefaultValue = 0.0)]
        public IDiffSpread<bool> FInputDoSeek;

        [Input("Seek Time", IsSingle = true,  DefaultValue = 0.0)]
        public IDiffSpread<double> FInputSeek;
		
        [Input("In Point", IsSingle = true,  DefaultValue = 0.0, MinValue = 0.0)]
        public IDiffSpread<double> FInputInPoint;

		[Input("Out Point", IsSingle = true,  DefaultValue = 999.0, MinValue = 0.0)]
        public IDiffSpread<double> FInputOutPoint;
		
        [Input("Filename", IsSingle = true, StringType = StringType.Filename, FileMask = "*.xef",  DefaultString = "")]
        public IDiffSpread<string> FInputFilename;
		
		[Output("Status")]
		public ISpread<string> FRecord;

        [Output("Time")]
        public ISpread<double> FRecordDuration;

        [Output("Duration", IsSingle = true)]
        public ISpread<double> FRecordTotalDuration;

        [Import()]
		public ILogger FLogger;		 
		#endregion fields & pins
		
        private delegate void OneArgDelegate(string arg, bool arg2);      
		private bool doPlay;
		private bool doLoop;
		private KStudioPlayback playback = null;
		private string xefFilePath;
	
		public Kinect2ToolsPlaybackNode()
		{

		}
		private bool doStep;

		public bool fileChanged;


		public void Evaluate(int SpreadMax)
		{
            if (FInputFilename.IsChanged) 	
			{
				if (playback!=null)	
				{
					if ((playback.State == KStudioPlaybackState.Playing) || (playback.State == KStudioPlaybackState.Paused) )
                    {
                    	playback.Stop();                    	
                    }
					fileChanged = true;
				}
			}
			
            if (FInputPlay.IsChanged || fileChanged  || doLoop)
            {
                doLoop = false;

                if (FInputPlay[0])
                {
                   	xefFilePath = @FInputFilename[0];

                    if (!string.IsNullOrEmpty(xefFilePath))
                    {		
                    	doPlay = true;
                        OneArgDelegate recording = new OneArgDelegate(this.PlayClip);
			            recording.BeginInvoke(xefFilePath, false, null, null);
                    }                	
                }
            	else
            	{
                    if (playback != null) playback.Stop();
            	}
            }
            if (FInputPause.IsChanged)
            {
				if (playback != null) 
            	{
	                if (FInputPause[0])
	                {
	                    if ((playback.State == KStudioPlaybackState.Playing))
	                    {
	                        playback.Pause();
	                    }	
	                }
	                else
	                {
	                    if ((playback.State == KStudioPlaybackState.Paused))
	                    {
	                        playback.Resume();
	                    }	
	                }            		
            	} 
            	else
            	{
            		    if (FInputPause[0])
		            	{
		            		    doPlay = true;
		                        OneArgDelegate recording = new OneArgDelegate(this.PlayClip);
					            recording.BeginInvoke(xefFilePath, true, null, null);
		            	}
            	}
            }
			fileChanged = false;
            if (FInputStep[0])
            {
            	doStep = true;
            }
            if (FInputDoSeek[0] && (playback !=null))
            {
                TimeSpan seekTime =  TimeSpan.FromSeconds(FInputSeek[0]);
            	try
				{
	                playback.Pause();
	                playback.SeekByRelativeTime(seekTime);
	                playback.Resume();					
				}
            	catch
            	{
            		
            	}
            }
			if (playback!=null)
			{
				try
				{
					FRecordDuration[0] = playback.CurrentRelativeTime.TotalSeconds;	                	
					FRecord[0] =  playback.State.ToString();					
				}
				catch 
				{
					
				}
			}            
        }

        private void PlayClip(string filePath, bool startpaused)
        {
   			using (KStudioClient client = KStudio.CreateClient())
            {          	
                client.ConnectToService();

				KStudioPlaybackFlags flags = KStudioPlaybackFlags.IgnoreOptionalStreams;
                try
                {
                    playback = client.CreatePlayback(filePath, flags);
                }
                catch (Exception ex)
                {
                	FLogger.Log(LogType.Debug, ex.ToString());
                }
				using (playback)                
            	{
	                playback.PropertyChanged += Playback_PropertyChanged;
                    playback.StateChanged += Playback_StateChanged;

                   	playback.LoopCount = 0;
                    playback.InPointByRelativeTime = TimeSpan.FromSeconds(FInputInPoint[0]);
                    playback.OutPointByRelativeTime = TimeSpan.FromSeconds(Math.Min(FInputOutPoint[0],playback.Duration.TotalSeconds));
                    var kStudioMetadata = playback.GetMetadata(KStudioMetadataType.Public);

                    if (startpaused)
            		{
            			playback.StartPaused();            			
            		}
            		else
            		{
	            		playback.Start();
            		};

	            	FRecordTotalDuration[0] = playback.Duration.TotalSeconds;

	                while ((playback.State == KStudioPlaybackState.Playing)||(playback.State == KStudioPlaybackState.Paused) && (doPlay))
	                {
	                	
	                	if (doStep)
	                	{
							if ((playback.State == KStudioPlaybackState.Playing))
							{
							    	playback.Pause();					
							}            	
							if(playback.State == KStudioPlaybackState.Paused)
							{
								playback.StepOnce();
								Thread.Sleep(20);                        
							}
	                		doStep = false;
	                	}
	                    Thread.Sleep(20);                        
                    }
	                playback.Stop();            		
            	}

            	playback = null;
				client.DisconnectFromService();                
            }				
        }		
        public void StopPlayback()
        {
			doPlay = false;
        }

        private void Playback_StateChanged(object sender, EventArgs e)
        {

        }

        private void Playback_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if ((e != null))
            {
            	if (e.PropertyName == "State")            	
            	{
            		if (playback.State == KStudioPlaybackState.Stopped)
                    {
                        doLoop = (FInputPlay[0] && FInputLoop[0]);
                    }
            	}
                try
                {
                	
                }
                catch (Exception)
                {
                    
                }
            }        					
        }	
	}
}
