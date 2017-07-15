/***********************************************************************
PSMove - Class to represent a PlayStation Move game controller as an
inertially-tracked input device.
Copyright (c) 2013-2016 Oliver Kreylos

This file is part of the optical/inertial sensor fusion tracking
package.

The optical/inertial sensor fusion tracking package is free software;
you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

The optical/inertial sensor fusion tracking package is distributed in
the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the optical/inertial sensor fusion tracking package; if not, write
to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA 02111-1307 USA
***********************************************************************/

#ifndef PSMOVE_INCLUDED
#define PSMOVE_INCLUDED

#include <string>
#include <Threads/Thread.h>
#include <RawHID/Device.h>

#include "IMU.h"

class PSMove:public RawHID::Device,public IMU
	{
	/* Embedded classes: */
	public:
	struct FeatureState // Structure to report the state of buttons and valuators on the PS Move device
		{
		/* Elements: */
		public:
		bool buttons[9]; // Button states: Select, Start, Triangle, Circle, X, Square, PS, Move, Trigger
		unsigned char valuators[1]; // Valuator state: Trigger
		};
	
	typedef Misc::FunctionCall<const FeatureState&> FeatureStateCallback; // Type of callback called when the PS Move's feature state is updated
	typedef Misc::FunctionCall<int> BatteryStateCallback; // Type of callback called when the state of the PS Move's battery changes
	
	/* Elements: */
	private:
	FeatureStateCallback* featureStateCallback; // Callback called when a new feature state packet arrives
	BatteryStateCallback* batteryStateCallback; // Callback called when the state of the battery changes
	Threads::Thread samplingThread; // Thread object for the background sampling thread
	volatile bool keepSampling; // Flag to shut down the background sampling thread
	unsigned char ledColor[3]; // Current color of the LED ball in RGB with [0..255] components
	volatile bool ledColorChanged; // Flag whether the LED color has changed since the last sample
	bool showSamplingError; // Flag whether to display an error message when the sampling thread terminates due to an exception
	
	/* Private methods: */
	void initialize(void); // Initializes the PS Move after the raw HID device has been opened
	void* samplingThreadMethod(void); // Thread method for the background sampling thread
	
	/* Constructors and destructors: */
	public:
	PSMove(const char* devnode,const char* serialNumber); // Creates a PS Move device based on the given raw HID device node and serial number
	PSMove(unsigned int deviceIndex); // Connects to the PS Move controller of the given zero-based index on the local HID bus
	PSMove(const std::string& deviceSerialNumber); // Connects to the PS Move controller of the given serial number on the local HID bus
	virtual ~PSMove(void);
	
	/* Methods from IMU: */
	virtual std::string getSerialNumber(void) const;
	virtual Scalar getAccelerometerScale(void) const;
	virtual Scalar getGyroscopeScale(void) const;
	virtual Scalar getMagnetometerScale(void) const;
	virtual void startStreamingRaw(RawSampleCallback* newRawSampleCallback);
	virtual void startStreamingCalibrated(CalibratedSampleCallback* newCalibratedSampleCallback);
	virtual void stopStreaming(void);
	
	/* New methods: */
	void disableSamplingError(void); // Disable the error message that appears when the sampling thread terminates due to an exception
	void setFeatureStateCallback(FeatureStateCallback* newFeatureStateCallback); // Sets a callback to be called when the PS Move's feature state is updated
	void setBatteryStateCallback(BatteryStateCallback* newBatteryStateCallback); // Sets a callback to be called when the PS Move's battery's state changes
	void setLedColor(unsigned char red,unsigned char green,unsigned char blue); // Sets the LED ball's color; reduces sampling performance if called more than a few times per second
	};

#endif
