/***********************************************************************
TouchpadButtonsTool - Transform a clickable touchpad or analog stick to
multiple buttons arranged around a circle.
Copyright (c) 2016 Oliver Kreylos

This file is part of the Virtual Reality User Interface Library (Vrui).

The Virtual Reality User Interface Library is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Virtual Reality User Interface Library is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Virtual Reality User Interface Library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef VRUI_TOUCHPADBUTTONSTOOL_INCLUDED
#define VRUI_TOUCHPADBUTTONSTOOL_INCLUDED

#include <Vrui/TransformTool.h>

namespace Vrui {

class TouchpadButtonsTool;

class TouchpadButtonsToolFactory:public ToolFactory
	{
	friend class TouchpadButtonsTool;
	
	/* Embedded classes: */
	private:
	struct Configuration // Structure containing tool settings
		{
		/* Elements: */
		public:
		int numButtons; // Number of buttons to arrange around the circumference of the touchpad
		double centerRadius; // Central radius in touchpad coordinates around which buttons are arranged
		bool useCenterButton; // Flag whether the touchpad's center area emulates an additional button
		
		/* Constructors and destructors: */
		Configuration(void); // Creates default configuration
		
		/* Methods: */
		void read(const Misc::ConfigurationFileSection& cfs); // Overrides configuration from configuration file section
		void write(Misc::ConfigurationFileSection& cfs) const; // Writes configuration to configuration file section
		};
	
	/* Elements: */
	Configuration configuration; // Default configuration for all tools
	
	/* Constructors and destructors: */
	public:
	TouchpadButtonsToolFactory(ToolManager& toolManager);
	virtual ~TouchpadButtonsToolFactory(void);
	
	/* Methods from ToolFactory: */
	virtual const char* getName(void) const;
	virtual const char* getButtonFunction(int buttonSlotIndex) const;
	virtual const char* getValuatorFunction(int valuatorSlotIndex) const;
	virtual Tool* createTool(const ToolInputAssignment& inputAssignment) const;
	virtual void destroyTool(Tool* tool) const;
	};

class TouchpadButtonsTool:public TransformTool
	{
	friend class TouchpadButtonsToolFactory;
	
	/* Elements: */
	private:
	static TouchpadButtonsToolFactory* factory; // Pointer to the factory object for this class
	TouchpadButtonsToolFactory::Configuration configuration; // Private configuration of this tool
	int pressedButton; // Index of currently pressed button, or -1
	
	/* Constructors and destructors: */
	public:
	TouchpadButtonsTool(const ToolFactory* factory,const ToolInputAssignment& inputAssignment);
	virtual ~TouchpadButtonsTool(void);
	
	/* Methods from Tool: */
	virtual void initialize(void);
	virtual const ToolFactory* getFactory(void) const;
	virtual void buttonCallback(int buttonSlotIndex,InputDevice::ButtonCallbackData* cbData);
	
	/* Methods from class DeviceForwarder: */
	virtual InputDeviceFeatureSet getSourceFeatures(const InputDeviceFeature& forwardedFeature);
	virtual InputDeviceFeatureSet getForwardedFeatures(const InputDeviceFeature& sourceFeature);
	};

}

#endif
