/***********************************************************************
SketchingTool - Tool to create and edit 3D curves.
Copyright (c) 2009-2015 Oliver Kreylos

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

#ifndef VRUI_SKETCHINGTOOL_INCLUDED
#define VRUI_SKETCHINGTOOL_INCLUDED

#include <string>
#include <vector>
#include <Geometry/Point.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <GLMotif/RadioBox.h>
#include <GLMotif/NewButton.h>
#include <GLMotif/TextFieldSlider.h>
#include <GLMotif/FileSelectionDialog.h>
#include <Vrui/Geometry.h>
#include <Vrui/UtilityTool.h>

/* Forward declarations: */
namespace IO {
class ValueSource;
class OStream;
}
namespace GLMotif {
class PopupWindow;
class RowColumn;
class FileSelectionHelper;
}

namespace Vrui {

class SketchingTool;

class SketchingToolFactory:public ToolFactory
	{
	friend class SketchingTool;
	
	/* Elements: */
	private:
	Scalar detailSize; // Minimal length of line segments in curves in physical coordinate units
	std::string curvesFileName; // Default name for curve files
	GLMotif::FileSelectionHelper* curvesSelectionHelper; // Helper object to load and save curve files
	
	/* Constructors and destructors: */
	public:
	SketchingToolFactory(ToolManager& toolManager);
	virtual ~SketchingToolFactory(void);
	
	/* Methods from ToolFactory: */
	virtual const char* getName(void) const;
	virtual const char* getButtonFunction(int buttonSlotIndex) const;
	virtual Tool* createTool(const ToolInputAssignment& inputAssignment) const;
	virtual void destroyTool(Tool* tool) const;
	
	/* New methods: */
	GLMotif::FileSelectionHelper* getCurvesSelectionHelper(void); // Returns pointer to a file selection helper for curve files
	};

class SketchingTool:public UtilityTool
	{
	friend class SketchingToolFactory;
	
	/* Embedded classes: */
	private:
	typedef GLColor<GLubyte,4> Color; // Type for colors
	
	struct SketchObject // Base class for sketching objects
		{
		/* Elements: */
		public:
		GLfloat lineWidth; // Curve's cosmetic line width
		Color color; // Curve's color
		
		/* Constructors and destructors: */
		virtual ~SketchObject(void);
		
		/* Methods: */
		virtual void write(IO::OStream& os) const; // Writes object state to file
		virtual void read(IO::ValueSource& vs); // Reads object state from file
		virtual void glRenderAction(GLContextData& contextData) const =0; // Method to render the sketching object into the current OpenGL context
		};
	
	struct Curve:public SketchObject // Structure to represent single-stroke curves
		{
		/* Embedded classes: */
		public:
		struct ControlPoint // Structure for curve control points
			{
			/* Elements: */
			public:
			Point pos; // Control point position
			Scalar t; // Control point sample time
			};
		
		/* Elements: */
		std::vector<ControlPoint> controlPoints; // The curve's control points
		
		/* Methods from SketchObject: */
		virtual void write(IO::OStream& os) const;
		virtual void read(IO::ValueSource& vs);
		virtual void glRenderAction(GLContextData& contextData) const;
		};
	
	struct Polyline:public SketchObject // Structure to represent polylines
		{
		/* Elements: */
		public:
		std::vector<Point> vertices; // The polyline's vertices
		
		/* Methods from SketchObject: */
		virtual void write(IO::OStream& os) const;
		virtual void read(IO::ValueSource& vs);
		virtual void glRenderAction(GLContextData& contextData) const;
		};
	
	/* Elements: */
	static SketchingToolFactory* factory; // Pointer to the factory object for this class
	static const Color colors[8]; // Standard line color palette
	GLMotif::PopupWindow* controlDialogPopup;
	GLMotif::RowColumn* colorBox;
	std::vector<SketchObject*> sketchObjects; // The list of existing sketching objects
	int newSketchObjectType; // Type of sketch objects to be created
	GLfloat newLineWidth; // Line width for new sketch objects
	Color newColor; // Color for new sketch objects
	bool active; // Flag whether the tool is currently creating a sketching object
	Curve* currentCurve; // Pointer to the currently created curve
	Polyline* currentPolyline; // Pointer to the currently created polyline
	Point lastPoint; // The last point appended to the current sketching object
	Point currentPoint; // The current dragging position
	
	/* Constructors and destructors: */
	public:
	SketchingTool(const Vrui::ToolFactory* sFactory,const Vrui::ToolInputAssignment& inputAssignment);
	virtual ~SketchingTool(void);
	
	/* Methods from Vrui::Tool: */
	virtual const Vrui::ToolFactory* getFactory(void) const
		{
		return factory;
		}
	virtual void buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData);
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	
	/* New methods: */
	void sketchObjectTypeCallback(GLMotif::RadioBox::ValueChangedCallbackData* cbData);
	void lineWidthSliderCallback(GLMotif::TextFieldSlider::ValueChangedCallbackData* cbData);
	void colorButtonSelectCallback(GLMotif::NewButton::SelectCallbackData* cbData);
	void saveCurvesCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void loadCurvesCallback(GLMotif::FileSelectionDialog::OKCallbackData* cbData);
	void deleteAllCurvesCallback(Misc::CallbackData* cbData);
	};

}

#endif
