/***********************************************************************
UdevEnumerator - Class to enumerate devices managed by a udev context.
Copyright (c) 2014-2016 Oliver Kreylos

This file is part of the Raw HID Support Library (RawHID).

The Raw HID Support Library is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Raw HID Support Library is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Raw HID Support Library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include <RawHID/Internal/UdevEnumerator.h>

#include <libudev.h>
#include <stdexcept>

#include <RawHID/Internal/UdevContext.h>
#include <RawHID/Internal/UdevListIterator.h>

namespace RawHID {

/*******************************
Methods of class UdevEnumerator:
*******************************/

UdevEnumerator::UdevEnumerator(void)
	:enumerator(0)
	{
	/* Create a new udev context: */
	UdevContext context;
	
	/* Create an enumerator for the new context: */
	enumerator=udev_enumerate_new(context.context);
	if(enumerator==0)
		throw std::runtime_error("RawHID::UdevEnumerator::UdevEnumerator: Cannot create udev enumerator");
	
	/* Reference the context: */
	udev_ref(context.getContext());
	}

UdevEnumerator::UdevEnumerator(UdevContext& context)
	:enumerator(0)
	{
	/* Create an enumerator for the given context: */
	enumerator=udev_enumerate_new(context.getContext());
	if(enumerator==0)
		throw std::runtime_error("RawHID::UdevEnumerator::UdevEnumerator: Cannot create udev enumerator");
	
	/* Reference the context: */
	udev_ref(context.getContext());
	}

UdevEnumerator::~UdevEnumerator(void)
	{
	/* Release the enumerator's udev context: */
	udev_unref(udev_enumerate_get_udev(enumerator));
	
	/* Release the udev enumerator: */
	udev_enumerate_unref(enumerator);
	}

UdevContext UdevEnumerator::getContext(void)
	{
	/* Get the context and reference it: */
	udev* context=udev_enumerate_get_udev(enumerator);
	udev_ref(context);
	
	/* Return a context object: */
	return UdevContext(context);
	}

void UdevEnumerator::addMatchSubsystem(const char* subsystem)
	{
	udev_enumerate_add_match_subsystem(enumerator,subsystem);
	}

void UdevEnumerator::scanDevices(void)
	{
	udev_enumerate_scan_devices(enumerator);
	}

UdevListIterator UdevEnumerator::getDevices(void)
	{
	return UdevListIterator(udev_enumerate_get_list_entry(enumerator));
	}

}
