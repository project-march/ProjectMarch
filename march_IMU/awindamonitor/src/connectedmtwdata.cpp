/*	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "../include/awindamonitor/connectedmtwdata.h"

ConnectedMTwData::ConnectedMTwData()
	:m_batteryLevel(0)
	,m_rssi(0)
	,m_effectiveUpdateRate(0)
	,m_orientation(XsEuler(0.0, 0.0, 0.0))
    ,m_frameSkipsList(QList<int>())
    ,m_sumFrameSkips(0)
	,m_containsOrientation(false)
{
}

ConnectedMTwData::~ConnectedMTwData()
{
}

int ConnectedMTwData::batteryLevel() const
{
    return m_batteryLevel;
}

int ConnectedMTwData::rssi() const
{
    return m_rssi;
}

int ConnectedMTwData::effectiveUpdateRate() const
{
    return m_effectiveUpdateRate;
}

XsEuler ConnectedMTwData::orientation() const
{
	return m_orientation;
}

QList<int>* ConnectedMTwData::frameSkipsList()
{
    return &m_frameSkipsList;
}

unsigned int ConnectedMTwData::sumFrameSkips() const
{
    return m_sumFrameSkips;
}

void ConnectedMTwData::setBatteryLevel(int newBatteryLevel)
{
    m_batteryLevel = newBatteryLevel;
}

void ConnectedMTwData::setRssi(int newRssi)
{
    m_rssi = newRssi;
}

void ConnectedMTwData::setEffectiveUpdateRate(int newEffectiveUpdateRate)
{
    m_effectiveUpdateRate = newEffectiveUpdateRate;
}

void ConnectedMTwData::setOrientation(XsEuler newOrientation)
{
    m_orientation = newOrientation;
	m_containsOrientation = true;
}

void ConnectedMTwData::setSumFrameSkips(int newSumFrameSkips)
{
    m_sumFrameSkips = newSumFrameSkips;
}

bool ConnectedMTwData::containsOrientation() const
{
	return m_containsOrientation;
}
