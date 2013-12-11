/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/UEventsManager.h"

UEventsHandler::~UEventsHandler()
{
	unregisterFromEventsManager();
}


void UEventsHandler::registerToEventsManager()
{
	UEventsManager::addHandler(this);
}
void UEventsHandler::unregisterFromEventsManager()
{
	UEventsManager::removeHandler(this);
}
