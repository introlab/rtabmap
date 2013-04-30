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

#ifndef UVARIANT_H
#define UVARIANT_H

#include <string>

/**
 * Experimental class...
 */
class UVariant
{
public:
	virtual ~UVariant() {}

	virtual std::string className() const = 0;

	template<class T>
	const T * data() const {
		if(data_)
			return (T*)data_;
		return (const T*)constData_;
	}

	template<class T>
	T * takeDataOwnership() {
		T * data = (T*)data_;
		constData_ = 0;
		data_=0;
		return data;
	}

protected:
	UVariant(void * data) :
		data_(data),
		constData_(0)
	{}

	UVariant(const void * data) :
		data_(0),
		constData_(data)
	{}

protected:
	void * data_;
	const void * constData_;
};

#endif /* UVARIANT_H */
