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

#ifndef UDESTROYER_H
#define UDESTROYER_H

/**
 * This class is used to delete a dynamically created
 * objects. It was mainly designed to remove dynamically created Singleton.
 * Created on the stack of a Singleton, when the
 * application is finished, his destructor make sure that the 
 * Singleton is deleted.
 *
 */
template <class T>
class UDestroyer
{
public:
	/**
	 * The constructor. Set the doomed object (take ownership of the object). The object is deleted
	 * when this object is deleted.
	 */
    UDestroyer(T* doomed = 0)  : doomed_(doomed) {}
    
    ~UDestroyer()
    {
        if(doomed_)
        {
            delete doomed_;
            doomed_ = 0;
        }
    }

    /**
	 * Set the doomed object. If a doomed object is already set, the function returns false.
	 * @param doomed the doomed object
	 * @return false if an object is already set and the new object is not null, otherwise true
	 */
    bool setDoomed(T* doomed)
	{
    	if(doomed_ && doomed)
    	{
    		return false;
    	}
		doomed_ = doomed;
		return true;
	}

private:
    // Prevent users from making copies of a 
    // Destroyer to avoid double deletion:
    UDestroyer(const UDestroyer<T>&);
    void operator=(const UDestroyer<T>&);

private:
    T* doomed_;
};

#endif // UDESTROYER_H
