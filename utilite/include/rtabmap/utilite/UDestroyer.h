/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
