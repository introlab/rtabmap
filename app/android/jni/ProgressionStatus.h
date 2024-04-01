/*
 * ProgressionStatus.h
 *
 *  Created on: Feb 28, 2017
 *      Author: mathieu
 */

#ifndef APP_ANDROID_JNI_PROGRESSIONSTATUS_H_
#define APP_ANDROID_JNI_PROGRESSIONSTATUS_H_

#include <rtabmap/core/ProgressState.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#ifdef __ANDROID__
#include <jni.h>
#endif

namespace rtabmap {

class ProgressEvent : public UEvent
{
public:
	ProgressEvent(int count = 1) : count_(count){}
	virtual std::string getClassName() const {return "ProgressEvent";}

	int count_;
};

class ProgressionStatus: public ProgressState, public UEventsHandler
{
public:
	ProgressionStatus() : count_(0), max_(100)
#ifdef __ANDROID__
    , jvm_(0), rtabmap_(0)
#else
    , swiftClassPtr_(0)
#endif
	{
		registerToEventsManager();
	}

#ifdef __ANDROID__
	void setJavaObjects(JavaVM * jvm, jobject rtabmap)
	{
		jvm_ = jvm;
		rtabmap_ = rtabmap;
	}
#else
    void setSwiftCallback(void * classPtr, void(*callback)(void *, int, int))
    {
        swiftClassPtr_ = classPtr;
        swiftCallback = callback;
    }
#endif

	void reset(int max)
	{
		count_=-1;
		max_ = max;
		setCanceled(false);

		increment();
	}

	void setMax(int max)
	{
		max_ = max;
	}
	int getMax() const {return max_;}

	void increment(int count = 1) const
	{
		UEventsManager::post(new ProgressEvent(count));
	}

	void finish()
	{

	}

	virtual bool callback(const std::string & msg) const
	{
		if(!isCanceled())
		{
			increment();
		}

		return ProgressState::callback(msg);
	}
	virtual ~ProgressionStatus(){}

protected:
	virtual bool handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("ProgressEvent") == 0)
		{
			count_ += ((ProgressEvent*)event)->count_;
			// Call JAVA callback
			bool success = false;
#ifdef __ANDROID__
			if(jvm_ && rtabmap_)
			{
				JNIEnv *env = 0;
				jint rs = jvm_->AttachCurrentThread(&env, NULL);
				if(rs == JNI_OK && env)
				{
					jclass clazz = env->GetObjectClass(rtabmap_);
					if(clazz)
					{
						jmethodID methodID = env->GetMethodID(clazz, "updateProgressionCallback", "(II)V" );
						if(methodID)
						{
							env->CallVoidMethod(rtabmap_, methodID,
									count_,
									max_);
							success = true;
						}
					}
				}
				jvm_->DetachCurrentThread();
			}
#else // APPLE
            if(swiftClassPtr_)
            {
                std::function<void()> actualCallback = [&](){
                    swiftCallback(swiftClassPtr_, count_, max_);
                };
                actualCallback();
                success = true;
            }
#endif
			if(!success)
			{
				UERROR("Failed to call rtabmap::updateProgressionCallback");
			}
		}
		return false;
	}

private:
	int count_;
	int max_;
#ifdef __ANDROID__
	JavaVM *jvm_;
	jobject rtabmap_;
#else
    void * swiftClassPtr_;
    void(*swiftCallback)(void *, int, int);
#endif
};

}


#endif /* APP_ANDROID_JNI_PROGRESSIONSTATUS_H_ */
