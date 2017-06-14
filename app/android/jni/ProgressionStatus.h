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
#include <jni.h>

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
	ProgressionStatus() : count_(0), max_(100), jvm_(0), rtabmap_(0)
	{
		registerToEventsManager();
	}

	void setJavaObjects(JavaVM * jvm, jobject rtabmap)
	{
		jvm_ = jvm;
		rtabmap_ = rtabmap;
	}

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
	JavaVM *jvm_;
	jobject rtabmap_;
};

}


#endif /* APP_ANDROID_JNI_PROGRESSIONSTATUS_H_ */
