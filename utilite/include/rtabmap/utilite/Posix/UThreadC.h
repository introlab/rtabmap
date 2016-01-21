/////////////////////////////////////////////////////////////////////
//  Written by Phillip Sitbon
//  Copyright 2003
//
//  Modified by Mathieu Labbe
//
//  Posix/Thread.h
//    - Posix thread
//
/////////////////////////////////////////////////////////////////////

#ifndef _U_Thread_Posix_
#define _U_Thread_Posix_

#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/utilite/UMutex.h"

#include <pthread.h>

/**
 * Calling thread sleeps for some milliseconds.
 */
inline void uSleep(unsigned int ms)
{
	struct timespec req;
	struct timespec rem;
	req.tv_sec = ms / 1000;
	req.tv_nsec = (ms - req.tv_sec * 1000) * 1000 * 1000;
	nanosleep (&req, &rem);
}

/**
 * Calling thread sleeps for some microseconds.
 */
inline void uSleepMicro(unsigned int us)
{
	struct timespec req;
	struct timespec rem;
	req.tv_sec = us / 1000000;
	req.tv_nsec = (us - req.tv_sec * 1000000) * 1000;
	nanosleep (&req, &rem);
}

/**
 * Calling thread sleeps for some nanoseconds.
 */
inline void uSleepNano(unsigned int ns)
{
	struct timespec req;
	struct timespec rem;
	req.tv_sec = ns / 1000000000;
	req.tv_nsec = (ns - req.tv_sec * 1000000000);
	nanosleep (&req, &rem);
}


#define InvalidHandle 0
#define THREAD_HANDLE pthread_t

typedef void *( * pthread_fn )( void * );

template
<
  typename Thread_T
>
class UThreadC
{
  private:
    struct Instance;

  public:
    typedef Thread_T              & Thread_R;
    typedef const Thread_T        & Thread_C_R;

    typedef THREAD_HANDLE Handle;
    typedef void ( *Handler)( Thread_R );

    virtual ~UThreadC() {}

  protected:
    UThreadC() {}

    virtual void ThreadMain( Thread_R ) = 0;

    static void Exit()
      { pthread_exit(0); }
#ifndef ANDROID
    static void TestCancel()
      { pthread_testcancel(); }
#endif

    static Handle Self()
      { return (Handle)pthread_self(); }

  public:

    static int Create(
      const Handler       & Function,
      Thread_C_R            Param,
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,
      const bool          & CancelAsync     = false
    )
    {
      M_Create().lock();
      pthread_attr_t attr;
      pthread_attr_init(&attr);

      if ( CreateDetached )
        pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

      if ( StackSize )
        pthread_attr_setstacksize(&attr,StackSize);

      Instance I(Param,0,Function,CancelEnable,CancelAsync);

      Handle h=InvalidHandle;
      int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

      pthread_attr_destroy(&attr);

      if(H) *H = h;
      if ( !R ) S_Create().acquire();

	  M_Create().unlock();
	  return R;
    }

    int Create(
      Thread_C_R            Param,
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,
      const bool          & CancelAsync     = false
    ) const
    {
      M_Create().lock();
      pthread_attr_t attr;
      pthread_attr_init(&attr);

      if ( CreateDetached )
        pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

      if ( StackSize )
        pthread_attr_setstacksize(&attr,StackSize);

      Instance I(Param,const_cast<UThreadC *>(this),0,CancelEnable,CancelAsync);

      Handle h=InvalidHandle;
      int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

      pthread_attr_destroy(&attr);

      if(H) *H = h;
      if ( !R ) S_Create().acquire();

	  M_Create().unlock();
	  return R;
    }

    static int Join( Handle H )
      { return pthread_join(H,0); }

#ifndef ANDROID
    static int Kill( Handle H )
      { return pthread_cancel(H); }
#endif

    static int Detach( Handle H )
      { return pthread_detach(H); }

  private:

    static const UMutex &M_Create()      { static UMutex M; return M; }
    static USemaphore &S_Create()  { static USemaphore S; return S; }

    static void *ThreadMainHandler( Instance *Param )
    {
      Instance  I(*Param);
      Thread_T  Data(I.Data);
      S_Create().release();

#ifndef ANDROID
      if ( I.Flags & 1 /*CancelEnable*/ )
      {
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);

        if ( I.Flags & 2 /*CancelAsync*/ )
          pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
        else
          pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
      }
      else
      {
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);
      }
#endif

      if ( I.Owner )
        I.Owner->ThreadMain(Data);
      else
        I.pFN(Data);

      return 0;
    }

    struct Instance
    {
      Instance( Thread_C_R P, UThreadC<Thread_T> *const &O, const UThreadC<Thread_T>::Handler &pH = 0, const bool &CE=false, const bool &CA=false )
        : Data(P), Owner(O), pFN(pH), Flags(0) { if ( CE ) Flags|=1; if ( CA ) Flags|=2; }

      Thread_C_R      Data;
      UThreadC<Thread_T>                * Owner;
      Handler         pFN;
      unsigned char                     Flags;
    };
};

/////////////////////////////////////////////////////////////////////
//  Explicit specialization, no thread parameters
//
template<>
class UThreadC<void>
{
  private:
    struct Instance;

  public:
    typedef THREAD_HANDLE Handle;
    typedef void ( *Handler)();

    virtual ~UThreadC<void>() {}

  protected:
    UThreadC<void>() {}

    virtual void ThreadMain() = 0;

    static void Exit()
      { pthread_exit(0); }

#ifndef ANDROID
    static void TestCancel()
      { pthread_testcancel(); }
#endif

    static Handle Self()
      { return (Handle)pthread_self(); }

  public:

    static int Create(
      const Handler       & Function,
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,
      const bool          & CancelAsync     = false
    )
    {
      M_Create().lock();
      pthread_attr_t attr;
      pthread_attr_init(&attr);

      if ( CreateDetached )
        pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

      if ( StackSize )
        pthread_attr_setstacksize(&attr,StackSize);

      Instance I(0,Function,CancelEnable,CancelAsync);

      Handle h=InvalidHandle;
      int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

      pthread_attr_destroy(&attr);

      if(H) *H = h;
      if ( !R ) S_Create().acquire();

	  M_Create().unlock();
	  return R;
    }

    int Create(
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,
      const bool          & CancelAsync     = false
    ) const
    {
      M_Create().lock();
      pthread_attr_t attr;
      pthread_attr_init(&attr);

      if ( CreateDetached )
        pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

      if ( StackSize )
        pthread_attr_setstacksize(&attr,StackSize);

      Instance I(const_cast<UThreadC *>(this),0,CancelEnable,CancelAsync);

      Handle h=InvalidHandle;
      int R = pthread_create((pthread_t *)&h,&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

      pthread_attr_destroy(&attr);

      if(H) *H = h;
      if ( !R ) S_Create().acquire();

	  M_Create().unlock();
	  return R;
    }

    int Create(
      unsigned long        & ThreadId,
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,
      const bool          & CancelAsync     = false
    ) const
    {
      M_Create().lock();
      pthread_attr_t attr;
      pthread_attr_init(&attr);

      if ( CreateDetached )
        pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);

      if ( StackSize )
        pthread_attr_setstacksize(&attr,StackSize);

      Instance I(const_cast<UThreadC *>(this),0,CancelEnable,CancelAsync);

      *H = InvalidHandle;
      int R = pthread_create((pthread_t *)&(*H),&attr,(pthread_fn)ThreadMainHandler,(void *)&I);

      ThreadId = (unsigned long)*H;

      pthread_attr_destroy(&attr);

      if ( !R ) S_Create().acquire();

      M_Create().unlock();
      return R;
    }

    static int Join( Handle H )
      { return pthread_join(H,0); }

#ifndef ANDROID
    static int Kill( Handle H )
      { return pthread_cancel(H); }
#endif

    static int Detach( Handle H )
      { return pthread_detach(H); }

  private:

    static const UMutex &M_Create()      { static UMutex M; return M; }
    static USemaphore &S_Create()  { static USemaphore S; return S; }

    static void *ThreadMainHandler( Instance *Param )
    {
      Instance  I(*Param);
      S_Create().release();

#ifndef ANDROID
      if ( I.Flags & 1 /*CancelEnable*/ )
      {
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);

        if ( I.Flags & 2 /*CancelAsync*/ )
          pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
        else
          pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
      }
      else
      {
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);
      }
#endif

      if ( I.Owner )
        I.Owner->ThreadMain();
      else
        I.pFN();

      return 0;
    }

    struct Instance
    {
		Instance( UThreadC<void> *const &O, const UThreadC<void>::Handler &pH = 0, const bool &CE=false, const bool &CA=false )
        : pFN(pH), Owner(O), Flags(0) { if ( CE ) Flags|=1; if ( CA ) Flags|=2; }

		UThreadC<void>::Handler             pFN;
		UThreadC<void>                *     Owner;
		unsigned char                     Flags;

    };
};

#endif // !_U_Thread_Posix_
