//----------------------------------------------------------------------------------------------------------------------
// GRVC Quadrotor HAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef _GRVCQUADROTOR_UAVSERVER_SERVICE_SERVICE_H_
#define _GRVCQUADROTOR_UAVSERVER_SERVICE_SERVICE_H_

#include <uav_common/types.h>
#include <functional>
#include <string>
#include <vector>

namespace grvc { namespace uav {
	
	/// Common interface for classes advertising the uav service over different channels

	/// Whoever uses this class is responsible for registering callbacks for every event they want to be notified of.
	/// New callbacks override older ones, so only the most recent one will be invoked for any event. If you require
	/// Multiple callbacks to be invoked, you will need to do the dispatching for yourself.
	class Service {
	public:
		// Generic callbacks for uav events
		typedef std::function<void (conststd::vector<Vec3>&)>	TrackPathCb;
		typedef std::function<void (double)>					TakeOffCb;
		typedef std::function<void ()>							LandCb;
		typedef std::function<void ()>							AbortCb;

		// Set callbacks
		virtual void onTrackPath	(TrackPathCb) = 0;	///< Set callback for TrackPath commands
		virtual void onTakeOff		(TakeOffCb) = 0;	///< Set callback for TakeOff commands
		virtual void onLand			(LandCb) = 0;		///< Set callback for Land commands
		virtual void onAbort		(AbortCb) = 0;		///< Set callback for command abortions

		// Direct interface
		virtual void publishPosition	(const Vec3&) = 0; ///< Publish latest position estimate

		virtual ~Service() = default; // Ensure proper destructor calling for derived classes

		/// \brief Create and start an adequate uav::Service depending on current platform and command arguments.
		/// \param _argc number of arguments in _argv
		/// \param _argv command line arguments passed to the program. This arguments will be parsed
		/// and used to select the best fitting implementation of Service from those available in the
		/// current platform.
		/// \return the newly created Service. Whoever calls this method, is responsible for eventually
		/// destroying the Service.
		/// \remark This is the recommended method for creating Services, since it abstracts from the
		/// underlying (platform-dependent) implementation details.
		static Service* createService(int _argc, char** _argv);
	};
	
}}	// namespace grvc::uav

#endif // _GRVCQUADROTOR_UAVSERVER_SERVICE_SERVICE_H_