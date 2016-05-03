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
#ifndef _GRVCQUADROTOR_UAVCLIENT_CLIENT_H_
#define _GRVCQUADROTOR_UAVCLIENT_CLIENT_H_

#include <uav_common/types.h>
#include <string>
#include <vector>

namespace grvc { namespace uav {
	
	/// \brief Cient interface for other modules to communicate with a uav (or uav-derived) node

	/// Different implementations of this interface are responsible of communications with an analogous uav service.
	/// For example, uav::ClientRos (derived from uav::Client), implements communications using topics and services
	/// and expects a uav::SericeRos to be running in some other node in the network. Equivalent, uav::ClientHttp
	/// would require uav::ServiceHttp to be running somewhere, with an accessible IP address.
	class Client {
	public:
		/// Follow the reqyested path, one waypoint at a time
		///
		/// If the UAV is landed, a take off will be automatically performed before actual path tracking can take place
		/// If a is already being tracked, it will be cancelled and substituted for the new path.
		/// \param _path an ordered waypoint list to follow.
		virtual void trackPath(const std::vector<Vec3>& _path) = 0;
		/// Perform a take off maneuver and hover until new commands arrive.
		/// \param _height targer height that must be reached to consider the take off complete.
		virtual void takeOff (double _height) = 0;
		/// Land on the current position. If a previous command was running. It is instantly aborted and landing takes
		/// place in the current location.
		virtual void land() = 0;
		/// Cancel execution of the current task
		virtual void		abortTask		() = 0;
		/// Latest position estimation of the robot
		virtual Vec3		position		() const = 0;

		virtual ~Client() = default; // Ensure proper destructor calling for derived classes

		/// \brief Create an adequate uav::Client depending on current platform and command arguments.
		/// \param _args command line arguments passed to the program. This arguments will be parsed
		/// and used to select the best fitting implementation of Client from those available in the
		/// current platform.
		/// \return the newly created Client. Whoever calls this method, is responsible for eventually
		/// destroying the Client.
		/// \remark This is the recommended method for creating Clients, since it abstracts from the
		/// underlying (platform-dependent) implementation details.
		static Client* createClient(const std::vector<std::string>& _args);
	};
	
}}	// namespace grvc::uav

#endif // _GRVCQUADROTOR_UAVCLIENT_CLIENT_H_