#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>

#include <string>
#include <vector>

#include <functional>
#include <fcntl.h>
#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
  #include <BaseTsd.h>
  typedef SSIZE_T ssize_t;
#endif

namespace gazebo {
	
struct ServoPacket
{
	bool armed = 0;
	/// \brief Motor speed data.
	/// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
	float servo_command[16] = {0.0f};
};

struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
};

class ArduPilotSocket{

  /// \brief constructor
  public: ArduPilotSocket()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
  }

  /// \brief destructor
  public: ~ArduPilotSocket()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->fd, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->fd, _buf, _size, 0);
    #endif
  }

  /// \brief Socket handle
  private: int fd;
  
};

class BalanceBotplugin : public ModelPlugin 
{
	public:
	
	BalanceBotplugin()
	{
		this->armed=0;
		this->arduPilotOnline = 0;
		this->connectionTimeoutCount = 0;
		this->connectionTimeoutMaxCount = 10;
		this->modelXYZToAirplaneXForwardZDown = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
		this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
	}
	
	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		this->model = _model;
		loadComponents();
		this->model->SetGravityMode(false);
		this->modelName = this->model->GetName();
		
		this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor("imu_sensor"));
			
		if ( !initSockets("127.0.0.1", 9002, "127.0.0.1", 9003) )
		{
			return;
		}
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BalanceBotplugin::OnUpdate, this));
	}
	
	void OnUpdate()
	{
		this->readInputs(this->pkt);
			
		if (this->arduPilotOnline) 
		{
			if ((!this->armed) && pkt.armed)
			{
				this->armed = true;
				this->model->SetGravityMode(true);
			} else if (!pkt.armed) 
			{
				this->armed = false;
			}
			this->applyMotorTorques();
			this->sendState();
		}
	}
	
	private:
	
	void loadComponents()
	{
		body = model->GetLink("body");
		left_wheel = model->GetLink("left wheel");
		right_wheel = model->GetLink("right wheel");
		left_wheel_joint = model->GetJoint("left wheel axis");
		right_wheel_joint = model->GetJoint("right wheel axis");
	}
	
	bool initSockets(std::string listen_addr, const uint16_t fdm_port_in, std::string fdm_addr, const uint16_t fdm_port_out)
	{
	  if (!this->socket_in.Bind(listen_addr.c_str(), fdm_port_in))
	  {
		gzerr << "[" << this->modelName << "] "
			  << "failed to bind with " << listen_addr
			  << ":" << fdm_port_in << " aborting plugin.\n";
		return false;
	  }

	  if (!this->socket_out.Connect(fdm_addr.c_str(), fdm_port_out))
	  {
		gzerr << "[" << this->modelName << "] "
			  << "failed to bind with " << fdm_addr
			  << ":" << fdm_port_out << " aborting plugin.\n";
		return false;
	  }

	  return true;
	}
	
	void readInputs(ServoPacket &pkt)
	{
		uint32_t waitMs;
		if (this->arduPilotOnline)
		{
			// increase timeout for receive once we detect a packet from
			// ArduPilot FCS.
			waitMs = 1000;
		}
		else
		{
			// Otherwise skip quickly and do not set control force.
			waitMs = 1;
		}
		ssize_t recvSize =
		this->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

		// Drain the socket in the case we're backed up
		int counter = 0;
		ServoPacket last_pkt;
		while (true)
		{
			// last_pkt = pkt;
			const ssize_t recvSize_last =
			  this->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
			if (recvSize_last == -1)
			{
			  break;
			}
			counter++;
			pkt = last_pkt;
			recvSize = recvSize_last;
		}
		if (counter > 0)
		{
			gzdbg << "[" << this->modelName << "] "
				  << "Drained n packets: " << counter << std::endl;
		}

		if (recvSize == -1)
		{
			// didn't receive a packet
			// gzdbg << "no packet\n";
			gazebo::common::Time::NSleep(100);
			if (this->arduPilotOnline)
			{
			  gzwarn << "[" << this->modelName << "] "
					 << "Broken ArduPilot connection, count ["
					 << this->connectionTimeoutCount
					 << "/" << this->connectionTimeoutMaxCount
					 << "]\n";
			  if (++this->connectionTimeoutCount >
				this->connectionTimeoutMaxCount)
			  {
				this->connectionTimeoutCount = 0;
				this->arduPilotOnline = false;
				gzwarn << "[" << this->modelName << "] "
					   << "Broken ArduPilot connection, resetting motor control.\n";
			  }
			}
		}
		else
		{
			const ssize_t expectedPktSize = sizeof(ServoPacket);
			if (recvSize < expectedPktSize)
			{
			  gzerr << "[" << this->modelName << "] "
					<< "got less than model needs. Got: " << recvSize
					<< "commands, expected size: " << expectedPktSize << "\n";
			}
			
			if (!this->arduPilotOnline)
			{
			  gzdbg << "[" << this->modelName << "] "
					<< "ArduPilot controller online detected.\n";
			  
			  // made connection, set some flags
			  this->connectionTimeoutCount = 0;
			  this->arduPilotOnline = true;
			}
		}
	}
	
	void applyMotorTorques()
	{
		float w_l = -left_wheel->GetRelativeAngularVel().z;
		float w_r = right_wheel->GetRelativeAngularVel().z;
		gzdbg<<"torque_l:"<<w_l<<" torque_r:"<<w_r<<std::endl;
		
		if(this->armed){
					
			float max_voltage = 12.0f;
			float input_left =  (pkt.servo_command[0]-0.5f) * 2.0f;
			float input_right = (pkt.servo_command[2]-0.5f) * 2.0f;
			float k = 0.15f;
			float R = 3.0f;
			double _maxTorque = 0.128;
			
			//double angular_p = 100.0;
			//double angular_i = 0.0;
			//double angular_d = 0.0;
			//double angular_imax = 123456789.0;
			
			//common::PID left_wheel_PID(angular_p, angular_i, angular_d,angular_imax, -angular_imax, _maxTorque, -_maxTorque);
			//common::PID controller_rotation(angular_p, angular_i, angular_d,angular_imax, -angular_imax, _maxTorque, -_maxTorque);
			
			//left_wheel_joint->SetParam("fmax", 0, 100.0);
			//left_wheel_joint->SetParam("vel", 0, (double)input_left);
			
			//right_wheel_joint->SetParam("fmax", 0, 100.0);
			//right_wheel_joint->SetParam("vel", 0, (double)input_right);
			
			
			float torque_left = ( (k * input_left * max_voltage) - (k * k * w_l) )/R;
			float torque_right = ( (k * input_right * max_voltage) - (k * k * w_r) )/R;
			
			
			left_wheel_joint->SetForce(0, torque_left);
			right_wheel_joint->SetForce(0,torque_right);
		}		
	}
	
	void sendState() 
	{
		 // send_fdm
		fdmPacket pkt;

		pkt.timestamp = this->model->GetWorld()->GetSimTime().Double();

		// asssumed that the imu orientation is:
		//   x forward
		//   y right
		//   z down

		// get linear acceleration in body frame
		const ignition::math::Vector3d linearAccel =
		this->imuSensor->LinearAcceleration();

		// copy to pkt
		pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
		pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
		pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
		// gzerr << "lin accel [" << linearAccel << "]\n";

		// get angular velocity in body frame
		const ignition::math::Vector3d angularVel =
		this->imuSensor->AngularVelocity();

		// copy to pkt
		pkt.imuAngularVelocityRPY[0] = angularVel.X();
		pkt.imuAngularVelocityRPY[1] = angularVel.Y();
		pkt.imuAngularVelocityRPY[2] = angularVel.Z();
		
		
		// get inertial pose and velocity
		// position of the uav in world frame
		// this position is used to calcualte bearing and distance
		// from starting location, then use that to update gps position.
		// The algorithm looks something like below (from ardupilot helper
		// libraries):
		//   bearing = to_degrees(atan2(position.y, position.x));
		//   distance = math.sqrt(self.position.x**2 + self.position.y**2)
		//   (self.latitude, self.longitude) = util.gps_newpos(
		//    self.home_latitude, self.home_longitude, bearing, distance)
		// where xyz is in the NED directions.
		// Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
		// around.
		// orientation of the uav in world NED frame -
		// assuming the world NED frame has xyz mapped to NED,
		// imuLink is NED - z down

		// model world pose brings us to model,
		// which for example zephyr has -y-forward, x-left, z-up
		// adding modelXYZToAirplaneXForwardZDown rotates
		//   from: model XYZ
		//   to: airplane x-forward, y-left, z-down
		const ignition::math::Pose3d gazeboXYZToModelXForwardZDown =
		this->modelXYZToAirplaneXForwardZDown +
		this->model->GetWorldPose().Ign();

		// get transform from world NED to Model frame
		const ignition::math::Pose3d NEDToModelXForwardZUp =
		gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

		// N
		pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

		// E
		pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

		// D
		pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

		// imuOrientationQuat is the rotation from world NED frame
		// to the uav frame.
		pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
		pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
		pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
		pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

		// Get NED velocity in body frame *
		// or...
		// Get model velocity in NED frame
		const ignition::math::Vector3d velGazeboWorldFrame =
		this->model->GetLink()->GetWorldLinearVel().Ign();
		const ignition::math::Vector3d velNEDFrame =
		this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
		pkt.velocityXYZ[0] = velNEDFrame.X();
		pkt.velocityXYZ[1] = velNEDFrame.Y();
		pkt.velocityXYZ[2] = velNEDFrame.Z();
		
		this->socket_out.Send(&pkt, sizeof(pkt));
	}
	
	event::ConnectionPtr updateConnection;
	
	std::string modelName;
	
	physics::ModelPtr model;
	physics::LinkPtr body, left_wheel, right_wheel;
	physics::JointPtr left_wheel_joint, right_wheel_joint;
	
	sensors::ImuSensorPtr imuSensor;
	
	/// \brief transform from model orientation to x-forward and z-down
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief transform from world frame to NED frame
    ignition::math::Pose3d gazeboXYZToNED;
	
	ServoPacket pkt;
	
	ArduPilotSocket socket_in, socket_out;
	
	bool armed;
	bool arduPilotOnline;
	int connectionTimeoutCount;
	int connectionTimeoutMaxCount;
};
GZ_REGISTER_MODEL_PLUGIN(BalanceBotplugin)
}
