#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "Rewarders\GeneralRewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\SpeedRewarder.h"
#include <time.h>
#include <Eigen/Core>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>

static Eigen::Vector3f get_dir_from_2d(const int x, const int y, const double res_x, const double res_y, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, bool draw_debug);
static Eigen::Vector3f get_coords_in_cam_coord_sys(float x, float y, float z, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view);

int res_x = 1920;
int res_y = 1200;
std::vector<int> vec;
std::vector<int> vec_h;

std::ofstream debugfile;


void Scenario::start(const Value& sc, const Value& dc) {
	if (running) return;
	GAMEPLAY::SET_TIME_SCALE(0.4);
	debugfile.open("debug.txt");
	const char* weatherList[] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
	const char* vehicleList[] = { "blista", "voltic"};
	//const char* vehicleList[] = { "blista", "voltic", "packer" };
	float x, y, heading;
	int hour, minute, width, height;
	const char* _weather;
	const char* _vehicle;

	Vector3 pos, rotation;
	Hash vehicleHash;

	//Parse options
	srand(std::time(NULL));
	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));

	if (dc["rate"].IsNull()) rate = 10;
	else rate = dc["rate"].GetInt();
	if (dc["frame"].IsNull()) {
		width = 320; height = 160;
	}
	else {
		if (dc["frame"][0].IsNull()) width = 320;
		else width = dc["frame"][0].GetInt();
		if (dc["frame"][1].IsNull()) height = 160;
		else height = dc["frame"][1].GetInt();
	}
	if (dc["vehicles"].IsNull()) vehicles = false;
	else vehicles = dc["vehicles"].GetBool();
	if (dc["peds"].IsNull()) peds = false;
	else peds = dc["peds"].GetBool();
	if (dc["trafficSigns"].IsNull()) trafficSigns = false;
	else trafficSigns = dc["trafficSigns"].GetBool();
	if (dc["direction"].IsNull()) direction = false;
	else {
		direction = true;
		if (dc["direction"][0].IsNull()) direction = false;
		else dir.x = dc["direction"][0].GetFloat();
		if (dc["direction"][1].IsNull()) direction = false;
		else dir.y = dc["direction"][1].GetFloat();
		if (dc["direction"][2].IsNull()) direction = false;
		else dir.z = dc["direction"][2].GetFloat();
	}
	if (dc["reward"].IsNull()) reward = false;
	else {
		reward = false;
		if (dc["reward"].IsArray()) {
			if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
				delete(rewarder);
				rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
				reward = true;
			}
		}
	}
	if (dc["throttle"].IsNull()) throttle = false;
	else throttle = dc["throttle"].GetBool();
	if (dc["brake"].IsNull()) brake = false;
	else brake = dc["brake"].GetBool();
	if (dc["steering"].IsNull()) steering = false;
	else steering = dc["steering"].GetBool();
	if (dc["speed"].IsNull()) speed = false;
	else speed = dc["speed"].GetBool();
	if (dc["yawRate"].IsNull()) yawRate = false;
	else yawRate = dc["yawRate"].GetBool();
	if (dc["drivingMode"].IsNull()) drivingMode = false;
	else drivingMode = dc["drivingMode"].GetBool();
	if (dc["location"].IsNull()) location = false;
	else location = dc["location"].GetBool();
	if (dc["time"].IsNull()) time = false;
	else time = dc["time"].GetBool();
    if(dc["bbox"].IsNull()) bbox = true;
    else bbox = dc["bbox"].GetBool();
    if(dc["drawDebug"].IsNull()) drawDebug = false;
    else drawDebug = dc["drawDebug"].GetBool();


	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

	if (location.IsNull()) {
		x = -3400 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 10000));
		y = -3600 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 10600));
	}
	else {
		if (location[0].IsNull()) x = -3400 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 10000));
		else x = location[0].GetFloat();
		if (location[1].IsNull()) y = -3600 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 10600));
		else y = location[1].GetFloat();
	}

	if (time.IsNull()) {
		hour = rand() % 24;
		minute = rand() % 60;
	}
	else {
		if (time[0].IsNull()) hour = rand() % 24;
		else hour = time[0].GetInt();
		if (time[1].IsNull()) minute = rand() % 60;
		else minute = time[1].GetInt();
	}

	if (weather.IsNull()) _weather = weatherList[rand() % 14];
	else _weather = weather.GetString();
	_weather = weatherList[1];

	if (vehicle.IsNull()) _vehicle = vehicleList[rand() % 3];
	else _vehicle = vehicle.GetString();

	if (drivingMode.IsNull()) {
		_drivingMode = rand() % 4294967296;
		_setSpeed = 1.0*(rand() % 20);
	}
	else {
		if (drivingMode.IsArray()) {
			if (drivingMode[0].IsNull()) _drivingMode = rand() % 4294967296;
			else _drivingMode = drivingMode[0].GetInt();
			if (drivingMode[1].IsNull()) _setSpeed = 1.0*(rand() % 20);
			else _setSpeed = drivingMode[1].GetFloat();
		}
		else _drivingMode = -1;
	}

	//Build scenario
	while (!PATHFIND::LOAD_ALL_PATH_NODES(TRUE)) WAIT(0);
	PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 300, 300);
	PATHFIND::LOAD_ALL_PATH_NODES(FALSE);

	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	while (!ENTITY::DOES_ENTITY_EXIST(this->vehicle)) {
		this->vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
		WAIT(0);
	}
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(this->vehicle);

	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
		WAIT(0);
	}

	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	PED::SET_PED_INTO_VEHICLE(ped, this->vehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	rotation = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, this->vehicle, 0, 2.35, 2.0, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(camera, this->vehicle, 0, 1.4, 1.8, TRUE);
	CAM::SET_CAM_FOV(camera, 64);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x - 10, rotation.y, rotation.z, 1);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	debugfile << "the ratio is : " << GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
	debugfile << "the fov is : " << CAM::GET_CAM_FOV(camera);

	float fov = CAM::GET_CAM_FOV(camera);
	float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);

	auto near_clip_height = near_clip * tan(fov / 2. * (3.14159 / 180.)); // field of view is returned vertically
	auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
	debugfile << "the near_clip_height is : " << near_clip * tan(fov / 2. * (3.14159 / 180.));
	debugfile << "the near_clip_width is : " << near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
	debugfile.close();
	float fov_unit = 26.9 / 63.0;
	/*
	for (int i = 0; i < 32; i++) {
		auto height_from_center = near_clip * tan((i + 0.5) * fov_unit * (3.14159 / 180.));
		int diff = res_y * (height_from_center / (2 * near_clip_height));
		int pix_num1 = res_y / 2 - diff;
		int pix_num2 = res_y / 2 + diff;
		vec.push_back(pix_num1);
		vec.push_back(pix_num2);
	}*/

	for (int i = 31; i >= 0; i--) {
		auto height_from_center = near_clip * tan((i + 0.5) * fov_unit * (3.14159 / 180.));
		int diff = res_y * (height_from_center / (2 * near_clip_height));
		int pix_num1 = res_y / 2 - diff;
		vec.push_back(pix_num1);
	}

	for (int i = 0; i < 32; i++) {
		auto height_from_center = near_clip * tan((i + 0.5) * fov_unit * (3.14159 / 180.));
		int diff = res_y * (height_from_center / (2 * near_clip_height));
		int pix_num1 = res_y / 2 + diff;
		vec.push_back(pix_num1);
	}

	float dov_unit = 90. / 512.;


	/*
	for (int i = 0; i < 256; i++) {
		auto width_from_center = near_clip * tan((i + 0.5) * dov_unit * (3.14159 / 180.));
		int diff = res_x * (width_from_center / (2 * near_clip_width));
		int pix_num1 = res_x / 2 - diff;
		int pix_num2 = res_x / 2 + diff;
		vec_h.push_back(pix_num1);
		vec_h.push_back(pix_num2);
	}*/

	for (int i = 255; i >= 0; i--) {
		auto width_from_center = near_clip * tan((i + 0.5) * dov_unit * (3.14159 / 180.));
		int diff = res_x * (width_from_center / (2 * near_clip_width));
		int pix_num1 = res_x / 2 - diff;

		vec_h.push_back(pix_num1);
	}

	for (int i = 0; i <= 255; i++) {
		auto width_from_center = near_clip * tan((i + 0.5) * dov_unit * (3.14159 / 180.));
		int diff = res_x * (width_from_center / (2 * near_clip_width));
		int pix_num1 = res_x / 2 + diff;
		vec_h.push_back(pix_num1);
	}

	auto fov_h = 2.0 * atan(near_clip_width / near_clip);

	int num_h = 2000 * fov_h / (2 * 3.1415926);
	int num_v = 64;
	int step_x = res_x / num_h;
	int step_y = res_y / num_v;

	AI::CLEAR_PED_TASKS(ped);
	if (_drivingMode >= 0) AI::TASK_VEHICLE_DRIVE_WANDER(ped, this->vehicle, _setSpeed, _drivingMode);

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

d.AddMember("car_points", a, allocator);
	if (vehicles) d.AddMember("vehicles", a, allocator);
	if (peds) d.AddMember("peds", a, allocator);
	if (trafficSigns) d.AddMember("trafficSigns", a, allocator);
	if (direction) d.AddMember("direction", a, allocator);
	if (reward) d.AddMember("reward", 0.0, allocator);
	if (throttle) d.AddMember("throttle", 0.0, allocator);
	if (brake) d.AddMember("brake", 0.0, allocator);
	if (steering) d.AddMember("steering", 0.0, allocator);
	if (speed) d.AddMember("speed", 0.0, allocator);
	if (yawRate) d.AddMember("yawRate", 0.0, allocator);
	if (this->drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (this->location) d.AddMember("location", a, allocator);
	if (this->time) d.AddMember("time", 0, allocator);

	delete(screenCapturer);
	screenCapturer = new ScreenCapturer(width, height);

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;

	const char* weatherList[] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
	const char* vehicleList[] = { "blista", "voltic", "packer" };
	float x, y, heading;
	int hour, minute, _drivingMode, width, height;
	const char* _weather;
	const char* _vehicle;

	Vector3 pos, rotation;
	Hash vehicleHash;

	//Parse options
	srand(std::time(NULL));
	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	GAMEPLAY::SET_TIME_SCALE(0.4);

	if (!dc["rate"].IsNull()) rate = dc["rate"].GetInt();
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) width = dc["frame"][0].GetInt(); 
		else width = screenCapturer->imageWidth;
		if (!dc["frame"][0].IsNull()) height = dc["frame"][1].GetInt();
		else height = screenCapturer->imageHeight;
		
		delete(screenCapturer);
		screenCapturer = new ScreenCapturer(width, height);
	}
	if (!dc["vehicles"].IsNull()) vehicles = dc["vehicles"].GetBool();
	if (!dc["peds"].IsNull()) peds = dc["peds"].GetBool();
	if (!dc["trafficSigns"].IsNull()) trafficSigns = dc["trafficSigns"].GetBool();
	if (!dc["directions"].IsNull()) {
		if (!dc["direction"][0].IsNull()) dir.x = dc["direction"][0].GetFloat();
		if (!dc["direction"][1].IsNull()) dir.y = dc["direction"][1].GetFloat();
		if (!dc["direction"][2].IsNull()) dir.z = dc["direction"][2].GetFloat();
	}
	if (dc["reward"].IsArray()) {
		reward = false;
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			delete(rewarder);
			rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
			reward = true;
		}
	} 
	if (!dc["throttle"].IsNull()) throttle = dc["throttle"].GetBool();
	if (!dc["brake"].IsNull()) brake = dc["brake"].GetBool();
	if (!dc["steering"].IsNull()) steering = dc["steering"].GetBool();
	if (!dc["speed"].IsNull()) speed = dc["speed"].GetBool();
	if (!dc["yawRate"].IsNull()) yawRate = dc["yawRate"].GetBool();
	if (!dc["drivingMode"].IsNull()) drivingMode = dc["drivingMode"].GetBool();
	if (!dc["location"].IsNull()) location = dc["location"].GetBool();
	if (!dc["time"].IsNull())  time = dc["time"].GetBool();
    if (!dc["bbox"].IsNull())  bbox = dc["bbox"].GetBool(); else bbox = true;
    if (!dc["drawDebug"].IsNull()) drawDebug = dc["drawDebug"].GetBool(); else drawDebug = false;
	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

d.AddMember("car_points", a, allocator);
	if (vehicles) d.AddMember("vehicles", a, allocator);
	if (peds) d.AddMember("peds", a, allocator);
	if (trafficSigns) d.AddMember("trafficSigns", a, allocator);
	if (direction) d.AddMember("direction", a, allocator);
	if (reward) d.AddMember("reward", 0.0, allocator);
	if (throttle) d.AddMember("throttle", 0.0, allocator);
	if (brake) d.AddMember("brake", 0.0, allocator);
	if (steering) d.AddMember("steering", 0.0, allocator);
	if (speed) d.AddMember("speed", 0.0, allocator);
	if (yawRate) d.AddMember("yawRate", 0.0, allocator);
	if (this->drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (this->location) d.AddMember("location", a, allocator);
	if (this->time) d.AddMember("time", 0, allocator);

	if (!vehicle.IsNull()) {
		_vehicle = vehicle.GetString();
		vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);

		pos = ENTITY::GET_ENTITY_COORDS(ped, 1);
		heading = ENTITY::GET_ENTITY_HEADING(this->vehicle);
		VEHICLE::DELETE_VEHICLE(&(this->vehicle));
		
		STREAMING::REQUEST_MODEL(vehicleHash);
		while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
		while (!ENTITY::DOES_ENTITY_EXIST(this->vehicle)) {
			this->vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
			WAIT(0);
		}
		VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(this->vehicle);

		PED::SET_PED_INTO_VEHICLE(ped, this->vehicle, -1);

		rotation = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
		CAM::DESTROY_ALL_CAMS(TRUE);
		camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
		if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, this->vehicle, 0, 2.35, 1.7, TRUE);
		else CAM::ATTACH_CAM_TO_ENTITY(camera, this->vehicle, 0, 0.5, 0.8, TRUE);
		CAM::SET_CAM_FOV(camera, 64);
		CAM::SET_CAM_ACTIVE(camera, TRUE);
		CAM::SET_CAM_ROT(camera, rotation.x - 10, rotation.y, rotation.z, 1);
		CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
		CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);
	}

	if (!location.IsNull()) {
		if (!location[0].IsNull()) x = location[0].GetFloat();
		if (!location[1].IsNull()) y = location[1].GetFloat();

		while (!PATHFIND::LOAD_ALL_PATH_NODES(TRUE)) WAIT(0);
		PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 300, 300);
		PATHFIND::LOAD_ALL_PATH_NODES(FALSE);

		PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
		while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

		ENTITY::SET_ENTITY_QUATERNION(this->vehicle, pos.x, pos.y, pos.z, heading);
		VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(this->vehicle);

		PED::SET_PED_INTO_VEHICLE(ped, this->vehicle, -1);

		rotation = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
		CAM::DESTROY_ALL_CAMS(TRUE);
		camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
		CAM::ATTACH_CAM_TO_ENTITY(camera, this->vehicle, 0, 0.5, 0.8, TRUE);
		CAM::SET_CAM_FOV(camera, 26.9);
		CAM::SET_CAM_ACTIVE(camera, TRUE);
		CAM::SET_CAM_ROT(camera, rotation.x - 10, rotation.y, rotation.z, 1);
		CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
		CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);
	}

	if (!time.IsNull()) {
		if (!time[0].IsNull()) hour = time[0].GetInt();
		else hour = TIME::GET_CLOCK_HOURS();
		if (!time[1].IsNull()) minute = time[1].GetInt();
		else minute = TIME::GET_CLOCK_MINUTES();

		TIME::SET_CLOCK_TIME(hour, minute, 0);
	}

	if (!weather.IsNull()) {
		_weather = weather.GetString();
		GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);
	}

	if (!drivingMode.IsNull()) {
		if (drivingMode.IsArray()) {
			if (!drivingMode[0].IsNull()) _drivingMode = drivingMode[0].GetInt();
			if (!drivingMode[1].IsNull()) _setSpeed = drivingMode[1].GetFloat();
		}
		else _drivingMode = -1;
		
		AI::CLEAR_PED_TASKS(ped);
		if (_drivingMode >= 0) AI::TASK_VEHICLE_DRIVE_WANDER(ped, this->vehicle, _setSpeed, _drivingMode);
	}
	
}

void Scenario::run() {
	if (running) {
		std::clock_t now = std::clock();

		Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
		CAM::SET_CAM_ROT(camera, rotation.x - 10, rotation.y, rotation.z, 1);

		if (_drivingMode < 0) {
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		
		float delay = ((float)(now - lastSafetyCheck)) / CLOCKS_PER_SEC;
		if (delay > 10) {
			lastSafetyCheck = std::clock();
			//Avoid bad things such as getting killed by the police, robbed, dying in car accidents or other horrible stuff
			PLAYER::SET_EVERYONE_IGNORE_PLAYER(player, TRUE);
			PLAYER::SET_POLICE_IGNORE_PLAYER(player, TRUE);
			PLAYER::CLEAR_PLAYER_WANTED_LEVEL(player); // Never wanted

			// Put on seat belt
			PED::SET_PED_CONFIG_FLAG(ped, 32, FALSE);

			// Invincible vehicle
			VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(vehicle, TRUE);

			VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(vehicle, FALSE);
			ENTITY::SET_ENTITY_INVINCIBLE(vehicle, TRUE);
			ENTITY::SET_ENTITY_PROOFS(vehicle, 1, 1, 1, 1, 1, 1, 1, 1);

			// Player invincible
			PLAYER::SET_PLAYER_INVINCIBLE(player, TRUE);

			// Driving characteristics
			PED::SET_DRIVER_AGGRESSIVENESS(ped, 0.0);
			PED::SET_DRIVER_ABILITY(ped, 100.0);
		}
	}
	scriptWait(0);
}

void Scenario::stop() {
	if (!running) return;
	running = false;
	CAM::DESTROY_ALL_CAMS(TRUE);
	CAM::RENDER_SCRIPT_CAMS(FALSE, TRUE, 500, FALSE, FALSE);
	AI::CLEAR_PED_TASKS(ped);
	setCommands(0.0, 0.0, 0.0);
}

void Scenario::setCommands(float throttle, float brake, float steering) {
	currentThrottle = throttle;
	currentBrake = brake;
	currentSteering = steering;
}

StringBuffer Scenario::generateMessage() {
	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);
	
	screenCapturer->capture();

	if (vehicles) setVehiclesList();
	if (peds) setPedsList();
	if (trafficSigns); //TODO
	if (direction) setDirection();
	if (reward) setReward();
	if (throttle) setThrottle();
	if (brake) setBrake();
	if (steering) setSteering();
	if (speed) setSpeed();
	if (yawRate) setYawRate();
	if (drivingMode); //TODO
	if (location) setLocation();
	if (time) setTime();

	d.Accept(writer);

	return buffer;
}
Eigen::Vector3f rotate(Eigen::Vector3f a, Eigen::Vector3f theta)
{
    Eigen::Vector3f d;

    d(0) = (float)cos((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) - (float)sin((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
    d(1) = (float)sin((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) + (float)cos((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
    d(2) = -(float)sin((double)theta(1))*a(0) + (float)cos((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2));

    return d;
}

static Eigen::Vector2f get_2d_from_3d(const Eigen::Vector3f& vertex, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, bool draw_debug = false){
    // Inspired by Artur Filopowicz: Video Games for Autonomous Driving: https://github.com/arturf1/GTA5-Scripts/blob/master/Annotator.cs#L379
	draw_debug = false;
    static const Eigen::Vector3f WORLD_NORTH(0.0, 1.0, 0.0);
    static const Eigen::Vector3f WORLD_UP(0.0, 0.0, 1.0);
    static const Eigen::Vector3f WORLD_EAST(1.0, 0.0, 0.0);
    Eigen::Vector3f theta = (3.14159 / 180.0) * cam_rotation;
    auto cam_dir = rotate(WORLD_NORTH, theta);
    if (draw_debug)
    {
        auto cam_dir_line_end = cam_dir + cam_coords;

        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_dir_line_end.x(), cam_dir_line_end.y(), cam_dir_line_end.z(), 0, 255, 0, 200);
    }
    auto clip_plane_center = cam_coords + cam_near_clip * cam_dir;
    auto camera_center = -cam_near_clip * cam_dir;
    auto near_clip_height = 2 * cam_near_clip * tan(cam_field_of_view / 2. * (3.14159 / 180.)); // field of view is returned vertically
    auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

    const Eigen::Vector3f cam_up = rotate(WORLD_UP, theta);
    const Eigen::Vector3f cam_east = rotate(WORLD_EAST, theta);
    const Eigen::Vector3f near_clip_to_target = vertex - clip_plane_center; // del

    Eigen::Vector3f camera_to_target = near_clip_to_target - camera_center; // Total distance - subtracting a negative to add clip distance

    if (draw_debug)
    {
        auto cam_up_end_line = cam_up + cam_coords;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_up_end_line.x(), cam_up_end_line.y(), cam_up_end_line.z(), 100, 100, 255, 200);
        auto cam_east_end_line = cam_up + cam_coords;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_east_end_line.x(), cam_east_end_line.y(), cam_east_end_line.z(), 100, 100, 255, 200);
        auto del_draw = cam_coords + near_clip_to_target;
        GRAPHICS::DRAW_LINE(clip_plane_center.x(), clip_plane_center.y(), clip_plane_center.z(), del_draw.x(), del_draw.y(), del_draw.z(), 255, 255, 100, 255);
        auto viewerDistDraw = cam_coords + camera_to_target;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), viewerDistDraw.x(), viewerDistDraw.y(), viewerDistDraw.z(), 255, 100, 100, 255);
    }
    Eigen::Vector3f camera_to_target_unit_vector = camera_to_target * (1. / camera_to_target.norm()); // Unit vector in direction of plane / line intersection

    double view_plane_dist = cam_near_clip / cam_dir.dot(camera_to_target_unit_vector);

    Eigen::Vector3f up3d, forward3d, right3d;
    up3d = rotate(WORLD_UP, cam_rotation);
    right3d = rotate(WORLD_EAST, cam_rotation);
    forward3d = rotate(WORLD_NORTH, cam_rotation);
    Eigen::Vector3f new_origin = clip_plane_center + (near_clip_height / 2.) * cam_up - (near_clip_width / 2.) * cam_east;

    if (draw_debug)
    {
        auto top_right = new_origin + near_clip_width * right3d;

        GRAPHICS::DRAW_LINE(new_origin.x(), new_origin.y(), new_origin.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



        auto bottom_right = top_right - near_clip_height * up3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



        auto bottom_left = bottom_right - near_clip_width * right3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), bottom_left.x(), bottom_left.y(), bottom_left.z(), 100, 255, 100, 255);
        GRAPHICS::DRAW_LINE(bottom_left.x(), bottom_left.y(), bottom_left.z(), new_origin.x(), new_origin.y(), new_origin.z(), 100, 255, 100, 255);
    }
    Eigen::Vector2f ret;

    bool use_artur_method = true;

    if (use_artur_method)
    {
        Eigen::Vector3f view_plane_point = view_plane_dist * camera_to_target_unit_vector + camera_center;
        view_plane_point = (view_plane_point + clip_plane_center) - new_origin;
        double viewPlaneX = view_plane_point.dot(cam_east) / cam_east.dot(cam_east);
        double viewPlaneZ = view_plane_point.dot(cam_up) / cam_up.dot(cam_up);
        double screenX = viewPlaneX / near_clip_width;
        double screenY = -viewPlaneZ / near_clip_height;
        ret = { screenX, screenY };
    }
    else
    {
        auto intersection = cam_coords + view_plane_dist * camera_to_target_unit_vector;
        auto center_to_intersection = clip_plane_center - intersection;
        auto x_dist = center_to_intersection.dot(right3d);
        auto z_dist = center_to_intersection.dot(up3d);
        auto screen_x = 1. - (near_clip_width / 2. + x_dist) / near_clip_width;
        auto screen_y = (near_clip_height / 2. + z_dist) / near_clip_height;
        ret = { screen_x, screen_y };
    }
    return ret;
}

static const std::vector<Eigen::Vector3f> coefficients = {
    {-0.5, -0.5,-0.5}, 
    { 0.5, -0.5,-0.5},
    { 0.5,  0.5, 0.5},
    {-0.5,  0.5,-0.5},
    {-0.5, -0.5, 0.5},
    { 0.5, -0.5, 0.5},
    { 0.5,  0.5, 0.5},
    {-0.5,  0.5, 0.5}
};



void Scenario::setVehiclesList() {
	const int ARR_SIZE = 1024;
	Vehicle vehicles[ARR_SIZE];
	Value _vehicles(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
	Hash model;
	Vector3 min;
	Vector3 max;
	Vector3 speedVector;
	float heading, speed;
	int classid;

	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle);
	//Vector3 theta = CAM::GET_CAM_ROT(camera, 0);
	Vector3 theta = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
	theta.x = theta.x - 10;

    Vector3 cam_pos = CAM::GET_CAM_COORD(camera);
    float fov = CAM::GET_CAM_FOV(camera);
    float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);
	int count = worldGetAllVehicles(vehicles, ARR_SIZE);
	drawDebug = true;

	Value car_pts(kArrayType);

	auto near_clip_height = near_clip * tan(fov / 2. * (3.14159 / 180.)); // field of view is returned vertically
	auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

	auto fov_h = 2.0 * atan(near_clip_width / near_clip);

	int num_h = 512;
	int num_v = 64;
	int step_x = res_x / num_h;
	int step_y = res_y / num_v;

	int entity_num = 0;
	std::unordered_map<Entity, float> entity_set;

	for (int i = 0; i < count; i++) {
		if (vehicles[i] == vehicle) continue; //Don't process own car!

		ENTITY::SET_ENTITY_AS_MISSION_ENTITY(vehicles[i], TRUE, TRUE);

	}

	Ped peds[ARR_SIZE];

	int count_ped = worldGetAllPeds(peds, ARR_SIZE);
	for (int i = 0; i < count_ped; i++) {
		if (PED::IS_PED_IN_ANY_VEHICLE(peds[i], TRUE)) continue; //Don't process peds in vehicles!

		ENTITY::SET_ENTITY_AS_MISSION_ENTITY(peds[i], TRUE, TRUE);
	}

	for (int l = 0; l < vec_h.size(); l++)
		for (int k = 0; k < vec.size(); k++) {
			int i = vec_h[l];
			int j = vec[k];
			Eigen::Vector3f far_pt = get_dir_from_2d(i, j, res_x, res_y,
				Eigen::Vector3f(cam_pos.x, cam_pos.y, cam_pos.z),
				Eigen::Vector3f(theta.x, theta.y, theta.z), near_clip, fov, false);
			Vector3 camCoords = cam_pos;
			Vector3 farCoords; //getCoordsFromCam(5000.0F); //get coords of point 5000m from your crosshair
			farCoords.x = far_pt(0);
			farCoords.y = far_pt(1);
			farCoords.z = far_pt(2);

			bool active = false, is_ped, is_vehicle, is_object;
			Vector3 endCoords, surfaceNormal; BOOL hit; //placeholder variables that will be filled by _GET_RAYCAST_RESULT
			Entity entityHit = -1; //also will be filled by RAYCAST_RESULT, but you should clear it to 0/NULL/-1 each time so that you dont pick up the last entity you hit if the ray misses

			// THESE TWO NATIVES MUST BE CALLED RIGHT ONE AFTER ANOTHER IN ORDER TO WORK:
			// Cast ray from camera to farCoords:
			int ray = WORLDPROBE::_CAST_RAY_POINT_TO_POINT(camCoords.x, camCoords.y, camCoords.z, farCoords.x, farCoords.y, farCoords.z, -1, player, 7);
			// flag "-1" means it will intersect with anything. 
			// "player" means it will ignore the player (player = PLAYER_PED_ID()). Not sure what 7 means.
			// You can change "player" to "0" if you want the ray to be able to hit yourself; or you can change it to another entity you want the ray to ignore.

			WORLDPROBE::_GET_RAYCAST_RESULT(ray, &hit, &endCoords, &surfaceNormal, &entityHit);
			// fills the bool "hit" with whether or not the ray was stopped before it reached farCoords
			// fills "endCoords" with the coords of where the ray was stopped on its way to farCoords
			// fills "surfaceNormal" with some kind of offset-coords of where the entity was hit relative to said entity
			// fills "entityHit" with the handle of the entity that was hit (if it was an entity)

			float dist = sqrt((camCoords.x - endCoords.x)*(camCoords.x - endCoords.x) + (camCoords.y - endCoords.y)*(camCoords.y - endCoords.y) + (camCoords.z - endCoords.z)*(camCoords.z - endCoords.z));
			Eigen::Vector3f coords_in_cam_sys = get_coords_in_cam_coord_sys(endCoords.x, endCoords.y, endCoords.z, 
																			Eigen::Vector3f(cam_pos.x, cam_pos.y, cam_pos.z),
																			Eigen::Vector3f(theta.x, theta.y, theta.z), near_clip, fov);
			position.x = 0.0;
			position.y = 0.0;
			position.z = 0.0;
			//If you want to use the entityHit for something, use code like below:
			if (hit && ENTITY::DOES_ENTITY_EXIST(entityHit)) {
				ENTITY::GET_ENTITY_MATRIX(entityHit, &rightVector, &forwardVector, &upVector, &position);
				active = true; //use an "active" bool if you need it for what you're doing
				is_ped, is_vehicle, is_object = false; //reset these from what they were with the last entityHit
				switch (ENTITY::GET_ENTITY_TYPE(entityHit)) { //GET_ENTITY_TYPE returns 1 if ped, 2 if veh and 3 if object (and 0 if none of those).
				case 1:
					is_ped = true;
					car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float) j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(1.0, allocator).PushBack((float)entityHit, allocator);
					break;
				case 2:
					is_vehicle = true;
					Hash hit_model;
					hit_model = ENTITY::GET_ENTITY_MODEL(entityHit);

					if (VEHICLE::IS_THIS_MODEL_A_CAR(hit_model))
						car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(2.0, allocator).PushBack((float)entityHit, allocator);
					else if (VEHICLE::IS_THIS_MODEL_A_BIKE(hit_model))
						car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(3.0, allocator).PushBack((float)entityHit, allocator);
					else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(hit_model))
						car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(4.0, allocator).PushBack((float)entityHit, allocator);
					else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(hit_model))
						car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(5.0, allocator).PushBack((float)entityHit, allocator);
					break;
				case 3:
					is_object = true;
					car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(0.0, allocator).PushBack((float)entityHit, allocator);
					break;
				default:
					active = false; //something is wrong, the entity hit is not a ped/vehicle/object, so set inactive
					car_pts.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(-1.0, allocator).PushBack((float)entityHit, allocator);
					break;
				}
				//the below native will retrieve the offset from the entity's main coords to the coords of where the ray hit the entity
				//Vector3 offset = ENTITY::GET_OFFSET_FROM_ENTITY_GIVEN_WORLD_COORDS(entityHit, endCoords.x, endCoords.y, endCoords.z);

				//if (is_vehicle)
					//car_pts.PushBack(i, allocator).PushBack(j, allocator);
			}
			else
				car_pts.PushBack(0.0f, allocator).PushBack(0.0f, allocator).PushBack(0.0f, allocator).PushBack((float)i, allocator).PushBack((float)j, allocator).PushBack(dist, allocator).PushBack(coords_in_cam_sys(0), allocator).PushBack(coords_in_cam_sys(1), allocator).PushBack(coords_in_cam_sys(2), allocator).PushBack(-2.0, allocator).PushBack((float)entityHit, allocator);
		}

	d["car_points"] = car_pts;



	for (int i = 0; i < count; i++) {
		if (vehicles[i] == vehicle) continue; //Don't process own car!
		if (ENTITY::IS_ENTITY_ON_SCREEN(vehicles[i])) {
			//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(vehicles[i], &rightVector, &forwardVector, &upVector, &position); //Blue or red pill
			if (true & SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < 62500) { //250 m.
				if (true & ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(vehicle, vehicles[i], 19)){
					//Check if we see it (not occluded)
					model = ENTITY::GET_ENTITY_MODEL(vehicles[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(vehicles[i], false);
					speed = ENTITY::GET_ENTITY_SPEED(vehicles[i]);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}

					if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) classid = 0;
					else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) classid = 1;
					else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) classid = 2;
					else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) classid = 3;
					else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) classid = 4;
					else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) classid = 5;
					else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) classid = 6;
					else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) classid = 7;
					else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) classid = 8;
					else classid = 9; //unknown (ufo?)

					//Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);

					FUR.x = position.x + max.y*rightVector.x + max.x*forwardVector.x + max.z*upVector.x;
					FUR.y = position.y + max.y*rightVector.y + max.x*forwardVector.y + max.z*upVector.y;
					//FUR.z = position.z + max.y*rightVector.z + max.x*forwardVector.z + max.z*upVector.z;
					GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, 1000.0, &(FUR.z), 0);
					FUR.z += 2 * dim.z;

					BLL.x = position.x + min.y*rightVector.x + min.x*forwardVector.x + min.z*upVector.x;
					BLL.y = position.y + min.y*rightVector.y + min.x*forwardVector.y + min.z*upVector.y;
					//BLL.z = position.z + min.y*rightVector.z + min.x*forwardVector.z + min.z*upVector.z;
					GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Value _vehicle(kObjectType);

					Value _vector(kArrayType);
					_vector.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					_vehicle.AddMember("FUR", _vector, allocator);
					_vector.SetArray();
					_vector.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					_vehicle.AddMember("BLL", _vector, allocator).AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);

					if(bbox){
                        float min_x = 2;
                        float min_y = 2;
                        float max_x = -1;
                        float max_y = -1;
                        Eigen::Matrix3f R;
                        R.col(0) = Eigen::Vector3f(forwardVector.x, forwardVector.y, forwardVector.z);
                        R.col(1) = Eigen::Vector3f(rightVector.x, rightVector.y, rightVector.z);
                        R.col(2) = Eigen::Vector3f(upVector.x, upVector.y, upVector.z);

						Vector3 edge[9];
						edge[1] = BLL;
						edge[5] = FUR;


						edge[2].x = edge[1].x + 2 * dim.y*rightVector.x;
						edge[2].y = edge[1].y + 2 * dim.y*rightVector.y;
						edge[2].z = edge[1].z + 2 * dim.y*rightVector.z;

						edge[3].x = edge[2].x + 2 * dim.z*upVector.x;
						edge[3].y = edge[2].y + 2 * dim.z*upVector.y;
						edge[3].z = edge[2].z + 2 * dim.z*upVector.z;

						edge[4].x = edge[1].x + 2 * dim.z*upVector.x;
						edge[4].y = edge[1].y + 2 * dim.z*upVector.y;
						edge[4].z = edge[1].z + 2 * dim.z*upVector.z;

						edge[6].x = edge[5].x - 2 * dim.y*rightVector.x;
						edge[6].y = edge[5].y - 2 * dim.y*rightVector.y;
						edge[6].z = edge[5].z - 2 * dim.y*rightVector.z;

						edge[7].x = edge[6].x - 2 * dim.z*upVector.x;
						edge[7].y = edge[6].y - 2 * dim.z*upVector.y;
						edge[7].z = edge[6].z - 2 * dim.z*upVector.z;

						edge[8].x = edge[5].x - 2 * dim.z*upVector.x;
						edge[8].y = edge[5].y - 2 * dim.z*upVector.y;
						edge[8].z = edge[5].z - 2 * dim.z*upVector.z;


						Eigen::Vector3f pts[8];

						for (int i = 0; i < 8; i++)
							pts[i] = Eigen::Vector3f(edge[i + 1].x, edge[i + 1].y, edge[i + 1].z);


						_vector.SetArray();
                        for(int i = 0; i < 8; ++i){
							Eigen::Vector3f pt = Eigen::Vector3f(position.x, position.y, position.z) + R*Eigen::Vector3f(dim.x, dim.y, dim.z).cwiseProduct(2*coefficients[i]);

							pt = pts[i];
                            Eigen::Vector2f uv = get_2d_from_3d(pt, 
                                                                Eigen::Vector3f(cam_pos.x, cam_pos.y, cam_pos.z), 
                                                                Eigen::Vector3f(theta.x, theta.y, theta.z), near_clip, fov);
							_vector.PushBack(uv(0), allocator);
							_vector.PushBack(uv(1), allocator);

                            min_x = min(uv(0), min_x);
                            min_y = min(uv(1), min_y);
                            max_x = max(uv(0), max_x);
                            max_y = max(uv(1), max_y);
                        }
						_vehicle.AddMember("2d_coords", _vector, allocator);

                        _vector.SetArray();
                        _vector.PushBack(min_x, allocator).PushBack(min_y, allocator);
                        _vehicle.AddMember("TL", _vector, allocator);
                        _vector.SetArray();
                        _vector.PushBack(max_x, allocator).PushBack(max_y, allocator);
                        _vehicle.AddMember("BR", _vector, allocator);

						_vector.SetArray();
						_vector.PushBack(GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false), allocator);
						_vehicle.AddMember("ratio", _vector, allocator);

						auto near_clip_height = 2 * near_clip * tan(fov / 2. * (3.14159 / 180.)); // field of view is returned vertically
						auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

						_vector.SetArray();
						_vector.PushBack(near_clip_width, allocator).PushBack(near_clip_height, allocator);
						_vehicle.AddMember("width_and_height", _vector, allocator);
                    }
                    _vehicles.PushBack(_vehicle, allocator);

					if(drawDebug){
					    Vector3 edge1 = BLL;
					    Vector3 edge2;
					    Vector3 edge3;
					    Vector3 edge4;
					    Vector3 edge5 = FUR;
					    Vector3 edge6;
					    Vector3 edge7;
					    Vector3 edge8;

					    edge2.x = edge1.x + 2 * dim.y*rightVector.x;
					    edge2.y = edge1.y + 2 * dim.y*rightVector.y;
					    edge2.z = edge1.z + 2 * dim.y*rightVector.z;

					    edge3.x = edge2.x + 2 * dim.z*upVector.x;
					    edge3.y = edge2.y + 2 * dim.z*upVector.y;
					    edge3.z = edge2.z + 2 * dim.z*upVector.z;

					    edge4.x = edge1.x + 2 * dim.z*upVector.x;
					    edge4.y = edge1.y + 2 * dim.z*upVector.y;
					    edge4.z = edge1.z + 2 * dim.z*upVector.z;

					    edge6.x = edge5.x - 2 * dim.y*rightVector.x;
					    edge6.y = edge5.y - 2 * dim.y*rightVector.y;
					    edge6.z = edge5.z - 2 * dim.y*rightVector.z;

					    edge7.x = edge6.x - 2 * dim.z*upVector.x;
					    edge7.y = edge6.y - 2 * dim.z*upVector.y;
					    edge7.z = edge6.z - 2 * dim.z*upVector.z;

					    edge8.x = edge5.x - 2 * dim.z*upVector.x;
					    edge8.y = edge5.y - 2 * dim.z*upVector.y;
					    edge8.z = edge5.z - 2 * dim.z*upVector.z;

					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);

					    GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);

					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 0, 255, 0, 200);
					    GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
                    }

				}
			}
		}
	}
			
	d["vehicles"] = _vehicles;
}

void Scenario::setPedsList(){
	const int ARR_SIZE = 1024;
	Ped peds[ARR_SIZE];
	Value _peds(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
	Hash model;
	Vector3 min;
	Vector3 max;
	Vector3 speedVector;
	float heading, speed;
	int classid;

	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle);
    //Vector3 theta = CAM::GET_CAM_ROT(camera, 0);
	Vector3 theta = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
	theta.x = theta.x - 10;

    Vector3 cam_pos = CAM::GET_CAM_COORD(camera);
    float fov = CAM::GET_CAM_FOV(camera);
    float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);
	int count = worldGetAllPeds(peds, ARR_SIZE);
	for (int i = 0; i < count; i++) {
		if (PED::IS_PED_IN_ANY_VEHICLE(peds[i], TRUE)) continue; //Don't process peds in vehicles!
		if (ENTITY::IS_ENTITY_ON_SCREEN(peds[i])) {
			//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(peds[i], &rightVector, &forwardVector, &upVector, &position); //Blue or red pill
			if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < 22500) { //150 m.
				if (ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(ped, peds[i], 19)){
					//Check if we see it (not occluded)
					model = ENTITY::GET_ENTITY_MODEL(peds[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(peds[i], false);
					speed = ENTITY::GET_ENTITY_SPEED(peds[i]);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}

					if (PED::GET_PED_TYPE(peds[i]) == 28) classid = 11; //animal
					else classid = 10;
                    
					//Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);

					FUR.x = position.x + max.y*rightVector.x + max.x*forwardVector.x + max.z*upVector.x;
					FUR.y = position.y + max.y*rightVector.y + max.x*forwardVector.y + max.z*upVector.y;
					FUR.z = position.z + max.y*rightVector.z + max.x*forwardVector.z + max.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, 1000.0, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x + min.y*rightVector.x + min.x*forwardVector.x + min.z*upVector.x;
					BLL.y = position.y + min.y*rightVector.y + min.x*forwardVector.y + min.z*upVector.y;
					BLL.z = position.z + min.y*rightVector.z + min.x*forwardVector.z + min.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);
                    
					Value _ped(kObjectType);

					Value _vector(kArrayType);
					_vector.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					_ped.AddMember("FUR", _vector, allocator);
					_vector.SetArray();
					_vector.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					_ped.AddMember("BLL", _vector, allocator).AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);
                    if(bbox){
                        float min_x = 2;
                        float min_y = 2;
                        float max_x = -1;
                        float max_y = -1;
                        Eigen::Matrix3f R;
                        R.col(0) = Eigen::Vector3f(forwardVector.x, forwardVector.y, forwardVector.z);
                        R.col(1) = Eigen::Vector3f(rightVector.x, rightVector.y, rightVector.z);
                        R.col(2) = Eigen::Vector3f(upVector.x, upVector.y, upVector.z);
                        for (int i = 0; i < 8; ++i) {
                            Eigen::Vector3f pt = Eigen::Vector3f(position.x, position.y, position.z) + R*Eigen::Vector3f(dim.x, dim.y, dim.z).cwiseProduct(2*coefficients[i]);
                            Eigen::Vector2f uv = get_2d_from_3d(pt,
                                Eigen::Vector3f(cam_pos.x, cam_pos.y, cam_pos.z),
                                Eigen::Vector3f(theta.x, theta.y, theta.z), near_clip, fov);
                            min_x = min(uv(0), min_x);
                            min_y = min(uv(1), min_y);
                            max_x = max(uv(0), max_x);
                            max_y = max(uv(1), max_y);
                        }
                        _vector.SetArray();
                        _vector.PushBack(min_x, allocator).PushBack(min_y, allocator);
                        _ped.AddMember("TL", _vector, allocator);
                        _vector.SetArray();
                        _vector.PushBack(max_x, allocator).PushBack(max_y, allocator);
                        _ped.AddMember("BR", _vector, allocator);
                    }
					_peds.PushBack(_ped, allocator);

					if(drawDebug){
					    Vector3 edge1 = BLL;
					    Vector3 edge2;
					    Vector3 edge3;
					    Vector3 edge4;
					    Vector3 edge5 = FUR;
					    Vector3 edge6;
					    Vector3 edge7;
					    Vector3 edge8;

					    edge2.x = edge1.x + 2 * dim.y*rightVector.x;
					    edge2.y = edge1.y + 2 * dim.y*rightVector.y;
					    edge2.z = edge1.z + 2 * dim.y*rightVector.z;

					    edge3.x = edge2.x + 2 * dim.z*upVector.x;
					    edge3.y = edge2.y + 2 * dim.z*upVector.y;
					    edge3.z = edge2.z + 2 * dim.z*upVector.z;

					    edge4.x = edge1.x + 2 * dim.z*upVector.x;
					    edge4.y = edge1.y + 2 * dim.z*upVector.y;
					    edge4.z = edge1.z + 2 * dim.z*upVector.z;

					    edge6.x = edge5.x - 2 * dim.y*rightVector.x;
					    edge6.y = edge5.y - 2 * dim.y*rightVector.y;
					    edge6.z = edge5.z - 2 * dim.y*rightVector.z;

					    edge7.x = edge6.x - 2 * dim.z*upVector.x;
					    edge7.y = edge6.y - 2 * dim.z*upVector.y;
					    edge7.z = edge6.z - 2 * dim.z*upVector.z;

					    edge8.x = edge5.x - 2 * dim.z*upVector.x;
					    edge8.y = edge5.y - 2 * dim.z*upVector.y;
					    edge8.z = edge5.z - 2 * dim.z*upVector.z;

					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 255, 0, 0, 200);

					    GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 255, 0, 0, 200);

					    GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 255, 0, 0, 200);
					    GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 255, 0, 0, 200);
                    }

				}
			}
		}
	}		
	d["peds"] = _peds;
}


void Scenario::setThrottle(){
	d["throttle"] = getFloatValue(vehicle, 0x8FC);
}

void Scenario::setBrake(){
	d["brake"] = getFloatValue(vehicle, 0x900);
}

void Scenario::setSteering(){
	d["steering"] = getFloatValue(vehicle, 0x8F4) / -0.7;
}

void Scenario::setSpeed(){
	d["speed"] = ENTITY::GET_ENTITY_SPEED(vehicle);
}

void Scenario::setYawRate(){
	Vector3 rates = ENTITY::GET_ENTITY_ROTATION_VELOCITY(vehicle);
	d["yawRate"] = rates.z*180.0 / 3.14159265359;
}

void Scenario::setLocation(){
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 pos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Value location(kArrayType);
	location.PushBack(pos.x, allocator).PushBack(pos.y, allocator).PushBack(pos.z, allocator);
	d["location"] = location;
}

void Scenario::setTime(){
	d["time"] = TIME::GET_CLOCK_HOURS();
}

void Scenario::setDirection(){
	int direction;
	float distance;
	Document::AllocatorType& allocator = d.GetAllocator();
	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &vehicle, &distance);
	Value _direction(kArrayType);
	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
	d["direction"] = _direction;
}

void Scenario::setReward() {
	d["reward"] = rewarder->computeReward(vehicle);
}








//static Eigen::Vector2f get_dir_from_2d(const Eigen::Vector3f& vertex, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, bool draw_debug = false) {
static Eigen::Vector3f get_dir_from_2d(const int x, const int y, const double res_x, const double res_y, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, bool draw_debug) {
		// Inspired by Artur Filopowicz: Video Games for Autonomous Driving: https://github.com/arturf1/GTA5-Scripts/blob/master/Annotator.cs#L379
	draw_debug = false;
	static const Eigen::Vector3f WORLD_NORTH(0.0, 1.0, 0.0);
	static const Eigen::Vector3f WORLD_UP(0.0, 0.0, 1.0);
	static const Eigen::Vector3f WORLD_EAST(1.0, 0.0, 0.0);
	Eigen::Vector3f theta = (3.14159 / 180.0) * cam_rotation;
	auto cam_dir = rotate(WORLD_NORTH, theta);


	auto clip_plane_center = cam_coords + cam_near_clip * cam_dir;
	auto camera_center = -cam_near_clip * cam_dir;
	auto near_clip_height = 2 * cam_near_clip * tan(cam_field_of_view / 2. * (3.14159 / 180.)); // field of view is returned vertically
	auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

	const Eigen::Vector3f cam_up = rotate(WORLD_UP, theta);
	const Eigen::Vector3f cam_east = rotate(WORLD_EAST, theta);


		//const Eigen::Vector3f near_clip_to_target = vertex - clip_plane_center; // del

		//Eigen::Vector3f camera_to_target = near_clip_to_target - camera_center; // Total distance - subtracting a negative to add clip distance



		//Eigen::Vector3f camera_to_target_unit_vector = camera_to_target * (1. / camera_to_target.norm()); // Unit vector in direction of plane / line intersection

	//double view_plane_dist = cam_near_clip / cam_dir.dot(camera_to_target_unit_vector);

	Eigen::Vector3f up3d, forward3d, right3d;
	up3d = rotate(WORLD_UP, cam_rotation);
	right3d = rotate(WORLD_EAST, cam_rotation);
	forward3d = rotate(WORLD_NORTH, cam_rotation);
	Eigen::Vector3f new_origin = clip_plane_center + (near_clip_height / 2.) * cam_up - (near_clip_width / 2.) * cam_east;
	Eigen::Vector3f pt_on_clip_plane;
	pt_on_clip_plane = new_origin + (x / res_x) * near_clip_width * cam_east - (y / res_y) * near_clip_height * cam_up;

	Eigen::Vector3f ori_to_pt;
	ori_to_pt = pt_on_clip_plane - cam_coords;
	return cam_coords + ori_to_pt * (1.0 / ori_to_pt.norm()) * 50000.;

	/*
		if (draw_debug)
		{
			auto top_right = new_origin + near_clip_width * right3d;

			GRAPHICS::DRAW_LINE(new_origin.x(), new_origin.y(), new_origin.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



			auto bottom_right = top_right - near_clip_height * up3d;

			GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



			auto bottom_left = bottom_right - near_clip_width * right3d;

			GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), bottom_left.x(), bottom_left.y(), bottom_left.z(), 100, 255, 100, 255);
			GRAPHICS::DRAW_LINE(bottom_left.x(), bottom_left.y(), bottom_left.z(), new_origin.x(), new_origin.y(), new_origin.z(), 100, 255, 100, 255);
		}

	Eigen::Vector2f ret;

	bool use_artur_method = true;

	if (use_artur_method)
	{
		Eigen::Vector3f view_plane_point = view_plane_dist * camera_to_target_unit_vector + camera_center;
		view_plane_point = (view_plane_point + clip_plane_center) - new_origin;
		double viewPlaneX = view_plane_point.dot(cam_east) / cam_east.dot(cam_east);
		double viewPlaneZ = view_plane_point.dot(cam_up) / cam_up.dot(cam_up);
		double screenX = viewPlaneX / near_clip_width;
		double screenY = -viewPlaneZ / near_clip_height;
		ret = { screenX, screenY };
	}
	else
	{
		auto intersection = cam_coords + view_plane_dist * camera_to_target_unit_vector;
		auto center_to_intersection = clip_plane_center - intersection;
		auto x_dist = center_to_intersection.dot(right3d);
		auto z_dist = center_to_intersection.dot(up3d);
		auto screen_x = 1. - (near_clip_width / 2. + x_dist) / near_clip_width;
		auto screen_y = (near_clip_height / 2. + z_dist) / near_clip_height;
		ret = { screen_x, screen_y };
	}
	return ret;*/
}



static Eigen::Vector3f get_coords_in_cam_coord_sys(float x, float y, float z, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view) {
	// Inspired by Artur Filopowicz: Video Games for Autonomous Driving: https://github.com/arturf1/GTA5-Scripts/blob/master/Annotator.cs#L379

	static const Eigen::Vector3f WORLD_NORTH(0.0, 1.0, 0.0);
	static const Eigen::Vector3f WORLD_UP(0.0, 0.0, 1.0);
	static const Eigen::Vector3f WORLD_EAST(1.0, 0.0, 0.0);
	Eigen::Vector3f theta = (3.14159 / 180.0) * cam_rotation;
	auto cam_dir = rotate(WORLD_NORTH, theta);


	auto clip_plane_center = cam_coords + cam_near_clip * cam_dir;
	auto camera_center = -cam_near_clip * cam_dir;
	auto near_clip_height = 2 * cam_near_clip * tan(cam_field_of_view / 2. * (3.14159 / 180.)); // field of view is returned vertically
	auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

	const Eigen::Vector3f cam_up = rotate(WORLD_UP, theta);
	const Eigen::Vector3f cam_east = rotate(WORLD_EAST, theta);


	//const Eigen::Vector3f near_clip_to_target = vertex - clip_plane_center; // del

	//Eigen::Vector3f camera_to_target = near_clip_to_target - camera_center; // Total distance - subtracting a negative to add clip distance

	float vec_x, vec_y, vec_z;
	vec_x = x - cam_coords(0);
	vec_y = y - cam_coords(1);
	vec_z = z - cam_coords(2);

	Eigen::Vector3f cam_to_target(vec_x, vec_y, vec_z);

	float res_x = cam_to_target.dot(cam_dir);
	float res_y = -cam_to_target.dot(cam_east);
	float res_z = cam_to_target.dot(cam_up);

	return Eigen::Vector3f(res_x, res_y, res_z);


	/*
	if (draw_debug)
	{
	auto top_right = new_origin + near_clip_width * right3d;

	GRAPHICS::DRAW_LINE(new_origin.x(), new_origin.y(), new_origin.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



	auto bottom_right = top_right - near_clip_height * up3d;

	GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



	auto bottom_left = bottom_right - near_clip_width * right3d;

	GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), bottom_left.x(), bottom_left.y(), bottom_left.z(), 100, 255, 100, 255);
	GRAPHICS::DRAW_LINE(bottom_left.x(), bottom_left.y(), bottom_left.z(), new_origin.x(), new_origin.y(), new_origin.z(), 100, 255, 100, 255);
	}

	Eigen::Vector2f ret;

	bool use_artur_method = true;

	if (use_artur_method)
	{
	Eigen::Vector3f view_plane_point = view_plane_dist * camera_to_target_unit_vector + camera_center;
	view_plane_point = (view_plane_point + clip_plane_center) - new_origin;
	double viewPlaneX = view_plane_point.dot(cam_east) / cam_east.dot(cam_east);
	double viewPlaneZ = view_plane_point.dot(cam_up) / cam_up.dot(cam_up);
	double screenX = viewPlaneX / near_clip_width;
	double screenY = -viewPlaneZ / near_clip_height;
	ret = { screenX, screenY };
	}
	else
	{
	auto intersection = cam_coords + view_plane_dist * camera_to_target_unit_vector;
	auto center_to_intersection = clip_plane_center - intersection;
	auto x_dist = center_to_intersection.dot(right3d);
	auto z_dist = center_to_intersection.dot(up3d);
	auto screen_x = 1. - (near_clip_width / 2. + x_dist) / near_clip_width;
	auto screen_y = (near_clip_height / 2. + z_dist) / near_clip_height;
	ret = { screen_x, screen_y };
	}
	return ret;*/
}