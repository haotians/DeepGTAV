#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "Rewarders\GeneralRewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\SpeedRewarder.h"
#include "defaults.h"
#include <time.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include "Server.h"
#include <windows.h>

#define DEBUG 1

char* Scenario::weatherList[14] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
char* Scenario::vehicleList[3] = { "blista", "voltic", "packer" };

extern std::ofstream debugfile("debug.txt", std::ofstream::out | std::ofstream::app);
char text[100];

static Eigen::Vector3f get_pos_from_2d_and_dist(const int x, const int y, const int dist, const double res_x, const double res_y, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view);

std::vector<Vehicle> all_vehicles;

void wait(int seconds)
{
	clock_t endwait;
	endwait = clock() + seconds * CLOCKS_PER_SEC;
	while (clock() < endwait) {}
}

void display(const char * str, int secs) {
	sprintf(text, str);
	UI::CLEAR_PRINTS();
	UI::_SET_TEXT_ENTRY_2("STRING");
	UI::_ADD_TEXT_COMPONENT_STRING(text);
	UI::_DRAW_SUBTITLE_TIMED(1000 * secs, TRUE);
	/*
	int total_tic = secs * CLOCKS_PER_SEC;
	for (int i = 0; i < total_tic; i++) {
		sprintf(text, str);
		UI::SET_TEXT_CENTRE(TRUE);
		UI::SET_TEXT_SCALE(0.5f, 0.5f);
		UI::SET_TEXT_COLOUR(255, 255, 255, 255);
		UI::_SET_TEXT_ENTRY("STRING");
		UI::_ADD_TEXT_COMPONENT_STRING(text);
		UI::_DRAW_TEXT(0.5f, 0.5f);
	}*/
}

void Scenario::parseScenarioConfig(const Value& sc, bool setDefaults) {
	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

	if (location.IsArray()) {
		if (!location[0].IsNull()) x = location[0].GetFloat();
		else if (setDefaults) x = 5000 * ((float)rand() / RAND_MAX) - 2500;

		if (!location[1].IsNull()) y = location[1].GetFloat(); 
		else if (setDefaults) y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}
	else if (setDefaults) {
		x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}

	if (time.IsArray()) {
		if (!time[0].IsNull()) hour = time[0].GetInt();
		else if (setDefaults) hour = rand() % 24;

		if (!time[1].IsNull()) minute = time[1].GetInt();
		else if (setDefaults) minute = rand() % 60;
	}
	else if (setDefaults) {
		hour = rand() % 24;
		minute = rand() % 60;
	}

	if (!weather.IsNull()) _weather = weather.GetString();
	else if (setDefaults) _weather = weatherList[rand() % 14];

	if (!vehicle.IsNull()) _vehicle = vehicle.GetString();
	else if (setDefaults) _vehicle = vehicleList[rand() % 3];

	if (drivingMode.IsArray()) {
		if (!drivingMode[0].IsNull()) _drivingMode = drivingMode[0].GetInt();
		else if (setDefaults)  _drivingMode = rand() % 4294967296;
		if (drivingMode[1].IsNull()) _setSpeed = drivingMode[1].GetFloat(); 
		else if (setDefaults) _setSpeed = 1.0*(rand() % 20);
	}
	else if (setDefaults) {
		_drivingMode = -1;
	}
}

void Scenario::parseDatasetConfig(const Value& dc, bool setDefaults) {
	if (!dc["rate"].IsNull()) rate = dc["rate"].GetInt();
	else if (setDefaults) rate = _RATE_;
	
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) width = dc["frame"][0].GetInt();
		else if (setDefaults) width = _WIDTH_;

		if (!dc["frame"][1].IsNull()) height = dc["frame"][1].GetInt();
		else if (setDefaults) height = _HEIGHT_;
	}
	else if (setDefaults) {
		width = _WIDTH_;
		height = _HEIGHT_;
	}	

	if (!dc["vehicles"].IsNull()) vehicles = dc["vehicles"].GetBool();
	else if (setDefaults) vehicles = _VEHICLES_;

	if (!dc["peds"].IsNull()) peds = dc["peds"].GetBool();
	else if (setDefaults) peds = _PEDS_;

	if (!dc["trafficSigns"].IsNull()) trafficSigns = dc["trafficSigns"].GetBool();
	else if (setDefaults) trafficSigns = _TRAFFIC_SIGNS_;

	if (!dc["direction"].IsNull()) {
		direction = true;
		if (!dc["direction"][0].IsNull()) dir.x = dc["direction"][0].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][1].IsNull()) dir.y = dc["direction"][1].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][2].IsNull()) dir.z = dc["direction"][2].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;
	}
	else if (setDefaults) direction = _DIRECTION_;

	if (dc["reward"].IsArray()) {
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
			reward = true;
		}
		else if (setDefaults) reward = _REWARD_;
	}
	else if (setDefaults) reward = _REWARD_;

	if (!dc["throttle"].IsNull()) throttle = dc["throttle"].GetBool();
	else if (setDefaults) throttle = _THROTTLE_;
	if (!dc["brake"].IsNull()) brake = dc["brake"].GetBool();
	else if (setDefaults) brake = _BRAKE_;
	if (!dc["steering"].IsNull()) steering = dc["steering"].GetBool();
	else if (setDefaults) steering = _STEERING_;
	if (!dc["speed"].IsNull()) speed = dc["speed"].GetBool();
	else if (setDefaults) speed = _SPEED_;
	if (!dc["yawRate"].IsNull()) yawRate = dc["yawRate"].GetBool();
	else if (setDefaults) yawRate = _YAW_RATE_;
	if (!dc["drivingMode"].IsNull()) drivingMode = dc["drivingMode"].GetBool();
	else if (setDefaults) drivingMode = _DRIVING_MODE_;
	if (!dc["location"].IsNull()) location = dc["location"].GetBool();
	else if (setDefaults) location = _LOCATION_;
	if (!dc["time"].IsNull()) time = dc["time"].GetBool();
	else if (setDefaults) time = _TIME_;

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

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
	if (drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (location) d.AddMember("location", a, allocator);
	if (time) d.AddMember("time", 0, allocator);

	screenCapturer = new ScreenCapturer(width, height);
}

void Scenario::buildScenario() {
	Vector3 pos, rotation;
	Hash vehicleHash;
	float heading;

	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	while (!PATHFIND::LOAD_ALL_PATH_NODES(TRUE)) WAIT(0);
	PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);
	PATHFIND::LOAD_ALL_PATH_NODES(FALSE);

	ENTITY::DELETE_ENTITY(&vehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	while (!ENTITY::DOES_ENTITY_EXIST(vehicle)) {
		vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
		WAIT(0);
	}
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);

	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
		WAIT(0);
	}

	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	PED::SET_PED_INTO_VEHICLE(ped, vehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 0.5, 0.8, TRUE);
	CAM::SET_CAM_FOV(camera, 60);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	AI::CLEAR_PED_TASKS(ped);
	display("Setting first vehicle now...", 5);

	//for (int i = 0; i < 4; i++) {
	//	std::string s("setting cars..");
	//	s = s + " " + std::to_string(i);
	//	display(s.c_str(), 5);
	//	Vector3 cam_pos = CAM::GET_CAM_COORD(camera);
	//	Vector3 theta = ENTITY::GET_ENTITY_ROTATION(this->vehicle, 1);
	//	float near_clip = CAM::GET_CAM_NEAR_CLIP(camera);
	//	float fov = CAM::GET_CAM_FOV(camera);

	//	int idx = i % 2;
	//	Vehicle vehicle_temp;
	//	
	//	vehicleHash = GAMEPLAY::GET_HASH_KEY("slamvan");
	//	STREAMING::REQUEST_MODEL(vehicleHash);
	//	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);

	//	Vector3 coords = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(this->vehicle, 0.0, 10.0 * i, 0.0);




	//	//while (!ENTITY::DOES_ENTITY_EXIST(vehicle_temp)) {
	//		vehicle_temp = VEHICLE::CREATE_VEHICLE(vehicleHash, coords.x, coords.y, coords.z, heading, TRUE, TRUE);
	//		//WAIT(0);
	//	//}
	//	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle_temp);
	//	VEHICLE::SET_VEHICLE_CUSTOM_PRIMARY_COLOUR(vehicle_temp, 0, 153, 0);
	//	
	//}

	//if (_drivingMode >= 0) AI::TASK_VEHICLE_DRIVE_WANDER(ped, vehicle, _setSpeed, _drivingMode);
}

void Scenario::clearAllVehicles(void) {
	for (int i = 0; i < all_vehicles.size(); i++) {
		ENTITY::SET_ENTITY_AS_MISSION_ENTITY(all_vehicles[i], true, true);
		VEHICLE::DELETE_VEHICLE(&all_vehicles[i]);
	}
	all_vehicles.clear();
}

void Scenario::buildOneFormalScenario(const Value& cfg, Server *const server)
{
	clearAllVehicles();
	Vector3 pos, rotation;
	Hash vehicleHash;
	float heading;
	int hour, min;
	rapidjson::SizeType vehicle_num = cfg["vehicles"].Size();
	const Value& vehicles = cfg["vehicles"];

	pos.x = cfg["location"][0].GetFloat();
	pos.y = cfg["location"][1].GetFloat();
	pos.z = cfg["location"][2].GetFloat(); //Probably need to get the gound height

	GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(pos.x, pos.y, pos.z, &(pos.z), 0);

	hour = cfg["time"][0].GetInt();
	min  = cfg["time"][1].GetInt();

	_weather = cfg["weather"].GetString();

	heading = cfg["view_heading"].GetFloat();

	//GAMEPLAY::SET_RANDOM_SEED(std::time(NULL)); // FIXME

	//while (!PATHFIND::LOAD_ALL_PATH_NODES(TRUE)) WAIT(0);
	//PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);
	//PATHFIND::LOAD_ALL_PATH_NODES(FALSE);

	//ENTITY::DELETE_ENTITY(&vehicle);
	//_vehicle = "voltic";
	//vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	//STREAMING::REQUEST_MODEL(vehicleHash);
	//while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	//while (!ENTITY::DOES_ENTITY_EXIST(vehicle)) {
	//	vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
	//	WAIT(0);
	//}
	//VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);

	// get entity to teleport
	Entity e = PLAYER::PLAYER_PED_ID();
	if (PED::IS_PED_IN_ANY_VEHICLE(e, 0))
		e = PED::GET_VEHICLE_PED_IS_USING(e);

	ENTITY::SET_ENTITY_HEADING(e, heading);
	ENTITY::SET_ENTITY_COORDS_NO_OFFSET(e, pos.x, pos.y, pos.z, 0, 0, 1);
	OBJECT::PLACE_OBJECT_ON_GROUND_PROPERLY(e);
	WAIT(0);

	

	//while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
	//	ped = PLAYER::PLAYER_PED_ID();
	//	WAIT(0);
	//}

	//player = PLAYER::PLAYER_ID();
	//PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	//while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	//PED::SET_PED_INTO_VEHICLE(ped, vehicle, -1);
	//STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	//rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
	//CAM::DESTROY_ALL_CAMS(TRUE);
	//camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	//if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	//else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 0.5, 0.8, TRUE);
	//CAM::SET_CAM_FOV(camera, 60);
	//CAM::SET_CAM_ACTIVE(camera, TRUE);
	//CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);
	//CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
	//CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);


	//AI::CLEAR_PED_TASKS(ped);
	
	for (rapidjson::SizeType i = 0; i < vehicle_num; i++) {
		const Value& v = vehicles[i];
		//std::string model = std::string(v["model"].GetString());
		float loc_offset_x = v["location_offset"][0].GetFloat();
		float loc_offset_y = v["location_offset"][1].GetFloat();
		float loc_offset_z = v["location_offset"][2].GetFloat();
		float heading_diff = v["heading"].GetFloat();
		int r = v["color"][0].GetInt();
		int g = v["color"][1].GetInt();
		int b = v["color"][2].GetInt();

		debugfile << "Vehicle " << i << std::endl;
		debugfile << "location_offset is : " << loc_offset_x << ", " << loc_offset_y << ", " << loc_offset_z << std::endl;
		debugfile << "heading is : " << heading << std::endl;
		debugfile << "color is : " << r << ", " << g << ", " << b << std::endl;


		char model[20];
		strcpy(model, v["model"].GetString());

		vehicleHash = GAMEPLAY::GET_HASH_KEY(model);
		debugfile << "The car model is : " << model << std::endl;

		if (STREAMING::IS_MODEL_IN_CDIMAGE(vehicleHash) && STREAMING::IS_MODEL_A_VEHICLE(vehicleHash)) {
			STREAMING::REQUEST_MODEL(vehicleHash);
			while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);

			Vector3 coords = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), loc_offset_x, loc_offset_y, loc_offset_z);

			//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(coords.x, coords.y, coords.z, &(coords.z), 0);

			Vehicle vehicle_temp = VEHICLE::CREATE_VEHICLE(vehicleHash, coords.x, coords.y, coords.z, 0.0, TRUE, TRUE);
			all_vehicles.push_back(vehicle_temp);
			VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle_temp);
			VEHICLE::SET_VEHICLE_CUSTOM_PRIMARY_COLOUR(vehicle_temp, r, g, b);
			ENTITY::SET_ENTITY_HEADING(vehicle_temp, heading + heading_diff);
			WAIT(0);
			STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);
		}
		else
			debugfile << "The model doesn't exist!!" << std::endl;

	}

	WAIT(10000);
	server->checkSendMessage();

}

void Scenario::buildFormalScenarios(const Value& cfgs, Server *const server)
{
	//Vector3 pos, rotation;
	//Hash vehicleHash;
	//float heading;

	//GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	//while (!PATHFIND::LOAD_ALL_PATH_NODES(TRUE)) WAIT(0);
	//PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);
	//PATHFIND::LOAD_ALL_PATH_NODES(FALSE);

	//ENTITY::DELETE_ENTITY(&vehicle);
	//_vehicle = "blista";
	//vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	//STREAMING::REQUEST_MODEL(vehicleHash);
	//while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	//while (!ENTITY::DOES_ENTITY_EXIST(vehicle)) {
	//	vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
	//	WAIT(0);
	//}
	//VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);

	//while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
	//	ped = PLAYER::PLAYER_PED_ID();
	//	WAIT(0);
	//}

	//player = PLAYER::PLAYER_ID();
	//PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	//while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	//PED::SET_PED_INTO_VEHICLE(ped, vehicle, -1);
	//STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	_vehicle = "blista";
	x = cfgs[0]["location"][0].GetFloat();
	y = cfgs[0]["location"][1].GetFloat();
	hour = 12;
	minute = 12;
	_weather = "EXTRASUNNY";
	//buildScenario();

	rapidjson::SizeType cfgs_num = cfgs.Size();

	for (rapidjson::SizeType i = 0; i < cfgs_num; i++) {
		debugfile << "Processing Scenario : " << i << std::endl;
		buildOneFormalScenario(cfgs[i], server);
		//std::this_thread::sleep_for(std::chrono::seconds(10));
	}

	//const Value& cfg1 = cfgs[0];
	///*'{"Formal_Configs": [{"location": [1.0, 2.0], "time": [9, 34], "vehicles": [{"location_offset": [1, 2, 3], "heading": 1, "color": [2, 3, 4]}, {"location_offset": null, "heading": 1, "color": null}], "vehicle_num": 2},
	//{"location": [1.0, 2.0], "time": [9, 34], "vehicles": [{"location_offset": [1, 2, 3], "heading": 1, "color": [2, 3, 4]}, {"location_offset": null, "heading": 1, "color": null}], "vehicle_num": 2}]}'*/
	//const Value& location = cfg1["location"];
	//debugfile << "The location is: " << location[0].GetFloat() << ", " << location[1].GetFloat() << std::endl;
	//const Value& time = cfg1["time"];
	//debugfile << "The time is: " << time[0].GetFloat() << ", " << time[1].GetFloat() << std::endl;
	//const Value& vehicles = cfg1["vehicles"];
	//const Value& v1 = vehicles[0];
	//debugfile << "The location offset of the first v is: " << v1["location_offset"][0].GetFloat() << ", " << v1["location_offset"][1].GetFloat() << ", " << v1["location_offset"][2].GetFloat() << std::endl;
	//debugfile << "The num of vehicles in the second scenario is: " << cfgs[1]["vehicles"].Size() << std::endl;
	////for (int i = 0; i < )
}


void Scenario::start(const Value& sc, const Value& dc) {
	debugfile << "STARTING@@" << std::endl;
	if (running) return;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, true);
	parseDatasetConfig(dc, true);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;

	running = false;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, false);
	parseDatasetConfig(dc, false);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::run() {
	if (running) {
		std::clock_t now = std::clock();

		Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
		CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);

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
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

	d.AddMember("vehicles", a, allocator);
	d.AddMember("lidar_pts", a, allocator);

	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);
	
	screenCapturer->capture();

	if (server->isFormalScenarios) {
		vehicles = true;
	}

	if (vehicles) setVehiclesList();
	//if (peds) setPedsList();
	//if (trafficSigns); //TODO
	//if (direction) setDirection();
	//if (reward) setReward();
	//if (throttle) setThrottle();
	//if (brake) setBrake();
	//if (steering) setSteering();
	//if (speed) setSpeed();
	//if (yawRate) setYawRate();
	//if (drivingMode); //TODO
	//if (location) setLocation();
	//if (time) setTime();

	

	//Document::AllocatorType& allocator = d.GetAllocator();
	//Value test(kArrayType);
	//test.SetArray();
	//test.PushBack(Value().SetDouble(999.0), allocator).PushBack(Value().SetDouble(888.0), allocator).PushBack(Value().SetDouble(777.0), allocator);
	//debugfile << "Inside: Before setting..." << std::endl;
	//d["another"] = test;
	//debugfile << "Inside: After setting..." << std::endl;
	

	d.Accept(writer);

	debugfile << "Inside generateMessage() : " << buffer.GetString() << std::endl;
	return buffer;
}

void Scenario::setLidarPoints()
{
	;
}

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
	Vector3 currentPos;

	Entity e = PLAYER::PLAYER_PED_ID();
	if (PED::IS_PED_IN_ANY_VEHICLE(e, 0)) {
		vehicle = PED::GET_VEHICLE_PED_IS_USING(e);
		currentPos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	}
	else 
		currentPos = ENTITY::GET_ENTITY_COORDS(e, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicle);

	int count = worldGetAllVehicles(vehicles, ARR_SIZE);
	count = all_vehicles.size();
	debugfile << "car # is : " << count << std::endl;
	for (int i = 0; i < count; i++) {
		if (all_vehicles[i] == vehicle) continue; //Don't process own car!
		if (ENTITY::IS_ENTITY_ON_SCREEN(all_vehicles[i])) {
			debugfile << "get car number " << i << std::endl;
			//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(all_vehicles[i], &rightVector, &forwardVector, &upVector, &position); //Blue or red pill
			//if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < 22500) { //150 m.
			//	if (ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(vehicle, vehicles[i], 19)){
					//Check if we see it (not occluded)
					model = ENTITY::GET_ENTITY_MODEL(all_vehicles[i]);
					debugfile << "Getting one vehicle..." << std::endl;
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(all_vehicles[i], false);
					speed = ENTITY::GET_ENTITY_SPEED(all_vehicles[i]);
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

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, FUR.z, &(FUR.z), 0);
					FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Value _vehicle(kObjectType);

					Value _vector(kArrayType);
					_vector.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					_vehicle.AddMember("FUR", _vector, allocator);
					_vector.SetArray();
					_vector.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					_vehicle.AddMember("BLL", _vector, allocator).AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);

					

					float screen_x, screen_y;
					_vector.SetArray();

					Vector3 edge1 = BLL;
					Vector3 edge2;
					Vector3 edge3;
					Vector3 edge4;
					Vector3 edge5 = FUR;
					Vector3 edge6;
					Vector3 edge7;
					Vector3 edge8;

					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge1.x, edge1.y, edge1.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge2.x = edge1.x + 2 * dim.y*rightVector.x;
					edge2.y = edge1.y + 2 * dim.y*rightVector.y;
					edge2.z = edge1.z + 2 * dim.y*rightVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge2.x, edge2.y, edge2.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge3.x = edge2.x + 2 * dim.z*upVector.x;
					edge3.y = edge2.y + 2 * dim.z*upVector.y;
					edge3.z = edge2.z + 2 * dim.z*upVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge3.x, edge3.y, edge3.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge4.x = edge1.x + 2 * dim.z*upVector.x;
					edge4.y = edge1.y + 2 * dim.z*upVector.y;
					edge4.z = edge1.z + 2 * dim.z*upVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge4.x, edge4.y, edge4.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge5.x, edge5.y, edge5.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge6.x = edge5.x - 2 * dim.y*rightVector.x;
					edge6.y = edge5.y - 2 * dim.y*rightVector.y;
					edge6.z = edge5.z - 2 * dim.y*rightVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge6.x, edge6.y, edge6.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge7.x = edge6.x - 2 * dim.z*upVector.x;
					edge7.y = edge6.y - 2 * dim.z*upVector.y;
					edge7.z = edge6.z - 2 * dim.z*upVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge7.x, edge7.y, edge7.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);

					edge8.x = edge5.x - 2 * dim.z*upVector.x;
					edge8.y = edge5.y - 2 * dim.z*upVector.y;
					edge8.z = edge5.z - 2 * dim.z*upVector.z;
					GRAPHICS::_WORLD3D_TO_SCREEN2D(edge8.x, edge8.y, edge8.z, &screen_x, &screen_y);
					_vector.PushBack(screen_x, allocator).PushBack(screen_y, allocator);


					_vehicle.AddMember("coords_2d", _vector, allocator);
					_vehicles.PushBack(_vehicle, allocator);

					//#ifdef DEBUG
					//for (int i = 0; i < 10000; i++) {
					//	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 0, 255, 0, 200);

					//	GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);

					//	GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 0, 255, 0, 200);
					//	GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 0, 255, 0, 200);
					//}
					//#endif
					
				//}
			//}
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

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, 1000.0, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Value _ped(kObjectType);

					Value _vector(kArrayType);
					_vector.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					_ped.AddMember("FUR", _vector, allocator);
					_vector.SetArray();
					_vector.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					_ped.AddMember("BLL", _vector, allocator).AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);

					_peds.PushBack(_ped, allocator);

					#ifdef DEBUG
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
					#endif
				}
			}
		}
	}		
	d["peds"] = _peds;
}


void Scenario::setThrottle(){
	d["throttle"] = getFloatValue(vehicle, 0x90C);
}

void Scenario::setBrake(){
	d["brake"] = getFloatValue(vehicle, 0x910);
}

void Scenario::setSteering(){
	d["steering"] = getFloatValue(vehicle, 0x904) / -0.7;
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
	Vehicle temp_vehicle;
	Document::AllocatorType& allocator = d.GetAllocator();
	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &temp_vehicle, &distance);
	Value _direction(kArrayType);
	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
	d["direction"] = _direction;
}

void Scenario::setReward() {
	d["reward"] = rewarder->computeReward(vehicle);
}

Eigen::Vector3f rotate(Eigen::Vector3f a, Eigen::Vector3f theta)
{
	Eigen::Vector3f d;

	d(0) = (float)cos((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) - (float)sin((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
	d(1) = (float)sin((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) + (float)cos((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
	d(2) = -(float)sin((double)theta(1))*a(0) + (float)cos((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2));

	return d;
}

static Eigen::Vector3f get_pos_from_2d_and_dist(const int x, const int y, const int dist, const double res_x, const double res_y, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view) {
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
	return cam_coords + ori_to_pt * (1.0 / ori_to_pt.norm()) * dist;
}