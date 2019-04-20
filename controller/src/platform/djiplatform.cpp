/*
 * djiplatform.cpp
 *
 *  Created on: Jan 30, 2019
 *      Author: rob
 */

#include <string>
#include <fstream>
#include <unordered_map>

#include <djiosdk/dji_telemetry.hpp>

#include "platform/djiplatform.hpp"

using namespace platform;
using namespace platform::dji;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

DJIPlatform::DJIPlatform() :
	m_listener(nullptr),
	m_vehicle(nullptr) {
}

void DJIPlatform::addListener(PlatformListener* listener) {
	m_listener = listener;
}

void DJIPlatform::loadActivateData(const std::string& config) {

	std::unordered_map<std::string, std::string> conf;
	{
		// Load the config file.
		std::ifstream input(config);
		std::string name;
		while(std::getline(input, name, ':')) {
			std::getline(input, conf[name]);
		}
	}

	// TODO: Find out what config parameters are needed.
	//m_activateData.ID = atoi(conf["ID"].c_str());
	//m_activateData.encKey = conf["encKey"].c_str();
	//m_activateData.version = atoi(conf["version"].c_str());
}

// NOTE: The platform API must be enabled using the DJI assistant.
// Permissions on the linux box must be configured for group dialout, per https://developer.dji.com/onboard-sdk/documentation/development-workflow/environment-setup.html
void DJIPlatform::startup(const std::string& config) {

	if(m_vehicle)
		shutdown();

	loadActivateData(config);

	m_vehicle = new Vehicle(true);

	ACK::ErrorCode res;

	// Activate the vehicle.
	res = m_vehicle->activate(&m_activateData, DJI_TIMEOUT);
	if(ACK::getError(res) != ACK::SUCCESS) {
		//ACK::getErrorCodeMessage(res, __func__);
		throw std::runtime_error("Failed to activate vehicle.");
	}

	// Package 0: Subscribe to flight status at freq 1 Hz
	int pkgIndex = 0;	// Indices are sequential from 0.
	int freq = 1;		// Update frequency.
	TopicName topicList[] = { TOPIC_STATUS_FLIGHT }; // TODO: Add topics
	int numTopics = sizeof(topicList) / sizeof(topicList[0]);

	// Retain the package index.
	m_subPkgIds.push_back(pkgIndex);

	// Subscribe the package.
	bool pkgStatus = m_vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopics, topicList, true, freq);
	if (!pkgStatus)
		throw std::runtime_error("Failed to register telemetry topics.");

	// Start the subscription.
	res = m_vehicle->subscribe->startPackage(pkgIndex, DJI_TIMEOUT);
	if (ACK::getError(res) != ACK::SUCCESS) {
		//ACK::getErrorCodeMessage(res, __func__);
		m_vehicle->subscribe->removePackage(pkgIndex, DJI_TIMEOUT);
		throw std::runtime_error("Failed to start telemetry topics.");
	}
}

void DJIPlatform::shutdown() {
	if(m_vehicle) {
		for(int id : m_subPkgIds)
			m_vehicle->subscribe->removePackage(id, DJI_TIMEOUT);
		m_vehicle->releaseCtrlAuthority(DJI_TIMEOUT);
		delete m_vehicle;
		m_vehicle = nullptr;
	}
}

DJIPlatform::~DJIPlatform() {
	shutdown();
}

