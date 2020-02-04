// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "Lockstep.h"
#include "Engine/Engine.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CoreDelegates.h"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "UnrealImageCapture.h"
#include "PawnSimApi.h"

std::shared_ptr<FLockstep> GLockstep;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

FLockstep::FLockstep(TTimeDelta step, TTimePoint start)
	: msr::airlib::SteppableClock(step, start)
{
}

FLockstep::~FLockstep()
{
}

void FLockstep::Initialize(ASimModeBase* simmode)
{
	check(!GLockstep);

	ASimModeWorldBase* simmode_world = Cast<ASimModeWorldBase>(simmode);

	// disable debug message output
	GEngine->bEnableOnScreenDebugMessages = false;
	// disable rendering (this cound be restored if simGetImages requested afterward)
	simmode->CameraDirector->inputEventNoDisplayView();

	// Adjust steppable clock
	{
		typedef common_utils::Utils Utils;
		typedef msr::airlib::AirSimSettings AirSimSettings;
		typedef msr::airlib::ClockFactory ClockFactory;
		float clock_speed = AirSimSettings::singleton().clock_speed;
		if (!Utils::isApproximatelyEqual(clock_speed, 1.0f))
			throw std::invalid_argument("clock_speed must be 1.0 when lockstep is enabled");

		// For car : Change scalable clock to steppable clock by FixedDeltaTime
		// For multirotor : Change step size of steppable clock to physics period 
		GLockstep = std::make_shared<FLockstep>(static_cast<TTimeDelta>(
			simmode_world != nullptr ? simmode_world->getPhysicsLoopPeriod() * 1E-9 : FApp::GetFixedDeltaTime()));
		GLockstep->simmode_ = simmode;
		GLockstep->frameDeltaTime_ = FApp::GetFixedDeltaTime();
		GLockstep->isSimModeWorld_ = (simmode_world != nullptr);
		if (simmode_world != nullptr)
			GLockstep->RegisterPhysicsEvent(GLockstep->getStepSize());
		ClockFactory::get(GLockstep);
	}

	FCoreDelegates::OnEndFrame.AddRaw(GLockstep.get(), &FLockstep::Callback_OnEndFrame);
}

// Called in GameThread
void FLockstep::Callback_OnEndFrame() 
{
	check(IsInGameThread());

	// Signal end of frame
	{
		std::unique_lock<std::mutex> lk(mtx_);
		isGameThreadRunning_ = false;
	}
	cv_.notify_one();

	// Wait for lockstep signal of next frame
	{
		std::unique_lock<std::mutex> lk(mtx_);
		cv_.wait(lk, [this]() { return isGameThreadRunning_; });
	}

	// Step clock here (PhysX mode)
	if (!isSimModeWorld_ && FApp::GetFixedDeltaTime() > 0)
		msr::airlib::ClockFactory::get()->step();
}

// Called in external (i.e. rpc handler thread)
void FLockstep::Lockstep(bool paused) 
{
	check(!IsInGameThread());

	// Wait for end of current frame (This is required for the very first frame)
	{
		std::unique_lock<std::mutex> lk(mtx_);
		cv_.wait(lk, [this]() { return !isGameThreadRunning_; });
	}

	FApp::SetFixedDeltaTime(paused ? 0 : frameDeltaTime_);
	// Signal lockstep to UE GameThread.
	{
		std::unique_lock<std::mutex> lk(mtx_);
		isGameThreadRunning_ = true;
	}
	cv_.notify_one();

	// Wait for end of next frame.
	{
		std::unique_lock<std::mutex> lk(mtx_);
		cv_.wait(lk, [this]() { return !isGameThreadRunning_; });
	}
}

void FLockstep::WorldTick(msr::airlib::PhysicsWorld& world, float deltaTime)
{
	static TTimePoint sFrameTime = nowNanos();
	sFrameTime = addTo(sFrameTime, deltaTime);

	Event front;
	while (PopEvent(front, sFrameTime))
	{
		switch (front.type)
		{
		case EventType::kPhysics:
			world.lock();
			world.updateSync();
			world.unlock();
			break;
		case EventType::kWaiter:
			front.waiter_signal->waitForWorker();
			front.waiter_signal->signalToWorker();
			UE_LOG(LogTemp, Log, TEXT("ControlCommand %lld"), front.time);
			break;
		}
	}
}

void FLockstep::RestoreViewMode()
{
	ACameraDirector* camera = simmode_->CameraDirector;
	ECameraDirectorMode initialMode = simmode_->getInitialViewMode();
	if (camera->getMode() != initialMode)
	{
		UAirBlueprintLib::RunCommandOnGameThread([=]() {
			switch (initialMode)
			{
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FLY_WITH_ME: camera->inputEventFlyWithView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FPV: camera->inputEventFpvView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_GROUND_OBSERVER: camera->inputEventGroundView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_MANUAL: camera->inputEventManualView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_SPRINGARM_CHASE: camera->inputEventSpringArmChaseView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_BACKUP: camera->inputEventBackupView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_NODISPLAY: camera->inputEventNoDisplayView(); break;
			case ECameraDirectorMode::CAMERA_DIRECTOR_MODE_FRONT: camera->inputEventFrontView(); break;
			default:
				throw std::out_of_range("Unsupported view mode specified in CameraDirector::initializeForBeginPlay");
			}
		}, true);
	}
}

void FLockstep::registerWaiter(std::shared_ptr<WaiterSyncSignal> waiter_signal, TTimeDelta period)
{
	// start new event
	std::unique_lock<std::mutex> lk(eventMutex_);
	PushEvent(EventType::kWaiter, addTo(nowNanos(), period), period, waiter_signal);
}

void FLockstep::unregisterWaiter(std::shared_ptr<WaiterSyncSignal> waiter_signal)
{
	// remove this waiter event
	std::unique_lock<std::mutex> lk(eventMutex_);
	auto fi = std::find_if(events_.begin(), events_.end(), 
		[waiter_signal](const auto& e) { return waiter_signal == e.waiter_signal; });
	if (fi != events_.end())
		events_.erase(fi);
}

void FLockstep::signalCanceledWaiter()
{
	// collect canceled worker
	std::vector<std::shared_ptr<WaiterSyncSignal>> canceled_waiters;
	{
		std::unique_lock<std::mutex> lk(eventMutex_);
		for (auto& e : events_)
		{
			if (e.waiter_signal && e.waiter_signal->isCancelled())
				canceled_waiters.push_back(e.waiter_signal);
		}
	}
	// signal to worker
	for (auto& w : canceled_waiters)
		w->signalToWorker();
}

bool FLockstep::PopEvent(Event& front, TTimePoint until)
{
	std::unique_lock<std::mutex> lk(eventMutex_);
	if (events_.empty())
		throw std::runtime_error("Empty lockstep events list!");
	if (events_.front().time < until)
	{
		// pop front
		front = events_.front();
		events_.erase(events_.begin());
		// push next
		PushEvent(front.type, addTo(front.time, front.period), front.period, front.waiter_signal);
		return true;
	}
	return false;
}

void FLockstep::PushEvent(EventType type, TTimePoint time, TTimeDelta period, std::shared_ptr<WaiterSyncSignal> waiter_signal)
{
	auto fi = std::find_if(events_.begin(), events_.end(), [time](const auto& e) { return time < e.time; });
	events_.insert(fi, {type, time, period, waiter_signal });
}

void FLockstep::RegisterPhysicsEvent(TTimeDelta period)
{
	std::unique_lock<std::mutex> lk(eventMutex_);
	PushEvent(EventType::kPhysics, addTo(nowNanos(), period), period, nullptr);
}
