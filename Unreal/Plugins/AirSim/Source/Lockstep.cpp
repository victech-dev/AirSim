// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "Lockstep.h"
#include "Misc/CoreDelegates.h"
#include "Kismet/GameplayStatics.h"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"
#include "common/AirSimSettings.hpp"
#include "UnrealImageCapture.h"
#include "PawnSimApi.h"

FLockstep GLockstep;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

FLockstep::FLockstep()
{
}

FLockstep::~FLockstep()
{
}

void FLockstep::Initialize(ASimModeBase* simmode)
{
	check(simmode_ == nullptr);
	simmode_ = simmode;
	ASimModeWorldBase* simmode_world = Cast<ASimModeWorldBase>(simmode_);

	// Adjust steppable clock
	{
		typedef common_utils::Utils Utils;
		typedef msr::airlib::AirSimSettings AirSimSettings;
		typedef msr::airlib::ClockFactory ClockFactory;
		float clock_speed = AirSimSettings::singleton().clock_speed;
		if (!Utils::isApproximatelyEqual(clock_speed, 1.0f))
			throw std::invalid_argument("clock_speed must be 1.0 when lockstep is enabled");
		if (simmode_world != nullptr)
		{
			double physicsPeriod = simmode_world->getPhysicsWorld().getUpdatePeriodNanos() * 1E-9;
			physicsUpdatePerFrame_ = FMath::RoundToInt(float(FApp::GetFixedDeltaTime() / physicsPeriod));
			FApp::SetFixedDeltaTime(physicsPeriod * physicsUpdatePerFrame_);
			ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(physicsPeriod))); //no clock_speed multiplier
		}
		else
		{
			// We need to change scalable clock to steppable clock by FixedDeltaTime
			msr::airlib::ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(FApp::GetFixedDeltaTime())));
		}
	}

	// Disable window rendering
	simmode_->CameraDirector->inputEventNoDisplayView();

	// Disable all PIP camera
	TSubclassOf<APIPCamera> classToFind = APIPCamera::StaticClass();
	TArray<AActor*> foundActors;
	UGameplayStatics::GetAllActorsOfClass(simmode_, classToFind, foundActors);
	for (auto actor : foundActors)
	{
		if (APIPCamera* camera = Cast<APIPCamera>(actor))
		{
			camera->disableAll();
			camera->onViewModeChanged(true);
		}
	}

	FCoreDelegates::OnEndFrame.AddRaw(this, &FLockstep::Callback_OnEndFrame);
}


void FLockstep::Callback_OnEndFrame() // Called in GameThread
{
	check(IsInGameThread());

	// We have to call CaptureScene manually
	for (auto& simApi : simmode_->getApiProvider()->getVehicleSimApis())
	{
		auto pawnSimApi = static_cast<PawnSimApi*>(simApi);
		auto imageTypeCount = common_utils::Utils::toNumeric(ImageType::Count);
		for (auto& camera : pawnSimApi->getCameras())
		{
			for (int image_type = 0; image_type < imageTypeCount; ++image_type)
			{
				USceneCaptureComponent2D* capture = camera->getCaptureComponent(common_utils::Utils::toEnum<ImageType>(image_type), true);
				if (capture != nullptr)
				{
					capture->CaptureScene();
				}
			}
		}
	}

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

	// Step physics/clock here
	ASimModeWorldBase* simmode_world = Cast<ASimModeWorldBase>(simmode_);
	if (simmode_world != nullptr)
	{
		msr::airlib::PhysicsWorld& world = simmode_world->getPhysicsWorld();
		world.lock();
		for (int i = 0; i < physicsUpdatePerFrame_; i++)
			world.updateSync();
		world.unlock();
	}
	else
	{
		msr::airlib::ClockFactory::get()->step();
	}
}

void FLockstep::Lockstep() // Called in external (i.e. rpc handler thread)
{
	check(!IsInGameThread());

	// Wait for end of current frame.
	// This is required to synchronize the very first frame.
	{
		std::unique_lock<std::mutex> lk(mtx_);
		cv_.wait(lk, [this]() { return !isGameThreadRunning_; });
	}

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
