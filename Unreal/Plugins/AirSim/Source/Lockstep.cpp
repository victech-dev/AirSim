// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "Lockstep.h"
#include "Misc/CoreDelegates.h"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"
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

void FLockstep::SetEnabled(msr::airlib::ApiProvider* apiProvider)
{
	check(!isEnabled_);
	isEnabled_ = true;
	apiProvider_ = apiProvider;

	// disable all vehicle camera
	for (auto& simApi : apiProvider_->getVehicleSimApis())
	{
		auto pawnSimApi = static_cast<PawnSimApi*>(simApi);
		auto imageTypeCount = common_utils::Utils::toNumeric(ImageType::Count);
		for (auto& camera : pawnSimApi->getCameras())
			camera->disableAll();
	}

	FCoreDelegates::OnEndFrame.AddRaw(this, &FLockstep::Callback_OnEndFrame);
}


void FLockstep::Callback_OnEndFrame() // Called in GameThread
{
	check(IsInGameThread());

	// We have to call CaptureScene manually because gameViewport->bDisableWorldRendering is turned on during lockstep
	// https://answers.unrealengine.com/questions/759610/how-to-completely-disable-any-camera.html
	for (auto& simApi : apiProvider_->getVehicleSimApis())
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

	// TODO this would not work in SimModeWorld which contains internal async stepping task
	msr::airlib::ClockFactory::get()->step();
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
