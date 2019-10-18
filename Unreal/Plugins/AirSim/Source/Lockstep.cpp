// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "Lockstep.h"
#include "Misc/CoreDelegates.h"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"

FLockstep GLockstep;

FLockstep::FLockstep()
{
}

FLockstep::~FLockstep()
{
}

void FLockstep::SetEnabled()
{
	check(!isEnabled_);
	isEnabled_ = true;
	FCoreDelegates::OnEndFrame.AddRaw(this, &FLockstep::Callback_OnEndFrame);
}


void FLockstep::Callback_OnEndFrame() // Called in GameThread
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
