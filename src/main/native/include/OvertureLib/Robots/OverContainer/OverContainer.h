// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

class OverContainer {
public:
	OverContainer();

	virtual void ConfigDriverBindings() = 0;
	virtual void ConfigOperatorBindings() = 0;
	virtual void ConfigDefaultCommands() = 0;
	virtual void setCharacterizationBindings() {
	}
	;

};
