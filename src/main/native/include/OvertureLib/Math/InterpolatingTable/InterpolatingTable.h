// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <map>
#include <algorithm>
#include <wpi/interpolating_map.h>

template<typename Key, typename Value>
class InterpolatingTable: public wpi::interpolating_map<Key, Value> {
public:
	typedef std::pair<const Key, Value> value_type;

	InterpolatingTable(std::initializer_list<value_type> table) {

		std::for_each(table.begin(), table.end(),
				[this](const std::pair<Key, Value> &a) {
					this->insert(a.first, a.second);
				});
	}
};
