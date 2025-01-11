#include "OvertureLib/Utils/WaitForButton/WaitForButton.h"

frc2::CommandPtr WaitForButton(OverXboxController *gamepad, int buttonNumber) {
	return frc2::cmd::WaitUntil([=]() {
		return gamepad->GetHID().GetRawButton(buttonNumber);
	});
}
