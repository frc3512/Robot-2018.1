// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

SubsystemBase::SubsystemBase() { m_notifier.StartPeriodic(0.02); }

void SubsystemBase::SubsystemPeriodic() {}
