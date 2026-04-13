/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm.h"

#if MECHANICAL_ARM_TYPE != MECHANICAL_ARM_NONE

#include <stdlib.h>

// MechanicalArmApi_t mechanical_arm = {
//     .SetCali = SetCali,
//     .CmdCali = CmdCali,
//     .GetStatus = GetStatus,
//     .GetDuration = GetDuration,
// };
MechanicalArmApi_t mechanical_arm = {
    .SetCali = NULL,
    .CmdCali = NULL,
    .GetStatus = NULL,
    .GetDuration = NULL,
};
#endif  // MECHANICAL_ARM_TYPR
/*------------------------------ End of File ------------------------------*/
