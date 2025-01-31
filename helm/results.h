#ifndef _HELM_RESULTS_H_
#define _HELM_RESULTS_H_

#include "goal.h"
#include "instrument-reading.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief
 */
typedef struct _Results {

  /*
   * @brief num goals reached
   * Since the ScenarioRunner will attempt Stage in-order, the following are
   * logically true: While the Scenario is not finished, numStagesReached
   * behaves as the index of the current Goal. After the Scenario is finished,
   * numStagesReached will be equal to the number of Stage in the
   * Scenario.
   */
  uint8_t numStagesReached;

  /*
   * @brief while the Scenario is not finished, this will be updated every time
   * the helm is updated with a new InstrumentReading.
   * Once the Scenario is finished, this will be "frozen" and new
   * InstrumentReadings will be ignored.
   */
  InstrumentReading finalReading;

} Results;

#endif // _HELM_RESULTS_H_;
