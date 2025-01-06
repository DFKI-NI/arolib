/*
 * Copyright 2021-2025 DFKI GmbH
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
*/
 
#ifndef _AROLIB_SIMANNEALING_STATETEGEN_SWAPTRACKS_HPP
#define _AROLIB_SIMANNEALING_STATETEGEN_SWAPTRACKS_HPP

#include "arolib/planning/track_sequencing/simulated_annealing/tracksequencersimannealing.hpp"

namespace arolib {


/**
 * @brief StateGenerator that swaps two random tracks in a sequence to create a new state/sequence
 */
class StateGeneratorSingleSwap : public TrackSequencerSimAnnealing::StateGenerator{
public:
private:

    /**
     * @brief Get a new state and the respective energy
     * @param currentState Current state
     * @param currentEnergy Current energy
     * @return New state and respective energy (nullptr on failure)
     */
    virtual ResultState _getState(const TrackSequencerSimAnnealing::State& currentState, double currentEnergy) override;

protected:
    ResultState getStateSingleSwap(const TrackSequencerSimAnnealing::State& currentState, double currentEnergy);

};

}
#endif // _AROLIB_SIMANNEALING_STATETEGEN_SWAPTRACKS_HPP
