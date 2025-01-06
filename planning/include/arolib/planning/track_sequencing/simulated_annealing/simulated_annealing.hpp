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
 
#ifndef _AROLIB_TRACKSEQUENCING_SIM_ANNEALING_HPP
#define _AROLIB_TRACKSEQUENCING_SIM_ANNEALING_HPP

#include <vector>
#include <functional>
#include <cmath>
#include <memory>
#include <chrono>

#include "arolib/misc/randomgeneration.hpp"

namespace arolib {

template<typename State>
class SimulatedAnnealing{
public:
    using TemperatureFunction = std::function< double ( size_t /*iteration*/ ) >;
    using BreakFunction = std::function< bool /*break*/ ( const State& /*state*/, double /*energy*/ ) >;
    using RegisterVisitedStateFunction = std::function< void ( const State& /*state*/, double /*energy*/, bool /*bestSoFar*/, bool /*accepted*/, bool /*partOfSubIteration*/ ) >;
    using ResultState = std::unique_ptr< std::pair<State /*state*/, double /*energy*/> >;

    static inline ResultState initResultState(const State& state, double energy = 0){
        return std::make_unique< std::pair<State, double> >( std::make_pair(state, energy) );
    }

    class IStateGenerator{
    public:

        /**
         * @brief Get a new state and the respective energy
         *
         * @param currentState Current state
         * @param currentEnergy Current energy
         * @return New state and respective energy (nullptr on failure)
         */
        virtual ResultState getState(const State& currentState, double currentEnergy) = 0;
    };


    /**
     * @brief Compute
     * @param start Initial state
     * @param currentEnergy Initial energy
     * @param iterations Number of iterations
     * @param stateGenerator State generator
     * @param temperatureFunction Callback to compute the temperature
     * @param minimizeEnergy Minimize (true) or maximize (false) energy
     * @param subIterations Number of sub-iterations per iteration (the best state from the sub-inetarions is used for the iteration)
     * @param breakFunction Callback to know when to break based on the current state
     * @param maxPlanningTime Max planning time [s] (disregarded if <= 0)
     * @param (optional) registerVisitedState Callback to add/track the computed states
     * @return New state and respective energy (nullptr on failure)
     */
    ResultState compute(const State& start, double currentEnergy,
                        size_t iterations,
                        IStateGenerator & stateGenerator,
                        const TemperatureFunction & temperatureFunction,
                        bool minimizeEnergy,
                        size_t subIterations = 1,
                        BreakFunction *breakFunction = nullptr,
                        double maxPlanningTime = -1,
                        const RegisterVisitedStateFunction & registerVisitedState = [](const State&, double, bool, bool, bool){}){

        auto isBetter = [&minimizeEnergy](double a, double b){ //returns b better than a
            return (minimizeEnergy ? b < a : b > a);
        };

        ResultState result = nullptr;

        ResultState currentState = std::make_unique< std::pair<State, double> >( std::make_pair(start, currentEnergy) );
        double bestEnergy = currentEnergy;
        bool gotBetter = false;
        bool ok = false;

        size_t _subIterations = std::max((size_t)1, subIterations);

        registerVisitedState(start, currentEnergy, true, true, false);

        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

        for(size_t i = 0 ; i <= iterations ; ++i){

            if(maxPlanningTime > 1e-6){
                double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
                if(duration > maxPlanningTime){
                    break;
                }
            }

            try{
                ResultState result2 = nullptr;
                for(size_t j = 0 ; j < _subIterations ; ++j){
                    auto newStateAndEnergy = stateGenerator.getState(currentState->first, currentState->second);
                    if(!newStateAndEnergy || std::isnan(newStateAndEnergy->second))
                        continue;
                    registerVisitedState(newStateAndEnergy->first, newStateAndEnergy->second, false, false, true);
                    if(!result2 || isBetter(result2->second, newStateAndEnergy->second))
                        result2 = std::move(newStateAndEnergy);
                }

                if(!result2){
                    if(gotBetter)//the current state is the best state //@todo should be save some random state to restart the search from there? maybe setting a parameter for the number of attempts
                        break;

                    //go back to the best state and keep trying from there
                    if(result)
                        currentState = std::make_unique< std::pair<State, double> >( *result );
                    else
                        currentState = std::make_unique< std::pair<State, double> >( std::make_pair(start, currentEnergy) );
                    continue;
                }

                auto newEnergy = result2->second;

                ok = true;

                if(isBetter(bestEnergy, newEnergy)){
                    gotBetter = true;
                    result = std::make_unique< std::pair<State, double> >( *result2 );
                    bestEnergy = newEnergy;

                    registerVisitedState(result->first, bestEnergy, true, true, false);

                    if(breakFunction && (*breakFunction)(result->first, result->second)){
                        break;
                    }

                    currentState = std::move(result2);
                    continue;
                }

                if( std::exp( -std::fabs(currentState->second - newEnergy) / temperatureFunction(i) ) > gen_random_double(0, 1) ){
                    registerVisitedState(result2->first, newEnergy, false, true, false);

                    gotBetter = false;
                    currentState = std::move(result2);
                }
                else
                    registerVisitedState(result2->first, newEnergy, false, false, false);
            }
            catch(...){
                break;
            }
        }

        if(ok && !result)
            result = std::make_unique< std::pair<State, double> >( std::make_pair(start, currentEnergy) );
        return result;
    }
};

}
#endif // _AROLIB_TRACKSEQUENCING_SIM_ANNEALING_HPP
