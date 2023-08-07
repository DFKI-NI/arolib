/*
 * Copyright 2023  DFKI GmbH
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
 
#ifndef AROLIB_ASTAR_SUCCESSOR_CHECKERS_HPP
#define AROLIB_ASTAR_SUCCESSOR_CHECKERS_HPP

#include <functional>
#include "arolib/planning/path_search/astar.hpp"

namespace arolib{

//-----------------------------------AstarCustomSuccessorChecker------------------------

/**
 * @brief Custom Astar SuccessorChecker
 */
class AstarCustomSuccessorChecker: public Astar::ISuccesorChecker{
public:

    using CallbackFunct1Type = std::function< Astar::ISuccesorChecker::Validity (const Astar::ISuccesorChecker::IsSuccessorValidParams&)>;
    using CallbackFunct2Type = std::function< double (const Astar::ISuccesorChecker::GetMinDurationAtEdgeParams&)>;

    /**
     * @brief Constructor.
     * @param cb Callback function for 'isSuccessorValid'
     */
    AstarCustomSuccessorChecker(const CallbackFunct1Type& cb_isSuccessorValid, const CallbackFunct2Type& cb_getMinDurationAtEdge = [](...){return -1;});

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams &params) const override;

    /**
     * @brief Get the minimum duration [s] for a machine planning to go over the edge
     * @param params Input parameters
     * @return Minimum duration [s]. If < 0, no minimum duration applies (disregarded during search)
    */
    virtual double getMinDurationAtEdge(const GetMinDurationAtEdgeParams& params) const override;

private:
    CallbackFunct1Type m_cb_isSuccessorValid;  /**< Callback to isSuccessorValid */
    CallbackFunct2Type m_cb_getMinDurationAtEdge;  /**< Callback to getMinDurationAtEdge */
};


//-----------------------------------AstarSuccessorChecker_SuccessorTimestamp------------------------

/**
 * @brief Astar SuccessorChecker to check that the successor's timestamp is higher or lower than a given timestamp when the sucessor 'belongs' to a machine in a given restricted list
 *
 * Used to check that a machine does not visit unworked or worked vertices when the working of the successor vertex is dependent on the machine being planned (e.g. the machine being planned is a transport vehicle that will assist the harvestier that works that vertex)
 */
class AstarSuccessorChecker_SuccessorTimestamp: public Astar::ISuccesorChecker{
public:

    enum CheckOption{
        INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP, //the successor (worked) timestamp must be <= to the given timestamp limit (the successor must be worked before the given timestamp)
        INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP //the successor (worked) timestamp must be >= to the given timestamp limit (the successor must be worked after the given timestamp)
    };

    /**
     * @brief Constructor.
     *
     * @param Timestamp timestamp to be compared to the successor's timestamp
     * @param checkOption Type of check
     * @param restrictedMachineIds The check will be done only if the working machine corresponding to the successor is in this set
     */
    AstarSuccessorChecker_SuccessorTimestamp(double timestamp, CheckOption checkOption, const std::set<MachineId_t> &restrictedMachineIds = {Machine::AllMachineIds});

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams& params) const override;

    /**
     * @brief Get the minimum duration [s] for a machine planning to go over the edge
     * @param params Input parameters
     * @return Minimum duration [s]. If < 0, no minimum duration applies (disregarded during search)
    */
    virtual double getMinDurationAtEdge(const GetMinDurationAtEdgeParams& params) const override;

protected:
    double m_timestamp;  /**< Timestamp to be compared to the successor's timestamp */
    CheckOption m_checkOption;  /**< Type of check */
    std::set<MachineId_t> m_restrictedMachineIds;  /**< The check will be done only if the working machine corresponding to the successor is in this set */
};


//-----------------------------------AstarSuccessorChecker_GoalMaxTime------------------------


/**
 * @brief Astar SuccessorChecker to check that the goal is reached before a given timestamp limit
 */
class AstarSuccessorChecker_GoalMaxTime: public Astar::ISuccesorChecker{
public:

    /**
     * @brief Constructor.
     *
     * @param max_time_goal Limit time(-stamp) to reach the search goal
     */
    AstarSuccessorChecker_GoalMaxTime(float max_time_goal);

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams& params) const override;

protected:
    float m_max_time_goal;  /**< Limit time(-stamp) to reach the search goal */
};


//-----------------------------------AstarSuccessorChecker_VertexExcludeSet------------------------


/**
 * @brief Astar SuccessorChecker to check that the edge is not in a given exclude set
 */
class AstarSuccessorChecker_VertexExcludeSet: public Astar::ISuccesorChecker{
public:
    using ExceptFct = std::function< bool (const Astar::ISuccesorChecker::IsSuccessorValidParams&)>;


    /**
     * @brief Constructor.
     *
     * @param excludeSet Vertices to be excluded from the search
     */
    AstarSuccessorChecker_VertexExcludeSet(const std::set<DirectedGraph::vertex_t>& excludeSet);

    /**
     * @brief Constructor.
     *
     * @param excludeSet Vertices to be excluded from the search
     * @param exceptIf If the function returns true, the vertex will not be excluded
     */
    AstarSuccessorChecker_VertexExcludeSet(const std::set<DirectedGraph::vertex_t>& excludeSet,
                                           const ExceptFct& exceptIf);

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const IsSuccessorValidParams& params) const override;


protected:
    std::set<DirectedGraph::vertex_t> m_excludeSet;  /**< Vertices to be excluded from the search */
    bool m_hasExceptions = false;  /**< Flag stating if there are exceptions */
    ExceptFct m_except;  /**< Callback to check if a given vertex in the exclude set must de disregarded (i.e., not excluded) */
};


//-----------------------------------AstarSuccessorChecker_VertexExcludeSet_Exceptions1------------------------


/**
 * @brief Astar SuccessorChecker to check that the successor is not in a given exclude set with some predefined exceptions
 */
class AstarSuccessorChecker_VertexExcludeSet_Exceptions1: public AstarSuccessorChecker_VertexExcludeSet{
public:

    /**
     * @brief Constructor.
     *
     * @param excludeSet Vertices to be excluded from the search
     * @param exceptIfVtFromIsInitPoint Exception to the rule: If vt_from corresponds to a (machine) initial point and vt_to corresponds to a route_point of type headland.
     * @param exceptIfVtFromIsAccessPoint Exception to the rule: If vt_from corresponds to an access point and vt_to corresponds to a route_point of type headland.
     * @param exceptIfVtFromIsResourcePoint Exception to the rule: If vt_from corresponds to a resource point and vt_to corresponds to a route_point of type headland.
     */
    AstarSuccessorChecker_VertexExcludeSet_Exceptions1(const std::set<DirectedGraph::vertex_t>& excludeSet,
                                                       bool exceptIfVtFromIsInitPoint = true,
                                                       bool exceptIfVtFromIsAccessPoint = true,
                                                       bool exceptIfVtFromIsResourcePoint = true);

private:

    /**
     * @brief Check if a given vertex in the exclude set must de disregarded (i.e., not excluded).
     *
     * @param exceptIfVtFromIsInitPoint Exception to the rule: If vt_from corresponds to a (machine) initial point and vt_to corresponds to a route_point of type headland.
     * @param exceptIfVtFromIsAccessPoint Exception to the rule: If vt_from corresponds to an access point and vt_to corresponds to a route_point of type headland.
     * @param exceptIfVtFromIsResourcePoint Exception to the rule: If vt_from corresponds to a resource point and vt_to corresponds to a route_point of type headland.
     */
    static bool except(const IsSuccessorValidParams& params,
                       bool exceptIfVtFromIsInitPoint,
                       bool exceptIfVtFromIsAccessPoint,
                       bool exceptIfVtFromIsResourcePoint);
};


//-----------------------------------AstarSuccessorChecker_EdgeExcludeSet------------------------


/**
 * @brief Astar SuccessorChecker to check that the edge is not in a given exclude set
 */
class AstarSuccessorChecker_EdgeExcludeSet: public Astar::ISuccesorChecker{
public:
    using ExceptFct = std::function< bool (const Astar::ISuccesorChecker::IsSuccessorValidParams&)>;


    /**
     * @brief Constructor.
     *
     * @param excludeSet Edges to be excluded from the search
     */
    AstarSuccessorChecker_EdgeExcludeSet(const std::set<DirectedGraph::edge_t>& excludeSet);

    /**
     * @brief Constructor.
     *
     * @param excludeSet Edges to be excluded from the search
     * @param exceptIf If the function returns true, the edge will not be excluded
     */
    AstarSuccessorChecker_EdgeExcludeSet(const std::set<DirectedGraph::edge_t>& excludeSet,
                                         const ExceptFct& exceptIf);

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const IsSuccessorValidParams& params) const override;

protected:
    std::set<DirectedGraph::edge_t> m_excludeSet;  /**< Edges to be excluded from the search */
    bool m_hasExceptions = false;  /**< Flag stating if there are exceptions */
    ExceptFct m_except;  /**< Callback to check if a given edge in the exclude set must de disregarded (i.e., not excluded) */
};


//-----------------------------------AstarSuccessorChecker_FutureVisits------------------------


/**
 * @brief Astar SuccessorChecker to check for future visits in the vertices (for collition avoidance)
 *
 * Probem: since we dont know how much time the machine has to wait in the current node, it might happen that it becomes busy (based on the waiting/busy time of successor nodes
 * This workaround solves part of the problem, but it is not ideal:
 * If the current node and successor will be visited in the future by a restricted machine, don't expand
 * @todo maybe we can make a preemptive chech of the successors of the successors to check if the is at least one that is not restricted in the future. If all are restricted, adjust the busy time to let the restricted machines go ahead while this machine waits
 */
class AstarSuccessorChecker_FutureVisits: public Astar::ISuccesorChecker{
public:

    /**
     * @brief Constructor.
     *
     * @param restrictedMachineIds <machine id, last known/planned timestamp>. The check will return INVALID iif the machine visiting the nodes is in this map.
     */
    AstarSuccessorChecker_FutureVisits(const std::map<MachineId_t, double> &restrictedMachineIds);

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const IsSuccessorValidParams& params) const override;

protected:
    std::map<MachineId_t, double> m_restrictedMachineIds;  /**< The check will return INVALID iif the machine visiting the nodes is in this map. */
};


//-----------------------------------AstarSuccessorChecker_FutureVisits2------------------------


/**
 * @brief Astar SuccessorChecker to check for future visits in the vertices (for collition avoidance)
 *
 * Probem: since we dont know how much time the machine has to wait in the current node, it might happen that it becomes busy (based on the waiting/busy time of successor nodes
 * This workaround attemps to 'get away from the way' before the other machines drive passed: checks if the current node is free during the busy time and try to cause the machine to get out of the way
 * A possible solution could be to reopen affected nodes, backpropagaiting the waiting/busy times for the conflicting successors.
 * @warning Fails many times
 */
class AstarSuccessorChecker_FutureVisits2: public Astar::ISuccesorChecker{
public:

    /**
     * @brief Constructor.
     *
     * @param restrictedMachineIds The check will return INVALID iif the machines visiting the nodes is in this set
     */
    AstarSuccessorChecker_FutureVisits2();

    /**
     * @brief Check if a sucesor is valid during AStar search
     * @sa Astar::ISuccesorChecker::isSuccessorValid
    */
    virtual Astar::ISuccesorChecker::Validity isSuccessorValid(const IsSuccessorValidParams& params) const override;
};

}

#endif // AROLIB_ASTAR_SUCCESSOR_CHECKERS_HPP
