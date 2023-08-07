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
 
#ifndef _AROLIB_CELLSRANGESET_HPP
#define _AROLIB_CELLSRANGESET_HPP

#include <vector>
#include <set>
namespace arolib {
namespace gridmap{

/**
 * @brief Class to manage Cells' ranges (single y-indexes per x-index)
 *
 * It hold a list of (single) y-indexes for consecutive columns, where m_minX corresponds to the x-index of the first column. Ecah column corresponds to a x-index.
 */
class CellsRangeSet
{
public:
    /**
     * @brief Constructor.
     */
    CellsRangeSet();

    /**
     * @brief Initalize list of columns.
     *
     * @param _minX Minimum x-index (corresponding to the first column)
     * @param _size Number of columns
     */
    void init(const unsigned int & _minX, const unsigned int & _size);

    /**
     * @brief Initalize list of columns.
     *
     * @param _minX Minimum x-index (corresponding to the first column)
     * @param _maxX Maximum x-index (corresponding to the last column)
     */
    void initR(const unsigned int & _minX, const unsigned int & _maxX);

    /**
     * @brief Set the x-index corresponding to the first column.
     *
     * @param _minX Minimum x-index
     */
    inline void setMinX(const unsigned int & _minX) { m_minX = _minX; }

    /**
     * @brief Inserts a y-index to the column corresponding to a given x-index.
     *
     * @param _indX x-index
     * @param _indY y-index to be inserted
     * @return true on success
     */
    bool insertInColumn(const unsigned int & _indX, const int & _indY);



    /**
     * @brief Get the x-index corresponding to the first column.
     *
     * @return Minimum x-index
     */
    inline unsigned int minX() const { return m_minX; }

    /**
     * @brief Get the x-index corresponding to the last column.
     *
     * @return Maximum x-index
     */
    unsigned int maxX() const;

    /**
     * @brief Get the number of columns.
     *
     * @return Number of columns
     */
    inline unsigned int size() const { return m_list.size(); }

    /**
     * @brief Check if the list of columns is empty.
     *
     * @return True if empty
     */
    inline bool empty() const { return m_list.empty(); }

    /**
     * @brief Get the list of columns.
     *
     * @return List of columns
     */
    inline const std::vector< std::set<int> >& list() const { return m_list; }

    /**
     * @brief Get the set of y-indexes of the column corresponding to a x-index.
     *
     * @param _indX x-index
     * @return Set of y-indexes of the column
     */
    std::set<int> getColumn(const unsigned int & _indX) const;

    /**
     * @brief Get the minimum y-index of the column corresponding to a x-index.
     *
     * @param _indX x-index
     * @return Minimum y-index of the column
     */
    int getColumnMin(const unsigned int & _indX) const;

    /**
     * @brief Get the maximum y-index of the column corresponding to a x-index.
     *
     * @param _indX x-index
     * @return Maximum y-index of the column
     */
    int getColumnMax(const unsigned int & _indX) const;

    /**
     * @brief Get the index of the list of columns corresponding to a x-index.
     *
     * The list must be correctly initialized
     * @param _indX x-index
     * @return Index of the list of columns
     */
    inline unsigned int getVectorIndex(const unsigned int & _indX) const { return _indX - m_minX; }

    /**
     * @brief Check if a given x-index is within the x-index range if the list.
     *
     * @param _indX x-index
     * @return true if in range
     */
    bool checkRange(const unsigned int & _indX) const;

private:
    unsigned int m_minX; /**< x-index corresponding to the first column */
    std::vector< std::set<int> > m_list; /**< List of columns + column data */
};

}
}
#endif // _AROLIB_CELLSRANGESET_HPP
