/*
 * Copyright 2021  DFKI GmbH
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
 
#ifndef _AROLIB_CELLSRANGE_HPP
#define _AROLIB_CELLSRANGE_HPP

#include <set>
#include <vector>
namespace arolib {
namespace gridmap{

/**
 * @brief Class to manage Cells' ranges (y-index ranges per x-index)
 *
 * It hold a list of y-index ranges for consecutive columns, where m_minX corresponds to the x-index of the first column. Each column corresponds to a x-index.
 */
class CellsRangeList
{
    /**
     * @brief Struct to sort columns based on their ranges
     */
    struct ColumnInfoCmpr{
        bool operator()(const std::pair<int,int>& a, const std::pair<int,int>& b){ return a.first <= b.first; }
    };

public:
    using ColumnInfo_t = std::set< std::pair<int,int>, ColumnInfoCmpr >;
    using List_t = std::vector< ColumnInfo_t >;

    /**
     * @brief Constructor.
     */
    CellsRangeList();

    /**
     * @brief Clear data.
     */
    void clear();


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

//    bool setColumn(const unsigned int & _indX, const int & _minY, const int & _maxY);
//    bool setColumn(const unsigned int & _indX, const std::pair<int,int> & _rangeY);

    /**
     * @brief Set the column info (y-index ranges) for a given x-index.
     *
     * @param _indX x-index
     * @param _rangesY Column info (y-index ranges)
     * @return true on success
     */
    bool setColumn(const unsigned int & _indX, const ColumnInfo_t & _rangesY);

    /**
     * @brief Append a y-index range to the column corresponding to a given x-index.
     *
     * @param _indX x-index
     * @param _minY Minimum y-index of the range
     * @param _maxY Maximum y-index of the range
     * @return true on success
     */
    bool appendToColumn(const unsigned int & _indX, const int & _minY, const int & _maxY);

    /**
     * @brief Append a y-index range to the column corresponding to a given x-index.
     *
     * @param _indX x-index
     * @param _rangeY y-index range <min index, max index>
     * @return true on success
     */
    bool appendToColumn(const unsigned int & _indX, const std::pair<int,int> & _rangeY);


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
    inline const List_t& list() const { return m_list; }

    /**
     * @brief Get the index of the list of columns corresponding to a x-index.
     *
     * The list must be correctly initialized
     * @param _indX x-index
     * @return Index of the list of columns
     */
    inline unsigned int getVectorIndex(const unsigned int & _indX) const { return _indX - m_minX; }

//    std::pair<int,int> getColumn(const unsigned int & _indX) const;
//    bool getColumn(const unsigned int & _indX, int & _minY, int & _maxY) const;

    /**
     * @brief Get the information (y-index ranges) of the column corresponding to a x-index.
     *
     * @param _indX x-index
     * @return Information (y-index ranges) of the column
     */
    ColumnInfo_t getColumn(const unsigned int & _indX) const;

    /**
     * @brief Get the information (y-index ranges) of the column corresponding to a x-index.
     *
     * @param _indX x-index
     * @param  [out]rangesY Information (y-index ranges) of the column
     * @return true on success
     */
    bool getColumn(const unsigned int & _indX, ColumnInfo_t& rangesY) const;

    /**
     * @brief Check if a given x-index is within the x-index range if the list.
     *
     * @param _indX x-index
     * @return true if in range
     */
    bool checkRange(const unsigned int & _indX) const;

private:
    unsigned int m_minX; /**< x-index corresponding to the first column */
    List_t m_list; /**< List of columns + column data */
};

}
}

#endif // _AROLIB_CELLSRANGE_HPP
