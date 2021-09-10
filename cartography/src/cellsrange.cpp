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
 
#include "arolib/cartography/cellsrange.hpp"

namespace arolib {
namespace gridmap{

CellsRangeList::CellsRangeList():
    m_minX(0)
{
    m_list.clear();
}

void CellsRangeList::clear()
{
    m_minX = 0;
    m_list.clear();
}

unsigned int CellsRangeList::maxX() const
{
    if ( m_list.empty() )
        return 0;
    return m_minX + m_list.size() - 1;
}

void CellsRangeList::init(const unsigned int & _minX, const unsigned int & _size)
{
    m_minX = _minX;
    m_list.clear();
    m_list.resize(_size);
}

void CellsRangeList::initR(const unsigned int & _minX, const unsigned int & _maxX)
{
    m_minX = _minX;
    m_list.clear();
    if (_maxX < _minX)
        return;
    m_list.resize( _maxX-_minX+1);
}

//bool CellsRangeList::setColumn(const unsigned int & _indX, const int &_minY, const int &_maxY)
//{
//    if ( !checkRange(_indX) )
//        return false;
//    if (_maxY < _minY)
//        return false;
//    m_list.at(_indX - m_minX) = std::make_pair(_minY,_maxY);
//    return true;

//}

//bool CellsRangeList::setColumn(const unsigned int & _indX, const std::pair<int, int> &_rangeY)
//{
//    if ( !checkRange(_indX) )
//        return false;
//    if (_rangeY.second < _rangeY.first)
//        return false;
//    m_list.at(_indX - m_minX) = _rangeY;
//    return true;

//}

bool CellsRangeList::setColumn(const unsigned int &_indX, const CellsRangeList::ColumnInfo_t &_rangesY)
{
    if ( !checkRange(_indX) )
        return false;
    for(auto range : _rangesY){
        if (range.second < range.first)
            return false;
    }

    m_list.at(_indX - m_minX) = _rangesY;
    return true;

}

bool CellsRangeList::appendToColumn(const unsigned int &_indX, const int &_minY, const int &_maxY)
{
    if ( !checkRange(_indX) )
        return false;
    if (_maxY < _minY)
        return false;
    m_list.at(_indX - m_minX).insert( std::make_pair(_minY,_maxY) );
    return true;
}

bool CellsRangeList::appendToColumn(const unsigned int &_indX, const std::pair<int, int> &_rangeY)
{

    if ( !checkRange(_indX) )
        return false;
    if (_rangeY.second < _rangeY.first)
        return false;
    m_list.at(_indX - m_minX).insert( _rangeY );
    return true;
}

//std::pair<int, int> CellsRangeList::getColumn(const unsigned int &_indX) const
//{
//    if ( !checkRange(_indX) )
//        return std::make_pair(-1,-1);
//    return m_list.at(_indX - m_minX);
//}

//bool CellsRangeList::getColumn(const unsigned int &_indX, int &_minY, int &_maxY) const
//{
//    if ( !checkRange(_indX) )
//        return false;
//    _minY = m_list.at(_indX - m_minX).first;
//    _maxY = m_list.at(_indX - m_minX).second;

//    return true;
//}

CellsRangeList::ColumnInfo_t CellsRangeList::getColumn(const unsigned int &_indX) const
{
    if ( !checkRange(_indX) )
        return {};
    return m_list.at(_indX - m_minX);
}

bool CellsRangeList::getColumn(const unsigned int &_indX, CellsRangeList::ColumnInfo_t &rangesY) const
{
    rangesY.clear();
    if ( !checkRange(_indX) )
        return false;
    rangesY = m_list.at(_indX - m_minX);
    return true;
}


bool CellsRangeList::checkRange(const unsigned int & _indX) const
{
    long index = (long)_indX - m_minX;
    return ( index >= 0 && index < m_list.size() );
}

}
}
