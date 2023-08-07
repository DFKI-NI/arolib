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
 
#include "arolib/cartography/cellsrangeset.hpp"

#include <iostream>

namespace arolib {
namespace gridmap{

CellsRangeSet::CellsRangeSet():
    m_minX(0)
{
    m_list.clear();
}

unsigned int CellsRangeSet::maxX() const
{
    if ( m_list.empty() )
        return 0;
    return m_minX + m_list.size() - 1;
}

void CellsRangeSet::init(const unsigned int& _minX, const unsigned int& _size)
{
    m_minX = _minX;
    m_list.clear();
    m_list.resize(_size);
}

void CellsRangeSet::initR(const unsigned int & _minX, const unsigned int & _maxX)
{
    m_minX = _minX;
    m_list.clear();
    if (_maxX < _minX)
        return;
    m_list.resize( _maxX-_minX+1 );
}

bool CellsRangeSet::insertInColumn(const unsigned int & _indX, const int &_indY)
{
    if ( !checkRange(_indX) )
        return false;
    m_list.at(_indX - m_minX).insert(_indY);
    return true;
}

std::set<int> CellsRangeSet::getColumn(const unsigned int &_indX) const
{
    if ( !checkRange(_indX) ){
        std::set<int> ret;
        return ret;
    }
    return m_list.at(_indX - m_minX);
}

int CellsRangeSet::getColumnMin(const unsigned int &_indX) const
{
    if ( !checkRange(_indX) || m_list.at(_indX - m_minX).empty() || m_list.at(_indX - m_minX).empty() )
        return -1;
    return *(m_list.at(_indX - m_minX).begin());
}

int CellsRangeSet::getColumnMax(const unsigned int &_indX) const
{
    if ( !checkRange(_indX) || m_list.at(_indX - m_minX).empty() || m_list.at(_indX - m_minX).empty() )
        return -1;
    return *(m_list.at(_indX - m_minX).rbegin());
}


bool CellsRangeSet::checkRange(const unsigned int & _indX) const
{
    long index = (long)_indX - m_minX;
    return ( index >= 0 && index < m_list.size() );
}

}
}
