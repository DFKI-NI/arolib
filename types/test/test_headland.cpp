/*
 * Copyright 2022  DFKI GmbH
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
 
#include <boost/test/unit_test.hpp>
#include "arolib/types/headland.hpp"

BOOST_AUTO_TEST_SUITE(test_headland)
BOOST_AUTO_TEST_CASE(test_ph_sequences)
{
    auto doTest = [](std::vector<arolib::PartialHeadland>& hls, int ind_from, int ind_to, int expectedSize) -> bool{
        auto seqs = arolib::PartialHeadland::getHeadlandConnectionSequences(hls, ind_from, ind_to);
        std::cout << "HLs(s=" << hls.size() << ") :: " << ind_from << " -> " << ind_to << "  :: " << seqs.size() << (seqs.size() == expectedSize ? " == " : " != ") << expectedSize << std::endl;
        for(auto& seq : seqs){
            std::cout << "\t";
            for(auto i : seq)
                std::cout << i << " ";
            std::cout << std::endl;
        }
        std::cout << std::endl;
        return seqs.size() == expectedSize;
    };

    std::vector<arolib::PartialHeadland> hls1(4), hls2(2);

    for(size_t i = 0 ; i < hls1.size() ; ++i)
        hls1.at(i).id = i;
    for(size_t i = 0 ; i < hls2.size() ; ++i)
        hls2.at(i).id = i;

    hls1.at(2).connectingHeadlandIds = hls1.at(3).connectingHeadlandIds = std::make_pair(0,1);
    hls2.at(1).connectingHeadlandIds = std::make_pair(0,0);

    std::cout << "Running test for partial-headland sequences..." << std::endl << std::endl;

    BOOST_TEST( doTest(hls1, 0, 0, 3) );
    BOOST_TEST( doTest(hls1, 1, 1, 3) );
    BOOST_TEST( doTest(hls1, 2, 2, 3) );
    BOOST_TEST( doTest(hls1, 0, 1, 2) );
    BOOST_TEST( doTest(hls1, 2, 1, 2) );
    BOOST_TEST( doTest(hls1, 2, 3, 2) );

    BOOST_TEST( doTest(hls2, 0, 0, 2) );
    BOOST_TEST( doTest(hls2, 1, 1, 2) );
    BOOST_TEST( doTest(hls2, 0, 1, 1) );
    BOOST_TEST( doTest(hls2, 1, 0, 1) );

    std::cout << "Finished running test for partial-headland sequences." << std::endl << std::endl;
}
BOOST_AUTO_TEST_SUITE_END()
