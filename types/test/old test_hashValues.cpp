int main(int argc, char *argv[])
{

    const size_t numSamples1 = 1e4;
    const size_t numSamples2 = 200;

    std::set<RoutePoint> pointsSet;
    std::set<RoutePoint> pointsSet2;
    std::vector<RoutePoint> pointsVec(numSamples1);
    std::map<RoutePoint, size_t> pointsMap;
    std::unordered_map<RoutePoint, size_t, RoutePoint::KeyHash> pointsHashMap;

    double vLimit = 5;
    double timestampRef = 0;
    for(size_t i = 0 ; i < numSamples2; ++i){
        RoutePoint rp;
        rp.point() = Point( 1.1*gen_random_int(-vLimit, vLimit), 1.2*gen_random_int(-vLimit, vLimit), 1.3*gen_random_int(-vLimit, vLimit) );
        rp.type = RoutePoint::intToRoutePointType( gen_random_int( RoutePoint::DEFAULT, RoutePoint::RESOURCE_POINT ) );
        rp.track_id = gen_random_int( -1, 5);
        rp.track_idx = gen_random_int( 0, 5);
        rp.time_stamp = timestampRef;
        timestampRef += 1.2;
        if(timestampRef > 5)
            timestampRef = 0;
        pointsSet2.insert( rp );
    }
    for(size_t i = 0 ; i < pointsVec.size() ; ++i){
        RoutePoint rp;
        rp.point() = Point( 1.1*gen_random_int(-vLimit, vLimit), 1.2*gen_random_int(-vLimit, vLimit), 1.3*gen_random_int(-vLimit, vLimit) );
        rp.type = RoutePoint::intToRoutePointType( gen_random_int( RoutePoint::DEFAULT, RoutePoint::RESOURCE_POINT ) );
        rp.track_id = gen_random_int( -1, 5);
        rp.track_idx = gen_random_int( 0, 5);

        rp.type = RoutePoint::intToRoutePointType( gen_random_int( RoutePoint::DEFAULT, RoutePoint::RESOURCE_POINT ) );
        rp.time_stamp = timestampRef;

        timestampRef += 1.2;
        if(timestampRef > 5)
            timestampRef = 0;
        if( pointsSet.find( rp ) != pointsSet.end() ){
            --i;
            continue;
        }
        if( pointsSet2.find( rp ) != pointsSet2.end() ){
            --i;
            continue;
        }
        pointsVec.at(i) = rp;
        pointsMap[ pointsVec.at(i) ] = i;
        pointsHashMap[ pointsVec.at(i) ] = i;
        pointsSet.insert( pointsVec.at(i) );
    }


    size_t errorCount_map = 0, errorCount_hashMap = 0;
    size_t errorCount2_map = 0, errorCount2_hashMap = 0;

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
    for(size_t i = 0 ; i < pointsVec.size() ; ++i){
        auto it = pointsMap.find(pointsVec.at(i));
        if( it == pointsMap.end() ){
            errorCount_map++;
            continue;
        }
        if( it->second != i ){
            errorCount_map++;
        }
    }
    double duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    std::cout << "Duration test1 map = " << duration << " us : errors = " << errorCount_map << std::endl;

    time_start = std::chrono::steady_clock::now();
    for(size_t i = 0 ; i < pointsVec.size() ; ++i){
        auto it = pointsHashMap.find(pointsVec.at(i));
        if( it == pointsHashMap.end() ){
            errorCount_hashMap++;
            continue;
        }
        if( it->second != i ){
            errorCount_hashMap++;
        }
    }
    duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    std::cout << "Duration test1 hash map = " << duration << " us : errors = " << errorCount_hashMap << std::endl;

    time_start = std::chrono::steady_clock::now();
    for(auto p : pointsSet2){
        auto it = pointsMap.find(p);
        if( it != pointsMap.end() ){
            errorCount2_map++;
        }
    }
    duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    std::cout << "Duration test2 map = " << duration << " us : errors = " << errorCount2_map << std::endl;

    time_start = std::chrono::steady_clock::now();
    for(auto p : pointsSet2){
        auto it = pointsHashMap.find(p);
        if( it != pointsHashMap.end() ){
            errorCount2_hashMap++;
        }
    }
    duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    std::cout << "Duration test2 hash map = " << duration << " us : errors = " << errorCount2_hashMap << std::endl;

    return errorCount_map+errorCount_hashMap+errorCount2_map+errorCount2_hashMap;

}