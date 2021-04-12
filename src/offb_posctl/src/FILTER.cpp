//
// Created by zm on 19-3-11.
//

#include "FILTER.h"
#include <time.h>
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
#include <list>


FILTER::FILTER(int filterlength){
    filter_list.push_back(std::make_pair(0.0f, 0.0f));
    filter_data = 0;
    Output_filter = 0;
    start_filter_flag = false;
    filter_length=filterlength;
    derivation_legngth=filterlength;

}

float FILTER::satfunc(float data, float Max, float Thres)
{
    if (fabs(data)<Thres)
        return 0;
    else if(fabs(data)>Max){
        return (data>0)?Max:-Max;
    }
    else{
        return data;
    }
}

bool FILTER::filter_input(float data2fliter, float curtime)
{
    if(filter_list.size() == 1)
    {
        delta_time = curtime;
    }
    else{
        delta_time = curtime - filter_list.rbegin()->first;
//        std::cout<<delta_time<<endl;
    }

    filter_data = data2fliter;
    if(filter_list.size() < filter_length){
        filter_list.push_back(std::make_pair(curtime, filter_data));
    }
    else{
        std::vector<std::pair<float, float > > ::iterator fil_iter = filter_list.begin();
        filter_list.erase(fil_iter);
        std::pair<float, float > temp_iter(curtime, filter_data);
        filter_list.push_back(temp_iter);
    }
    return true;
}

void FILTER::filter_output()
{
    if(filter_list.size() < filter_length || ! start_filter_flag){
        Output_filter = 0;
    }
    else{
        std::vector<std::pair<float, float> >::iterator filter_k;
        float filter_sum = 0;
        for(filter_k = filter_list.begin(); filter_k != filter_list.end(); ++ filter_k){
            filter_sum = filter_sum + filter_k->second;
        }
        Output_filter = filter_sum * delta_time/(filter_list.back().first - filter_list.front().first);

    }
}

/**
 * filter by average a specific length
 * @param data to be filted
 */
double FILTER::filter(double data)
{
    double tempsum=0.0;
    if(filterlist.empty())
    {
        filterlist.push_back(data);
        return data;
    } else
    {
        filterlist.push_back(data);
        if(filterlist.size()>filter_length)
        {
            filterlist.pop_front();
        }
        std::list<double > ::iterator itor=filterlist.begin();
        while(itor!=filterlist.end())
        {
//            cout<<"*itor~~~~~~----:"<<*itor<<endl;
            tempsum=tempsum+(*itor++);
        }
        return tempsum/filterlist.size();
    }
}

double FILTER::derivation(float data2derivation,float curtime)
{
    if(derivation_list.size() < derivation_legngth)
    {
        derivation_list.push_back(std::make_pair(curtime, data2derivation));
        if(derivation_list.size()>=derivation_legngth)
        {
            return (derivation_list.back().second-derivation_list.front().second)/(derivation_list.back().first-derivation_list.front().first);
        }
        return 0.0;
    }
    else{
        std::vector<std::pair<float, float > > ::iterator derivation_iter = derivation_list.begin();
        derivation_list.erase(derivation_iter);
        derivation_list.push_back(std::make_pair(curtime, data2derivation));
//        std::cout<<"(derivation_list.back().second-derivation_list.front().second): "<<(derivation_list.back().second-derivation_list.front().second)
//        <<"  (derivation_list.back().first-derivation_list.front().first): "<<(derivation_list.back().first-derivation_list.front().first)<< std::endl;
        return (derivation_list.back().second-derivation_list.front().second)/(derivation_list.back().first-derivation_list.front().first);
    }

}