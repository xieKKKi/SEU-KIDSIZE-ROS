#ifndef __BASIC_PARSER_HPP
#define __BASIC_PARSER_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <eigen3/Eigen/Dense>

namespace common
{
    namespace bpt = boost::property_tree;

    bool get_tree_from_file(const std::string &filename, bpt::ptree &pt);
    void write_tree_to_file(const std::string &filename, const bpt::ptree &pt);

    bool parse_file(const std::string &cfgname, bpt::ptree &pt);

    template<typename T, int size>
    inline Eigen::Matrix<T, size, 1> get_config_vector(bpt::ptree &pt, const std::string &keyword)
    {
        Eigen::Matrix<T, size, 1> res = Eigen::Matrix<T, size, 1>::Zero(size, 1);
        int i=0;
        bpt::ptree tpt = pt.get_child(keyword);
        for(auto &t:tpt)
        {
            if(i>=size) break;
            res[i++] = t.second.get_value<T>();
        }
        return res;
    }
}

#endif
