/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <data_tools/std_data.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/date_time.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

using namespace std;

namespace std_data {

// instantiate all versions needed to read the structs
template nav_entry::EntriesT read_data<nav_entry::EntriesT>(const boost::filesystem::path& path);
template mbes_ping::PingsT read_data<mbes_ping::PingsT>(const boost::filesystem::path& path);
template sss_ping::PingsT read_data<sss_ping::PingsT>(const boost::filesystem::path& path);
template pt_submaps read_data<pt_submaps>(const boost::filesystem::path& path);

// instantiate all versions needed to write the structs
template void write_data<nav_entry::EntriesT>(nav_entry::EntriesT& data, const boost::filesystem::path& path);
template void write_data<mbes_ping::PingsT>(mbes_ping::PingsT& data, const boost::filesystem::path& path);
template void write_data<sss_ping::PingsT>(sss_ping::PingsT& data, const boost::filesystem::path& path);
template void write_data<pt_submaps>(pt_submaps& data, const boost::filesystem::path& path);

std::string time_string_from_time_stamp(long long time_stamp_)
{
    // TODO: can this be a static variable instead? we could also use that in other libraries
    const boost::posix_time::ptime epoch = boost::posix_time::time_from_string("1970-01-01 00:00:00.000");
    boost::posix_time::ptime t = epoch + boost::posix_time::milliseconds(time_stamp_);
    stringstream time_ss;
    time_ss << t;
    string time_string_ = time_ss.str();
    return time_string_;
}

} // namespace std_data

