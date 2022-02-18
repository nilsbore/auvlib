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

#include <data_tools/all_data.h>

using namespace std;
using namespace std_data;
using namespace all_data;

int main(int argc, char** argv)
{
    boost::filesystem::path path(argv[1]);

	cout << "Parsing file " << path << endl;

	all_nav_depth::EntriesT depths = parse_folder<all_nav_depth>(path);
	all_echosounder_depth::EntriesT echo_depths = parse_folder<all_echosounder_depth>(path);
	all_mbes_ping::PingsT pings = parse_file<all_mbes_ping>(path);
	all_nav_entry::EntriesT entries = parse_file<all_nav_entry>(path);
	all_sound_speed_profile::EntriesT sound_speed_profile = parse_file<all_sound_speed_profile>(path);
	all_raw_range_and_beam_angle::EntriesT raw_range_and_angle = parse_file<all_raw_range_and_beam_angle>(path);
	all_installation_param::EntriesT installation_param = parse_file<all_installation_param>(path);
	//all_nav_entry::EntriesT entries = parse_folder<all_nav_entry>(path);
	//all_mbes_ping::PingsT pings = parse_folder<all_mbes_ping>(path);

	cout << "Got " << pings.size() << " mbes pings, and " << entries.size() << " nav entries, and " << depths.size() << " nav depths.." << endl;
	//cout << "Got " << pings.size() << " mbes pings, and " << entries.size() << " nav entries" << endl;
	cout << "Got " << echo_depths.size() << " echo depth entries" << endl;

    /*
    for (const all_mbes_ping& ping : pings) {
        cout << "Got back ping " << ping.id_ << " with time: " << ping.time_string_ << endl;
    }
    */

    return 0;
}
