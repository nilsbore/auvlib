# pydata_tools

# pydata_tools.gsf_data
Basic utilities for working with the .gsf file format
## convert_pings
```python
convert_pings(self)
```
convert_pings(arg0: List[pydata_tools.gsf_data.gsf_mbes_ping]) -> List[mbes_ping]

Convert gsf_mbes_ping::EntriesT to mbes_ping::EntriesT

## gsf_mbes_ping
```python
gsf_mbes_ping(self)
```
Class for the gsf multibeam type
### amplitudes
Member
### beam_angles
Member
### beams
Member
### depth_
Member
### distances
Member
### first_in_file_
Member
### heading_
Member
### lat_
Member
### long_
Member
### parse_file
```python
gsf_mbes_ping.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_mbes_ping]

Parse gsf_mbes_ping from .gsf file

### parse_folder
```python
gsf_mbes_ping.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_mbes_ping]

Parse gsf_mbes_ping from folder of .gsf files

### pitch_
Member
### read_data
```python
gsf_mbes_ping.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_mbes_ping]

Read gsf_mbes_ping::PingsT from .cereal file

### roll_
Member
### time_stamp_
Member
### time_string_
Member
### travel_times
Member
## gsf_nav_entry
```python
gsf_nav_entry(self)
```
Class for the gsf nav entry type
### altitude
Member
### id_
Member
### lat_
Member
### long_
Member
### parse_file
```python
gsf_nav_entry.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_nav_entry]

Parse gsf_nav_entry from .gsf file

### parse_folder
```python
gsf_nav_entry.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_nav_entry]

Parse gsf_nav_entry from folder of .gsf files

### pitch_
Member
### pos_
Member
### read_data
```python
gsf_nav_entry.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_nav_entry]

Read gsf_nav_entry::EntriesT from .cereal file

### roll_
Member
### time_stamp_
Member
### time_string_
Member
### yaw_
Member
## gsf_sound_speed
```python
gsf_sound_speed(self)
```
Class for the gsf sound speed type
### below_speed
Member
### near_speed
Member
### parse_file
```python
gsf_sound_speed.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_sound_speed]

Parse gsf_sound_speed from .gsf file

### parse_folder
```python
gsf_sound_speed.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_sound_speed]

Parse gsf_sound_speed from folder of .gsf files

### read_data
```python
gsf_sound_speed.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.gsf_data.gsf_sound_speed]

Read gsf_sound_speed::SpeedsT from .cereal file

### time_stamp_
Member
### time_string_
Member
## write_data
```python
write_data(self)
```
write_data(*args, **kwargs)
Overloaded function.

1. write_data(arg0: List[pydata_tools.gsf_data.gsf_mbes_ping], arg1: unicode) -> None

Write gsf_mbes_ping::PingsT to .cereal file

2. write_data(arg0: List[pydata_tools.gsf_data.gsf_nav_entry], arg1: unicode) -> None

Write gsf_nav_entry::EntriesT to .cereal file

3. write_data(arg0: List[pydata_tools.gsf_data.gsf_sound_speed], arg1: unicode) -> None

Write gsf_sound_speed::SpeedsT to .cereal file

# pydata_tools.xtf_data
Basic utilities for working with the xtf file format
## make_waterfall_image
```python
make_waterfall_image(self)
```
make_waterfall_image(arg0: List[pydata_tools.xtf_data.xtf_sss_ping]) -> cv::Mat

Create an opencv waterfall image from xtf_sss_ping::PingsT

## write_data
```python
write_data(self)
```
write_data(arg0: List[pydata_tools.xtf_data.xtf_sss_ping], arg1: unicode) -> None

Write xtf pings to .cereal file

## xtf_sss_ping
```python
xtf_sss_ping(self)
```
Class for xtf sidescan type
### first_in_file_
Member
### heading_
Member
### lat_
Member
### long_
Member
### parse_file
```python
xtf_sss_ping.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.xtf_data.xtf_sss_ping]

Parse xtf_sss_ping from .xtf file

### parse_folder
```python
xtf_sss_ping.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.xtf_data.xtf_sss_ping]

Parse xtf_sss_ping from folder of .xtf files

### pitch_
Member
### pos_
Member
### read_data
```python
xtf_sss_ping.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.xtf_data.xtf_sss_ping]

Read xtf_sss_ping::PingsT from .cereal file

### roll_
Member
### sound_vel_
Member
### time_stamp_
Member
### time_string_
Member
## xtf_sss_ping_side
```python
xtf_sss_ping_side(self)
```
Class for one xtf sidescan side
### beam_width
Member
### pings
Member
### slant_range
Member
### tilt_angle
Member
### time_duration
Member
# pydata_tools.all_data
Basic utilities for working with the .all file format
## all_echosounder_depth
```python
all_echosounder_depth(self)
```
Class for the all single-beam echosounder depth
### depth_
Member
### first_in_file_
Member
### id_
Member
### parse_file
```python
all_echosounder_depth.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.all_data.all_echosounder_depth]

Parse all_echosounder_depth from .all file

### parse_folder
```python
all_echosounder_depth.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.all_data.all_echosounder_depth]

Parse all_echosounder_depth from folder of .all files

### read_data
```python
all_echosounder_depth.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.all_data.all_echosounder_depth]

Read all_echosounder_depth::EntriesT from .cereal file

### time_stamp_
Member
### time_string_
Member
## all_mbes_ping
```python
all_mbes_ping(self)
```
Class for the all multibeam type
### beams
Member
### first_in_file_
Member
### heading_
Member
### id_
Member
### parse_file
```python
all_mbes_ping.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.all_data.all_mbes_ping]

Parse all_mbes_ping from .all file

### parse_folder
```python
all_mbes_ping.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.all_data.all_mbes_ping]

Parse all_mbes_ping from folder of .all files

### read_data
```python
all_mbes_ping.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.all_data.all_mbes_ping]

Read all_mbes_ping::PingsT from .cereal file

### reflectivities
Member
### sound_vel_
Member
### time_stamp_
Member
### time_string_
Member
### transducer_depth_
Member
## all_nav_depth
```python
all_nav_depth(self)
```
Class for the all nav depth entry
### first_in_file_
Member
### height
Member
### height_type
Member
### id_
Member
### parse_file
```python
all_nav_depth.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.all_data.all_nav_depth]

Parse all_nav_depth from .all file

### parse_folder
```python
all_nav_depth.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.all_data.all_nav_depth]

Parse all_nav_depth from folder of .all files

### read_data
```python
all_nav_depth.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.all_data.all_nav_depth]

Read all_nav_depth::EntriesT from .cereal file

### time_stamp_
Member
### time_string_
Member
## all_nav_entry
```python
all_nav_entry(self)
```
Class for the all nav entry
### course_over_ground_
Member
### depth_
Member
### first_in_file_
Member
### heading_
Member
### id_
Member
### lat_
Member
### long_
Member
### parse_file
```python
all_nav_entry.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.all_data.all_nav_entry]

Parse all_nav_entry from .all file

### parse_folder
```python
all_nav_entry.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.all_data.all_nav_entry]

Parse all_nav_entry from folder of .all files

### read_data
```python
all_nav_entry.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.all_data.all_nav_entry]

Read all_nav_entry::EntriesT from .cereal file

### time_stamp_
Member
### time_string_
Member
## convert_matched_entries
```python
convert_matched_entries(self)
```
convert_matched_entries(arg0: List[pydata_tools.all_data.all_mbes_ping], arg1: List[pydata_tools.all_data.all_nav_entry]) -> List[pydata_tools.all_data.all_mbes_ping]

Matches xtf_sss_ping::PingsT and csv_nav_entry::EntriesT and assign pos data to pings

## write_data
```python
write_data(self)
```
write_data(*args, **kwargs)
Overloaded function.

1. write_data(arg0: List[pydata_tools.all_data.all_mbes_ping], arg1: unicode) -> None

Write all_mbes_ping::PingsT to .cereal file

2. write_data(arg0: List[pydata_tools.all_data.all_nav_entry], arg1: unicode) -> None

Write all_nav_entry::EntriesT to .cereal file

3. write_data(arg0: List[pydata_tools.all_data.all_nav_depth], arg1: unicode) -> None

Write all_nav_depth::EntriesT to .cereal file

4. write_data(arg0: List[pydata_tools.all_data.all_echosounder_depth], arg1: unicode) -> None

Write all_echosounder_depth::EntriesT to .cereal file

# pydata_tools.data_structures
Standard interfaces for working with different kinds of data. All data types should be converted into these before processing
## mbes_ping
```python
mbes_ping(self)
```
Standard class interface for working with multibeam data
### back_scatter
Member
### beams
Member
### first_in_file_
Member
### heading_
Member
### heave_
Member
### id_
Member
### parse_file
```python
mbes_ping.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.data_structures.mbes_ping]

Parse mbes_ping from an ASCII file exported from NaviEdit

### parse_folder
```python
mbes_ping.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.data_structures.mbes_ping]

Parse mbes_ping from folder of ASCII files exported from NaviEdit

### pitch_
Member
### pos_
Member
### read_data
```python
mbes_ping.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.data_structures.mbes_ping]

Read mbes_ping::PingsT from .cereal file

### roll_
Member
### time_stamp_
Member
### time_string_
Member
## nav_entry
```python
nav_entry(self)
```
Standard class interface for working with navigation data
### first_in_file_
Member
### parse_file
```python
nav_entry.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.data_structures.nav_entry]

Parse nav_entry from an ASCII file exported from NaviEdit

### parse_folder
```python
nav_entry.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.data_structures.nav_entry]

Parse nav_entry from folder of ASCII files exported from NaviEdit

### pos_
Member
### read_data
```python
nav_entry.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.data_structures.nav_entry]

Read nav_entry::Entries from .cereal file

### time_stamp_
Member
### time_string_
Member
## write_data
```python
write_data(self)
```
write_data(*args, **kwargs)
Overloaded function.

1. write_data(arg0: List[pydata_tools.data_structures.mbes_ping], arg1: unicode) -> None

Write mbes_ping::PingsT to .cereal file

2. write_data(arg0: List[pydata_tools.data_structures.nav_entry], arg1: unicode) -> None

Write nav_entry::EntriesT to .cereal file

# pydata_tools.csv_data
Basic utilities for working with csv based data
## convert_matched_entries
```python
convert_matched_entries(self)
```
convert_matched_entries(arg0: List[pydata_tools.xtf_data.xtf_sss_ping], arg1: List[pydata_tools.csv_data.csv_nav_entry]) -> List[pydata_tools.xtf_data.xtf_sss_ping]

Match xtf_sss_ping::PingsT and csv_nav_entry::EntriesT and assign pos info to pings

## csv_nav_entry
```python
csv_nav_entry(self)
```
Class for a csv based nav entry
### altitude
Member
### heading_
Member
### heading_std_
Member
### lat_
Member
### long_
Member
### parse_file
```python
csv_nav_entry.parse_file(self)
```
parse_file(arg0: unicode) -> List[pydata_tools.csv_data.csv_nav_entry]

Parse csv_nav_entry from .csv file

### parse_folder
```python
csv_nav_entry.parse_folder(self)
```
parse_folder(arg0: unicode) -> List[pydata_tools.csv_data.csv_nav_entry]

Parse csv_nav_entry from folder of .csv files

### pitch_
Member
### pitch_std_
Member
### pos_
Member
### read_data
```python
csv_nav_entry.read_data(self)
```
read_data(arg0: unicode) -> List[pydata_tools.csv_data.csv_nav_entry]

Read csv_nav_entry::EntriesT from .cereal file

### roll_
Member
### roll_std_
Member
### time_stamp_
Member
### time_string_
Member
### vel_
Member
## write_data
```python
write_data(self)
```
write_data(arg0: List[pydata_tools.csv_data.csv_nav_entry], arg1: unicode) -> None

Write csv_nav_entry::EntriesT to .cereal file

