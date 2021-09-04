subdir('include/gnuradio/atsc')

atsc_sources = []
atsc_cu_sources = []
atsc_pybind_sources = []
atsc_pybind_names = []
atsc_deps = []

# Individual block subdirectories
#  Copy block as an example
subdir('deinterleaver')
subdir('derandomizer')
subdir('equalizer')
subdir('fpll')
subdir('fs_checker')
subdir('rs_decoder')
subdir('sync')
subdir('viterbi_decoder')

subdir('lib')
if (get_option('enable_python'))
    subdir('python/newsched/atsc')
endif

subdir('test')
